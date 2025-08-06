import fs from 'fs';
import path from 'path';
import { promises as fsPromises } from 'fs';
import { logger } from '../utils/logger';

export interface WorkspaceConfig {
  userId: string;
  projectId: string;
  workspacePath: string;
  containerPath: string;
  permissions?: {
    read: boolean;
    write: boolean;
    execute: boolean;
  };
  syncMode?: 'realtime' | 'manual' | 'scheduled';
  backupEnabled?: boolean;
  backupInterval?: number; // in minutes
  maxBackups?: number;
}

export interface MountPoint {
  id: string;
  userId: string;
  projectId: string;
  hostPath: string;
  containerPath: string;
  permissions: string;
  status: 'mounted' | 'unmounted' | 'error';
  createdAt: string;
  lastSync?: string;
  lastBackup?: string;
  error?: string;
}

export interface BackupInfo {
  id: string;
  workspaceId: string;
  timestamp: string;
  path: string;
  size: number;
  compressionRatio?: number;
  status: 'completed' | 'failed' | 'in-progress';
  error?: string;
}

export interface SyncEvent {
  timestamp: string;
  type: 'file_created' | 'file_modified' | 'file_deleted' | 'directory_created' | 'directory_deleted';
  path: string;
  userId: string;
  projectId: string;
  details?: string;
}

export class WorkspaceMountingService {
  private mountPoints: Map<string, MountPoint> = new Map();
  private backups: Map<string, BackupInfo[]> = new Map();
  private syncEvents: SyncEvent[] = [];
  private maxSyncEvents: number = 1000;
  private baseWorkspaceDir: string;
  private backupDir: string;

  constructor() {
    this.baseWorkspaceDir = process.env.WORKSPACE_BASE_DIR || path.join(process.cwd(), 'generated', 'workspaces');
    this.backupDir = process.env.BACKUP_DIR || path.join(process.cwd(), 'generated', 'backups');
    this.initializeDirectories();
  }

  /**
   * Initialize required directories
   */
  private async initializeDirectories(): Promise<void> {
    try {
      await fsPromises.mkdir(this.baseWorkspaceDir, { recursive: true });
      await fsPromises.mkdir(this.backupDir, { recursive: true });
      logger.info('Workspace directories initialized successfully');
    } catch (error) {
      logger.error('Failed to initialize workspace directories:', undefined, error as Error);
      throw error;
    }
  }

  /**
   * Create workspace directory structure
   */
  async createWorkspace(config: WorkspaceConfig): Promise<MountPoint> {
    try {
      const workspaceId = `${config.userId}_${config.projectId}`;
      const hostPath = path.join(this.baseWorkspaceDir, workspaceId);
      const mountId = `mount_${workspaceId}_${Date.now()}`;

      // Create workspace directory
      await fsPromises.mkdir(hostPath, { recursive: true });

      // Set proper permissions
      const permissions = config.permissions || { read: true, write: true, execute: false };
      await this.setDirectoryPermissions(hostPath, permissions);

      // Create mount point record
      const mountPoint: MountPoint = {
        id: mountId,
        userId: config.userId,
        projectId: config.projectId,
        hostPath,
        containerPath: config.containerPath,
        permissions: this.formatPermissions(permissions),
        status: 'unmounted',
        createdAt: new Date().toISOString(),
      };

      this.mountPoints.set(mountId, mountPoint);

      logger.info(`Workspace created: ${hostPath} for user ${config.userId}, project ${config.projectId}`);
      return mountPoint;

    } catch (error) {
      logger.error(`Failed to create workspace for project ${config.projectId}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Generate Docker volume mount configuration
   */
  generateMountConfig(mountPoint: MountPoint): string {
    return `${mountPoint.hostPath}:${mountPoint.containerPath}`;
  }

  /**
   * Get all mount points for a user
   */
  getUserMountPoints(userId: string): MountPoint[] {
    return Array.from(this.mountPoints.values()).filter(
      mount => mount.userId === userId
    );
  }

  /**
   * Get mount points for a project
   */
  getProjectMountPoints(userId: string, projectId: string): MountPoint[] {
    return Array.from(this.mountPoints.values()).filter(
      mount => mount.userId === userId && mount.projectId === projectId
    );
  }

  /**
   * Get mount points by project ID only
   */
  getMountPointsByProjectId(projectId: string): MountPoint[] {
    return Array.from(this.mountPoints.values()).filter(
      mount => mount.projectId === projectId
    );
  }

  /**
   * Set directory permissions
   */
  private async setDirectoryPermissions(dirPath: string, permissions: { read: boolean; write: boolean; execute: boolean }): Promise<void> {
    try {
      let mode = 0;
      if (permissions.read) mode |= 0o444;
      if (permissions.write) mode |= 0o222;
      if (permissions.execute) mode |= 0o111;

      await fsPromises.chmod(dirPath, mode);
    } catch (error) {
      logger.error(`Failed to set permissions for ${dirPath}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Format permissions for display
   */
  private formatPermissions(permissions: { read: boolean; write: boolean; execute: boolean }): string {
    const parts = [];
    if (permissions.read) parts.push('r');
    if (permissions.write) parts.push('w');
    if (permissions.execute) parts.push('x');
    return parts.join('');
  }

  /**
   * Create backup of workspace
   */
  async createBackup(mountPointId: string): Promise<BackupInfo> {
    try {
      const mountPoint = this.mountPoints.get(mountPointId);
      if (!mountPoint) {
        throw new Error(`Mount point not found: ${mountPointId}`);
      }

      const backupId = `backup_${mountPoint.userId}_${mountPoint.projectId}_${Date.now()}`;
      const backupPath = path.join(this.backupDir, `${backupId}.tar.gz`);

      // Create backup info
      const backupInfo: BackupInfo = {
        id: backupId,
        workspaceId: mountPointId,
        timestamp: new Date().toISOString(),
        path: backupPath,
        size: 0,
        status: 'in-progress',
      };

      // Add to backups list
      if (!this.backups.has(mountPointId)) {
        this.backups.set(mountPointId, []);
      }
      this.backups.get(mountPointId)!.push(backupInfo);

      // Create tar.gz backup
      const { exec } = require('child_process');
      const tarCommand = `tar -czf "${backupPath}" -C "${path.dirname(mountPoint.hostPath)}" "${path.basename(mountPoint.hostPath)}"`;

      await new Promise<void>((resolve, reject) => {
        exec(tarCommand, (error: any, stdout: any, stderr: any) => {
          if (error) {
            backupInfo.status = 'failed';
            backupInfo.error = error.message;
            reject(error);
          } else {
            // Get backup size
            fs.stat(backupPath, (err, stats) => {
              if (!err) {
                backupInfo.size = stats.size;
              }
              backupInfo.status = 'completed';
              resolve();
            });
          }
        });
      });

      // Update mount point
      mountPoint.lastBackup = backupInfo.timestamp;
      this.mountPoints.set(mountPointId, mountPoint);

      // Cleanup old backups
      await this.cleanupOldBackups(mountPointId);

      logger.info(`Backup created: ${backupPath} (${backupInfo.size} bytes)`);
      return backupInfo;

    } catch (error) {
      logger.error(`Failed to create backup for mount point ${mountPointId}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Restore workspace from backup
   */
  async restoreFromBackup(mountPointId: string, backupId: string): Promise<void> {
    try {
      const mountPoint = this.mountPoints.get(mountPointId);
      if (!mountPoint) {
        throw new Error(`Mount point not found: ${mountPointId}`);
      }

      const backup = this.backups.get(mountPointId)?.find(b => b.id === backupId);
      if (!backup) {
        throw new Error(`Backup not found: ${backupId}`);
      }

      // Stop any running containers that might be using the workspace
      // This would typically be handled by the container lifecycle service

      // Restore from backup
      const { exec } = require('child_process');
      const restoreCommand = `tar -xzf "${backup.path}" -C "${path.dirname(mountPoint.hostPath)}"`;

      await new Promise<void>((resolve, reject) => {
        exec(restoreCommand, (error: any, stdout: any, stderr: any) => {
          if (error) {
            reject(error);
          } else {
            resolve();
          }
        });
      });

      logger.info(`Workspace restored from backup: ${backup.path}`);

    } catch (error) {
      logger.error(`Failed to restore workspace from backup ${backupId}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Get backup history for a mount point
   */
  getBackupHistory(mountPointId: string): BackupInfo[] {
    return this.backups.get(mountPointId) || [];
  }

  /**
   * Cleanup old backups
   */
  private async cleanupOldBackups(mountPointId: string): Promise<void> {
    try {
      const backups = this.backups.get(mountPointId);
      if (!backups) return;

      const mountPoint = this.mountPoints.get(mountPointId);
      if (!mountPoint) return;

      const maxBackups = 5; // Default max backups
      if (backups.length <= maxBackups) return;

      // Sort by timestamp (oldest first)
      backups.sort((a, b) => new Date(a.timestamp).getTime() - new Date(b.timestamp).getTime());

      // Remove old backups
      const backupsToRemove = backups.slice(0, backups.length - maxBackups);
      for (const backup of backupsToRemove) {
        try {
          await fsPromises.unlink(backup.path);
          logger.info(`Removed old backup: ${backup.path}`);
        } catch (error) {
          logger.warn(`Failed to remove old backup ${backup.path}: ${error}`);
        }
      }

      // Update backups list
      this.backups.set(mountPointId, backups.slice(backups.length - maxBackups));

    } catch (error) {
      logger.error(`Failed to cleanup old backups for mount point ${mountPointId}:`, undefined, error as Error);
    }
  }

  /**
   * Monitor file changes in workspace
   */
  async startFileMonitoring(mountPointId: string): Promise<void> {
    try {
      const mountPoint = this.mountPoints.get(mountPointId);
      if (!mountPoint) {
        throw new Error(`Mount point not found: ${mountPointId}`);
      }

      // Set up file watcher
      const watcher = fs.watch(mountPoint.hostPath, { recursive: true }, (eventType, filename) => {
        if (filename) {
          const filePath = path.join(mountPoint.hostPath, filename);
          this.logSyncEvent({
            timestamp: new Date().toISOString(),
            type: this.mapEventType(eventType),
            path: filePath,
            userId: mountPoint.userId,
            projectId: mountPoint.projectId,
            details: `File ${eventType}: ${filename}`,
          });

          // Update last sync time
          mountPoint.lastSync = new Date().toISOString();
          this.mountPoints.set(mountPointId, mountPoint);
        }
      });

      // Store watcher reference (in a real implementation, you'd want to manage this properly)
      logger.info(`File monitoring started for workspace: ${mountPoint.hostPath}`);

    } catch (error) {
      logger.error(`Failed to start file monitoring for mount point ${mountPointId}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Map file system event types to sync event types
   */
  private mapEventType(eventType: string): SyncEvent['type'] {
    switch (eventType) {
      case 'rename':
        return 'file_deleted';
      case 'change':
        return 'file_modified';
      default:
        return 'file_modified';
    }
  }

  /**
   * Log sync event
   */
  private logSyncEvent(event: SyncEvent): void {
    this.syncEvents.push(event);
    
    // Maintain event limit
    if (this.syncEvents.length > this.maxSyncEvents) {
      this.syncEvents = this.syncEvents.slice(-this.maxSyncEvents);
    }
  }

  /**
   * Get sync events
   */
  getSyncEvents(options?: {
    userId?: string;
    projectId?: string;
    limit?: number;
  }): SyncEvent[] {
    let events = [...this.syncEvents];

    // Filter by user
    if (options?.userId) {
      events = events.filter(event => event.userId === options.userId);
    }

    // Filter by project
    if (options?.projectId) {
      events = events.filter(event => event.projectId === options.projectId);
    }

    // Sort by timestamp (newest first)
    events.sort((a, b) => new Date(b.timestamp).getTime() - new Date(a.timestamp).getTime());

    // Apply limit
    if (options?.limit) {
      events = events.slice(0, options.limit);
    }

    return events;
  }

  /**
   * Validate workspace path security
   */
  validateWorkspacePath(workspacePath: string): boolean {
    try {
      // Ensure path is within base workspace directory
      const resolvedPath = path.resolve(workspacePath);
      const basePath = path.resolve(this.baseWorkspaceDir);
      
      // Check if the resolved path starts with the base path
      const isValid = resolvedPath.startsWith(basePath);
      
      logger.debug(`Path validation: ${workspacePath} -> ${resolvedPath}, base: ${basePath}, valid: ${isValid}`);
      
      return isValid;
    } catch (error) {
      logger.error(`Path validation error for ${workspacePath}:`, undefined, error as Error);
      return false;
    }
  }

  /**
   * Get workspace statistics
   */
  async getWorkspaceStats(mountPointId: string): Promise<{
    totalFiles: number;
    totalSize: number;
    lastModified: string;
    directoryCount: number;
  }> {
    try {
      const mountPoint = this.mountPoints.get(mountPointId);
      if (!mountPoint) {
        throw new Error(`Mount point not found: ${mountPointId}`);
      }

      const stats = await this.calculateDirectoryStats(mountPoint.hostPath);
      return stats;

    } catch (error) {
      logger.error(`Failed to get workspace stats for mount point ${mountPointId}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Calculate directory statistics
   */
  private async calculateDirectoryStats(dirPath: string): Promise<{
    totalFiles: number;
    totalSize: number;
    lastModified: string;
    directoryCount: number;
  }> {
    let totalFiles = 0;
    let totalSize = 0;
    let directoryCount = 0;
    let lastModified = new Date(0);

    const processDirectory = async (currentPath: string): Promise<void> => {
      const entries = await fsPromises.readdir(currentPath, { withFileTypes: true });
      
      for (const entry of entries) {
        const fullPath = path.join(currentPath, entry.name);
        
        if (entry.isDirectory()) {
          directoryCount++;
          await processDirectory(fullPath);
        } else if (entry.isFile()) {
          totalFiles++;
          const stats = await fsPromises.stat(fullPath);
          totalSize += stats.size;
          if (stats.mtime > lastModified) {
            lastModified = stats.mtime;
          }
        }
      }
    };

    await processDirectory(dirPath);

    return {
      totalFiles,
      totalSize,
      lastModified: lastModified.toISOString(),
      directoryCount,
    };
  }

  /**
   * Cleanup old sync events
   */
  cleanupOldSyncEvents(maxAgeHours: number = 24): void {
    const cutoffTime = new Date(Date.now() - maxAgeHours * 60 * 60 * 1000);
    this.syncEvents = this.syncEvents.filter(
      event => new Date(event.timestamp) > cutoffTime
    );
  }

  /**
   * Get service statistics
   */
  getServiceStats(): {
    totalMountPoints: number;
    activeMountPoints: number;
    totalBackups: number;
    totalSyncEvents: number;
  } {
    const mountPoints = Array.from(this.mountPoints.values());
    const totalBackups = Array.from(this.backups.values()).reduce(
      (sum, backups) => sum + backups.length, 0
    );

    return {
      totalMountPoints: mountPoints.length,
      activeMountPoints: mountPoints.filter(m => m.status === 'mounted').length,
      totalBackups,
      totalSyncEvents: this.syncEvents.length,
    };
  }
}

// Export singleton instance
export const workspaceMountingService = new WorkspaceMountingService(); 