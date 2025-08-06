import { containerLifecycleService } from './ContainerLifecycleService';
import { workspaceMountingService } from './WorkspaceMountingService';
import { logger } from '../utils/logger';

export interface CleanupPolicy {
  id: string;
  name: string;
  description: string;
  containerStates: string[];
  idleThresholdMinutes: number;
  gracePeriodMinutes: number;
  maxContainersPerUser: number;
  maxContainersPerProject: number;
  enabled: boolean;
  priority: 'low' | 'medium' | 'high' | 'critical';
}

export interface CleanupJob {
  id: string;
  policyId: string;
  status: 'pending' | 'running' | 'completed' | 'failed' | 'cancelled';
  startedAt?: string;
  completedAt?: string;
  containersProcessed: number;
  containersCleaned: number;
  errors: string[];
  userId?: string;
  projectId?: string;
}

export interface IdleContainer {
  containerId: string;
  containerName: string;
  userId: string;
  projectId: string;
  idleSince: string;
  idleDurationMinutes: number;
  resourceUsage: {
    cpu: number;
    memory: number;
    memoryLimit: number;
  };
  lastActivity: string;
  state: string;
}

export interface CleanupNotification {
  id: string;
  userId: string;
  type: 'warning' | 'cleanup' | 'error';
  title: string;
  message: string;
  containerIds: string[];
  timestamp: string;
  read: boolean;
  actionRequired: boolean;
}

export class AutomatedCleanupService {
  private cleanupPolicies: Map<string, CleanupPolicy> = new Map();
  private cleanupJobs: Map<string, CleanupJob> = new Map();
  private notifications: CleanupNotification[] = [];
  private scheduledJobs: Map<string, NodeJS.Timeout> = new Map();
  private isRunning: boolean = false;

  constructor() {
    this.initializeDefaultPolicies();
  }

  /**
   * Initialize default cleanup policies
   */
  private initializeDefaultPolicies(): void {
    const defaultPolicies: CleanupPolicy[] = [
      {
        id: 'idle_containers',
        name: 'Idle Container Cleanup',
        description: 'Clean up containers that have been idle for extended periods',
        containerStates: ['running', 'stopped'],
        idleThresholdMinutes: 60, // 1 hour
        gracePeriodMinutes: 15, // 15 minutes
        maxContainersPerUser: 10,
        maxContainersPerProject: 5,
        enabled: true,
        priority: 'medium',
      },
      {
        id: 'orphaned_containers',
        name: 'Orphaned Container Cleanup',
        description: 'Clean up containers without active projects',
        containerStates: ['stopped', 'error'],
        idleThresholdMinutes: 1440, // 24 hours
        gracePeriodMinutes: 60, // 1 hour
        maxContainersPerUser: 5,
        maxContainersPerProject: 3,
        enabled: true,
        priority: 'high',
      },
      {
        id: 'resource_intensive',
        name: 'Resource Intensive Container Cleanup',
        description: 'Clean up containers using excessive resources',
        containerStates: ['running'],
        idleThresholdMinutes: 30, // 30 minutes
        gracePeriodMinutes: 10, // 10 minutes
        maxContainersPerUser: 3,
        maxContainersPerProject: 2,
        enabled: true,
        priority: 'critical',
      },
    ];

    defaultPolicies.forEach(policy => {
      this.cleanupPolicies.set(policy.id, policy);
    });

    logger.info(`Initialized ${defaultPolicies.length} default cleanup policies`);
  }

  /**
   * Start the automated cleanup service
   */
  async startService(): Promise<void> {
    if (this.isRunning) {
      logger.warn('Automated cleanup service is already running');
      return;
    }

    this.isRunning = true;
    logger.info('Starting automated cleanup service');

    // Schedule periodic cleanup jobs
    this.schedulePeriodicCleanup();

    // Start monitoring for idle containers
    this.startIdleContainerMonitoring();
  }

  /**
   * Stop the automated cleanup service
   */
  async stopService(): Promise<void> {
    if (!this.isRunning) {
      logger.warn('Automated cleanup service is not running');
      return;
    }

    this.isRunning = false;
    logger.info('Stopping automated cleanup service');

    // Clear all scheduled jobs
    this.scheduledJobs.forEach(timeout => clearTimeout(timeout));
    this.scheduledJobs.clear();
  }

  /**
   * Schedule periodic cleanup jobs
   */
  private schedulePeriodicCleanup(): void {
    // Run cleanup every 30 minutes
    const cleanupInterval = 30 * 60 * 1000; // 30 minutes

    const scheduleNextCleanup = () => {
      if (!this.isRunning) return;

      const timeout = setTimeout(async () => {
        try {
          await this.runPeriodicCleanup();
        } catch (error) {
          logger.error('Periodic cleanup failed:', undefined, error as Error);
        } finally {
          scheduleNextCleanup();
        }
      }, cleanupInterval);

      this.scheduledJobs.set('periodic_cleanup', timeout);
    };

    scheduleNextCleanup();
  }

  /**
   * Start monitoring for idle containers
   */
  private startIdleContainerMonitoring(): void {
    // Check for idle containers every 10 minutes
    const monitoringInterval = 10 * 60 * 1000; // 10 minutes

    const scheduleNextMonitoring = () => {
      if (!this.isRunning) return;

      const timeout = setTimeout(async () => {
        try {
          await this.detectIdleContainers();
        } catch (error) {
          logger.error('Idle container detection failed:', undefined, error as Error);
        } finally {
          scheduleNextMonitoring();
        }
      }, monitoringInterval);

      this.scheduledJobs.set('idle_monitoring', timeout);
    };

    scheduleNextMonitoring();
  }

  /**
   * Run periodic cleanup based on all enabled policies
   */
  async runPeriodicCleanup(): Promise<void> {
    logger.info('Starting periodic cleanup run');

    const enabledPolicies = Array.from(this.cleanupPolicies.values()).filter(
      policy => policy.enabled
    );

    for (const policy of enabledPolicies) {
      try {
        await this.runCleanupJob(policy);
      } catch (error) {
        logger.error(`Cleanup job failed for policy ${policy.id}:`, undefined, error as Error);
      }
    }

    logger.info('Periodic cleanup run completed');
  }

  /**
   * Run a specific cleanup job
   */
  async runCleanupJob(policy: CleanupPolicy): Promise<CleanupJob> {
    const jobId = `job_${policy.id}_${Date.now()}`;
    
    const job: CleanupJob = {
      id: jobId,
      policyId: policy.id,
      status: 'running',
      startedAt: new Date().toISOString(),
      containersProcessed: 0,
      containersCleaned: 0,
      errors: [],
    };

    this.cleanupJobs.set(jobId, job);

    try {
      logger.info(`Starting cleanup job ${jobId} for policy ${policy.name}`);

      // Get containers that match the policy criteria
      const containers = await this.getContainersForPolicy(policy);
      job.containersProcessed = containers.length;

      // Process each container
      for (const container of containers) {
        try {
          await this.processContainerForCleanup(container, policy, job);
        } catch (error) {
          const errorMessage = error instanceof Error ? error.message : 'Unknown error';
          job.errors.push(`Container ${container.containerName}: ${errorMessage}`);
          logger.error(`Failed to process container ${container.containerName}:`, undefined, error as Error);
        }
      }

      job.status = 'completed';
      job.completedAt = new Date().toISOString();

      logger.info(`Cleanup job ${jobId} completed: ${job.containersCleaned}/${job.containersProcessed} containers cleaned`);

    } catch (error) {
      job.status = 'failed';
      job.completedAt = new Date().toISOString();
      job.errors.push(error instanceof Error ? error.message : 'Unknown error');
      
      logger.error(`Cleanup job ${jobId} failed:`, undefined, error as Error);
    }

    this.cleanupJobs.set(jobId, job);
    return job;
  }

  /**
   * Get containers that match a cleanup policy
   */
  private async getContainersForPolicy(policy: CleanupPolicy): Promise<IdleContainer[]> {
    const idleContainers: IdleContainer[] = [];
    const now = new Date();

    // Get all containers from the lifecycle service
    const allContainers = containerLifecycleService.getServiceStats();
    
    // This would typically query the actual container states
    // For now, we'll simulate the process
    const containers = Array.from(containerLifecycleService['containerStates'].values())
      .filter(container => policy.containerStates.includes(container.status));

    for (const container of containers) {
      try {
        // Check if container is idle based on policy threshold
        const idleSince = new Date(container.startedAt || container.createdAt);
        const idleDurationMinutes = (now.getTime() - idleSince.getTime()) / (1000 * 60);

        if (idleDurationMinutes >= policy.idleThresholdMinutes) {
          // Get resource usage
          const resourceUsage = await containerLifecycleService.getContainerResourceUsage(container.name);

          idleContainers.push({
            containerId: container.id,
            containerName: container.name,
            userId: container.userId,
            projectId: container.projectId,
            idleSince: idleSince.toISOString(),
            idleDurationMinutes,
            resourceUsage: resourceUsage || { cpu: 0, memory: 0, memoryLimit: 0 },
            lastActivity: container.startedAt || container.createdAt,
            state: container.status,
          });
        }
      } catch (error) {
        logger.warn(`Failed to check container ${container.name} for idle status: ${error}`);
      }
    }

    return idleContainers;
  }

  /**
   * Process a container for cleanup
   */
  private async processContainerForCleanup(
    idleContainer: IdleContainer,
    policy: CleanupPolicy,
    job: CleanupJob
  ): Promise<void> {
    // Check grace period
    const gracePeriodExpiry = new Date(idleContainer.idleSince);
    gracePeriodExpiry.setMinutes(gracePeriodExpiry.getMinutes() + policy.gracePeriodMinutes);

    if (new Date() < gracePeriodExpiry) {
      // Still in grace period, send warning notification
      await this.sendCleanupWarning(idleContainer, policy);
      return;
    }

    // Check if we've exceeded limits
    const userContainers = containerLifecycleService.getUserContainers(idleContainer.userId);
    const projectContainers = containerLifecycleService.getProjectContainers(
      idleContainer.userId,
      idleContainer.projectId
    );

    if (userContainers.length <= policy.maxContainersPerUser &&
        projectContainers.length <= policy.maxContainersPerProject) {
      // Within limits, no cleanup needed
      return;
    }

    // Perform cleanup
    try {
      // Stop the container if it's running
      if (idleContainer.state === 'running') {
        await containerLifecycleService.stopContainer(idleContainer.containerName);
      }

      // Destroy the container
      await containerLifecycleService.destroyContainer(idleContainer.containerName);

      job.containersCleaned++;

      // Send cleanup notification
      await this.sendCleanupNotification(idleContainer, policy);

      logger.info(`Cleaned up idle container: ${idleContainer.containerName}`);

    } catch (error) {
      throw new Error(`Failed to cleanup container ${idleContainer.containerName}: ${error}`);
    }
  }

  /**
   * Detect idle containers and send warnings
   */
  async detectIdleContainers(): Promise<void> {
    logger.info('Detecting idle containers');

    const enabledPolicies = Array.from(this.cleanupPolicies.values()).filter(
      policy => policy.enabled
    );

    for (const policy of enabledPolicies) {
      try {
        const idleContainers = await this.getContainersForPolicy(policy);
        
        for (const container of idleContainers) {
          // Check if we should send a warning (approaching threshold)
          const warningThreshold = policy.idleThresholdMinutes - policy.gracePeriodMinutes;
          
          if (container.idleDurationMinutes >= warningThreshold && 
              container.idleDurationMinutes < policy.idleThresholdMinutes) {
            await this.sendCleanupWarning(container, policy);
          }
        }
      } catch (error) {
        logger.error(`Failed to detect idle containers for policy ${policy.id}:`, undefined, error as Error);
      }
    }
  }

  /**
   * Send cleanup warning notification
   */
  private async sendCleanupWarning(idleContainer: IdleContainer, policy: CleanupPolicy): Promise<void> {
    const notification: CleanupNotification = {
      id: `warning_${idleContainer.containerId}_${Date.now()}`,
      userId: idleContainer.userId,
      type: 'warning',
      title: 'Container Cleanup Warning',
      message: `Container ${idleContainer.containerName} has been idle for ${Math.round(idleContainer.idleDurationMinutes)} minutes and will be cleaned up in ${policy.gracePeriodMinutes} minutes if no activity is detected.`,
      containerIds: [idleContainer.containerId],
      timestamp: new Date().toISOString(),
      read: false,
      actionRequired: true,
    };

    this.notifications.push(notification);
    logger.info(`Sent cleanup warning for container: ${idleContainer.containerName}`);
  }

  /**
   * Send cleanup notification
   */
  private async sendCleanupNotification(idleContainer: IdleContainer, policy: CleanupPolicy): Promise<void> {
    const notification: CleanupNotification = {
      id: `cleanup_${idleContainer.containerId}_${Date.now()}`,
      userId: idleContainer.userId,
      type: 'cleanup',
      title: 'Container Cleaned Up',
      message: `Container ${idleContainer.containerName} has been automatically cleaned up due to inactivity (idle for ${Math.round(idleContainer.idleDurationMinutes)} minutes).`,
      containerIds: [idleContainer.containerId],
      timestamp: new Date().toISOString(),
      read: false,
      actionRequired: false,
    };

    this.notifications.push(notification);
    logger.info(`Sent cleanup notification for container: ${idleContainer.containerName}`);
  }

  /**
   * Get cleanup policies
   */
  getCleanupPolicies(): CleanupPolicy[] {
    return Array.from(this.cleanupPolicies.values());
  }

  /**
   * Add or update a cleanup policy
   */
  setCleanupPolicy(policy: CleanupPolicy): void {
    this.cleanupPolicies.set(policy.id, policy);
    logger.info(`Updated cleanup policy: ${policy.name}`);
  }

  /**
   * Remove a cleanup policy
   */
  removeCleanupPolicy(policyId: string): boolean {
    const removed = this.cleanupPolicies.delete(policyId);
    if (removed) {
      logger.info(`Removed cleanup policy: ${policyId}`);
    }
    return removed;
  }

  /**
   * Get cleanup jobs
   */
  getCleanupJobs(options?: {
    status?: CleanupJob['status'];
    userId?: string;
    limit?: number;
  }): CleanupJob[] {
    let jobs = Array.from(this.cleanupJobs.values());

    if (options?.status) {
      jobs = jobs.filter(job => job.status === options.status);
    }

    if (options?.userId) {
      // Filter jobs by user (this would require additional logic to track user-specific jobs)
      jobs = jobs.filter(job => job.userId === options.userId);
    }

    // Sort by startedAt (newest first)
    jobs.sort((a, b) => {
      const aTime = a.startedAt ? new Date(a.startedAt).getTime() : 0;
      const bTime = b.startedAt ? new Date(b.startedAt).getTime() : 0;
      return bTime - aTime;
    });

    if (options?.limit) {
      jobs = jobs.slice(0, options.limit);
    }

    return jobs;
  }

  /**
   * Get notifications for a user
   */
  getUserNotifications(userId: string, options?: {
    unreadOnly?: boolean;
    limit?: number;
  }): CleanupNotification[] {
    let notifications = this.notifications.filter(n => n.userId === userId);

    if (options?.unreadOnly) {
      notifications = notifications.filter(n => !n.read);
    }

    // Sort by timestamp (newest first)
    notifications.sort((a, b) => new Date(b.timestamp).getTime() - new Date(a.timestamp).getTime());

    if (options?.limit) {
      notifications = notifications.slice(0, options.limit);
    }

    return notifications;
  }

  /**
   * Mark notification as read
   */
  markNotificationAsRead(notificationId: string): boolean {
    const notification = this.notifications.find(n => n.id === notificationId);
    if (notification) {
      notification.read = true;
      return true;
    }
    return false;
  }

  /**
   * Get service statistics
   */
  getServiceStats(): {
    isRunning: boolean;
    totalPolicies: number;
    enabledPolicies: number;
    totalJobs: number;
    totalNotifications: number;
    scheduledJobs: number;
  } {
    const enabledPolicies = Array.from(this.cleanupPolicies.values()).filter(p => p.enabled);

    return {
      isRunning: this.isRunning,
      totalPolicies: this.cleanupPolicies.size,
      enabledPolicies: enabledPolicies.length,
      totalJobs: this.cleanupJobs.size,
      totalNotifications: this.notifications.length,
      scheduledJobs: this.scheduledJobs.size,
    };
  }

  /**
   * Cleanup old notifications
   */
  cleanupOldNotifications(maxAgeDays: number = 7): void {
    const cutoffTime = new Date(Date.now() - maxAgeDays * 24 * 60 * 60 * 1000);
    const beforeCount = this.notifications.length;
    
    this.notifications = this.notifications.filter(
      notification => new Date(notification.timestamp) > cutoffTime
    );

    const afterCount = this.notifications.length;
    logger.info(`Cleaned up ${beforeCount - afterCount} old notifications`);
  }

  /**
   * Cleanup old jobs
   */
  cleanupOldJobs(maxAgeDays: number = 30): void {
    const cutoffTime = new Date(Date.now() - maxAgeDays * 24 * 60 * 60 * 1000);
    const beforeCount = this.cleanupJobs.size;
    
    for (const [jobId, job] of this.cleanupJobs.entries()) {
      if (job.completedAt && new Date(job.completedAt) < cutoffTime) {
        this.cleanupJobs.delete(jobId);
      }
    }

    const afterCount = this.cleanupJobs.size;
    logger.info(`Cleaned up ${beforeCount - afterCount} old jobs`);
  }
}

// Export singleton instance
export const automatedCleanupService = new AutomatedCleanupService(); 