import { dockerService } from './DockerService';
import { logger } from '../utils/logger';

export interface ContainerName {
  fullName: string;
  userId: string;
  projectId: string;
  prefix: string;
  timestamp?: string;
  suffix?: string;
}

export interface NamingValidationResult {
  isValid: boolean;
  errors: string[];
  warnings: string[];
}

export interface CollisionResolution {
  originalName: string;
  resolvedName: string;
  collisionType: 'exact' | 'similar' | 'none';
  resolutionMethod: 'timestamp' | 'suffix' | 'none';
}

export class ContainerNamingService {
  private readonly PREFIX = 'robium';
  private readonly MAX_NAME_LENGTH = 63; // Docker container name limit
  private readonly VALID_CHARS_REGEX = /^[a-zA-Z0-9_-]+$/;
  private readonly RESERVED_NAMES = ['docker', 'containerd', 'k8s', 'kubernetes'];

  /**
   * Generate container name using the robium_{user_id}_{project_id} convention
   */
  generateContainerName(userId: string, projectId: string, options?: {
    timestamp?: boolean;
    suffix?: string;
  }): ContainerName {
    const timestamp = options?.timestamp ? `_${Date.now()}` : '';
    const suffix = options?.suffix ? `_${options.suffix}` : '';
    
    const fullName = `${this.PREFIX}_${userId}_${projectId}${timestamp}${suffix}`;
    
    return {
      fullName,
      userId,
      projectId,
      prefix: this.PREFIX,
      timestamp: options?.timestamp ? Date.now().toString() : undefined,
      suffix: options?.suffix,
    };
  }

  /**
   * Validate container name format and constraints
   */
  validateContainerName(name: string): NamingValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Check length
    if (name.length > this.MAX_NAME_LENGTH) {
      errors.push(`Container name exceeds maximum length of ${this.MAX_NAME_LENGTH} characters`);
    }

    // Check for valid characters
    if (!this.VALID_CHARS_REGEX.test(name)) {
      errors.push('Container name contains invalid characters. Only alphanumeric, underscore, and hyphen are allowed');
    }

    // Check for reserved names
    const lowerName = name.toLowerCase();
    for (const reserved of this.RESERVED_NAMES) {
      if (lowerName.includes(reserved)) {
        warnings.push(`Container name contains reserved term: ${reserved}`);
      }
    }

    // Check for consecutive hyphens or underscores
    if (name.includes('__') || name.includes('--')) {
      warnings.push('Container name contains consecutive special characters');
    }

    // Check for leading/trailing special characters
    if (name.startsWith('-') || name.startsWith('_') || name.endsWith('-') || name.endsWith('_')) {
      errors.push('Container name cannot start or end with hyphen or underscore');
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
    };
  }

  /**
   * Parse container name to extract components
   */
  parseContainerName(containerName: string): ContainerName | null {
    const pattern = new RegExp(`^${this.PREFIX}_(.+?)_(.+?)(_\\d+)?(_[a-zA-Z0-9_-]+)?$`);
    const match = containerName.match(pattern);

    if (!match) {
      return null;
    }

    const [, userId, projectId, timestampMatch, suffixMatch] = match;
    const timestamp = timestampMatch ? timestampMatch.substring(1) : undefined;
    const suffix = suffixMatch ? suffixMatch.substring(1) : undefined;

    return {
      fullName: containerName,
      userId,
      projectId,
      prefix: this.PREFIX,
      timestamp,
      suffix,
    };
  }

  /**
   * Check for naming collisions and resolve them
   */
  async checkForCollisions(proposedName: string): Promise<CollisionResolution> {
    try {
      // Check if exact name exists
      const containers = await dockerService.listContainers(true);
      const exactMatch = containers.find(container => container.name === proposedName);

      if (exactMatch) {
        // Resolve with timestamp
        const timestamp = Date.now();
        const resolvedName = `${proposedName}_${timestamp}`;
        
        return {
          originalName: proposedName,
          resolvedName,
          collisionType: 'exact',
          resolutionMethod: 'timestamp',
        };
      }

      // Check for similar names (same user and project)
      const parsed = this.parseContainerName(proposedName);
      if (parsed) {
        const similarContainers = containers.filter(container => {
          const containerParsed = this.parseContainerName(container.name);
          return containerParsed && 
                 containerParsed.userId === parsed.userId && 
                 containerParsed.projectId === parsed.projectId;
        });

        if (similarContainers.length > 0) {
          // Resolve with suffix
          const suffix = `v${similarContainers.length + 1}`;
          const resolvedName = `${proposedName}_${suffix}`;
          
          return {
            originalName: proposedName,
            resolvedName,
            collisionType: 'similar',
            resolutionMethod: 'suffix',
          };
        }
      }

      return {
        originalName: proposedName,
        resolvedName: proposedName,
        collisionType: 'none',
        resolutionMethod: 'none',
      };
    } catch (error) {
      logger.error('Failed to check for naming collisions:', undefined, error as Error);
      throw error;
    }
  }

  /**
   * Generate unique container name with collision resolution
   */
  async generateUniqueContainerName(userId: string, projectId: string, options?: {
    timestamp?: boolean;
    suffix?: string;
    maxRetries?: number;
  }): Promise<ContainerName> {
    const maxRetries = options?.maxRetries || 5;
    let attempts = 0;

    while (attempts < maxRetries) {
      const containerName = this.generateContainerName(userId, projectId, options);
      const collision = await this.checkForCollisions(containerName.fullName);

      if (collision.collisionType === 'none') {
        return containerName;
      }

      // Update options for next attempt
      if (collision.resolutionMethod === 'timestamp') {
        options = { ...options, timestamp: true };
      } else if (collision.resolutionMethod === 'suffix') {
        const currentSuffix = options?.suffix || 'v1';
        const version = parseInt(currentSuffix.replace('v', '')) + 1;
        options = { ...options, suffix: `v${version}` };
      }

      attempts++;
    }

    throw new Error(`Failed to generate unique container name after ${maxRetries} attempts`);
  }

  /**
   * Get all containers for a specific user
   */
  async getUserContainers(userId: string): Promise<ContainerName[]> {
    try {
      const containers = await dockerService.listContainers(true);
      const userContainers: ContainerName[] = [];

      for (const container of containers) {
        const parsed = this.parseContainerName(container.name);
        if (parsed && parsed.userId === userId) {
          userContainers.push(parsed);
        }
      }

      return userContainers;
    } catch (error) {
      logger.error(`Failed to get containers for user ${userId}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Get all containers for a specific project
   */
  async getProjectContainers(userId: string, projectId: string): Promise<ContainerName[]> {
    try {
      const containers = await dockerService.listContainers(true);
      const projectContainers: ContainerName[] = [];

      for (const container of containers) {
        const parsed = this.parseContainerName(container.name);
        if (parsed && parsed.userId === userId && parsed.projectId === projectId) {
          projectContainers.push(parsed);
        }
      }

      return projectContainers;
    } catch (error) {
      logger.error(`Failed to get containers for project ${projectId}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Clean up container names (remove old containers)
   */
  async cleanupOldContainers(userId: string, projectId: string, keepLatest: number = 1): Promise<void> {
    try {
      const projectContainers = await this.getProjectContainers(userId, projectId);
      
      if (projectContainers.length <= keepLatest) {
        return;
      }

      // Sort by timestamp (newest first)
      const sortedContainers = projectContainers
        .filter(container => container.timestamp)
        .sort((a, b) => {
          const timestampA = parseInt(a.timestamp!);
          const timestampB = parseInt(b.timestamp!);
          return timestampB - timestampA;
        });

      // Remove old containers
      const containersToRemove = sortedContainers.slice(keepLatest);
      
      for (const container of containersToRemove) {
        try {
          await dockerService.removeContainer(container.fullName, true);
          logger.info(`Removed old container: ${container.fullName}`);
        } catch (error) {
          logger.warn(`Failed to remove old container ${container.fullName}: ${error}`);
        }
      }
    } catch (error) {
      logger.error(`Failed to cleanup old containers for project ${projectId}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Validate and sanitize user ID and project ID
   */
  sanitizeIdentifier(identifier: string): string {
    return identifier
      .toLowerCase()
      .replace(/[^a-z0-9_-]/g, '_')
      .replace(/_+/g, '_')
      .replace(/^_|_$/g, '');
  }

  /**
   * Generate container tags for metadata
   */
  generateContainerTags(userId: string, projectId: string, additionalTags?: Record<string, string>): Record<string, string> {
    const baseTags = {
      'robium.user_id': userId,
      'robium.project_id': projectId,
      'robium.created_at': new Date().toISOString(),
      'robium.version': '1.0.0',
    };

    return { ...baseTags, ...additionalTags };
  }
}

// Export singleton instance
export const containerNamingService = new ContainerNamingService(); 