import { dockerService, ContainerConfig } from './DockerService';
import { containerNamingService, ContainerName } from './ContainerNamingService';
import { logger } from '../utils/logger';

export interface ContainerState {
  id: string;
  name: string;
  status: 'creating' | 'running' | 'stopped' | 'destroyed' | 'error';
  createdAt: string;
  startedAt?: string;
  stoppedAt?: string;
  destroyedAt?: string;
  error?: string;
  userId: string;
  projectId: string;
  image: string;
  ports: { [key: string]: string };
  volumes: string[];
  environment: string[];
  resourceUsage?: {
    cpu: number;
    memory: number;
    memoryLimit: number;
  };
}

export interface LifecycleEvent {
  timestamp: string;
  eventType: 'created' | 'started' | 'stopped' | 'destroyed' | 'error';
  containerId: string;
  containerName: string;
  userId: string;
  projectId: string;
  details?: string;
  error?: string;
}

export interface ProjectContainerConfig {
  userId: string;
  projectId: string;
  image: string;
  command?: string[];
  environment?: string[];
  ports?: { [key: string]: string };
  volumes?: string[];
  workingDir?: string;
  user?: string;
  resourceLimits?: {
    cpuShares?: number;
    memory?: number;
    memorySwap?: number;
  };
  labels?: Record<string, string>;
  restartPolicy?: string;
}

export class ContainerLifecycleService {
  private containerStates: Map<string, ContainerState> = new Map();
  private lifecycleEvents: LifecycleEvent[] = [];
  private maxEvents: number = 1000;

  /**
   * Create a new container for a project
   */
  async createContainer(config: ProjectContainerConfig): Promise<ContainerState> {
    const startTime = Date.now();
    let containerName: ContainerName | null = null;
    
    try {
      // Generate unique container name
      containerName = await containerNamingService.generateUniqueContainerName(
        config.userId,
        config.projectId
      );

      // Create container tags
      const tags = containerNamingService.generateContainerTags(
        config.userId,
        config.projectId,
        config.labels
      );

      // Prepare Docker container configuration
      const dockerConfig: ContainerConfig = {
        name: containerName.fullName,
        image: config.image,
        command: config.command,
        env: config.environment,
        volumes: config.volumes,
        ports: config.ports,
        workingDir: config.workingDir,
        user: config.user,
        labels: tags,
        restartPolicy: config.restartPolicy,
        cpuShares: config.resourceLimits?.cpuShares,
        memory: config.resourceLimits?.memory,
        memorySwap: config.resourceLimits?.memorySwap,
      };

      // Create container state
      const containerState: ContainerState = {
        id: '',
        name: containerName.fullName,
        status: 'creating',
        createdAt: new Date().toISOString(),
        userId: config.userId,
        projectId: config.projectId,
        image: config.image,
        ports: config.ports || {},
        volumes: config.volumes || [],
        environment: config.environment || [],
      };

      // Store initial state
      this.containerStates.set(containerName.fullName, containerState);

      // Create container via Docker API
      const containerId = await dockerService.createContainer(dockerConfig);
      
      // Update state with container ID
      containerState.id = containerId;
      this.containerStates.set(containerName.fullName, containerState);

      // Log lifecycle event
      this.logLifecycleEvent({
        timestamp: new Date().toISOString(),
        eventType: 'created',
        containerId,
        containerName: containerName.fullName,
        userId: config.userId,
        projectId: config.projectId,
        details: `Container created in ${Date.now() - startTime}ms`,
      });

      logger.info(`Container created successfully: ${containerName.fullName} (${containerId})`);
      return containerState;

    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
      
      // Update state with error
      const containerState = this.containerStates.get(containerName?.fullName || 'unknown');
      if (containerState) {
        containerState.status = 'error';
        containerState.error = errorMessage;
        this.containerStates.set(containerState.name, containerState);
      }

      // Log error event
      this.logLifecycleEvent({
        timestamp: new Date().toISOString(),
        eventType: 'error',
        containerId: 'unknown',
        containerName: containerName?.fullName || 'unknown',
        userId: config.userId,
        projectId: config.projectId,
        error: errorMessage,
        details: 'Container creation failed',
      });

      logger.error(`Failed to create container for project ${config.projectId}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Start a container
   */
  async startContainer(containerName: string): Promise<ContainerState> {
    const startTime = Date.now();
    
    try {
      // Get container state
      const containerState = this.containerStates.get(containerName);
      if (!containerState) {
        throw new Error(`Container state not found: ${containerName}`);
      }

      // Update state
      containerState.status = 'creating';
      this.containerStates.set(containerName, containerState);

      // Start container via Docker API
      await dockerService.startContainer(containerState.id);

      // Update state
      containerState.status = 'running';
      containerState.startedAt = new Date().toISOString();
      this.containerStates.set(containerName, containerState);

      // Log lifecycle event
      this.logLifecycleEvent({
        timestamp: new Date().toISOString(),
        eventType: 'started',
        containerId: containerState.id,
        containerName,
        userId: containerState.userId,
        projectId: containerState.projectId,
        details: `Container started in ${Date.now() - startTime}ms`,
      });

      logger.info(`Container started successfully: ${containerName}`);
      return containerState;

    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
      
      // Update state with error
      const containerState = this.containerStates.get(containerName);
      if (containerState) {
        containerState.status = 'error';
        containerState.error = errorMessage;
        this.containerStates.set(containerName, containerState);
      }

      // Log error event
      this.logLifecycleEvent({
        timestamp: new Date().toISOString(),
        eventType: 'error',
        containerId: containerState?.id || 'unknown',
        containerName,
        userId: containerState?.userId || 'unknown',
        projectId: containerState?.projectId || 'unknown',
        error: errorMessage,
        details: 'Container start failed',
      });

      logger.error(`Failed to start container ${containerName}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Stop a container
   */
  async stopContainer(containerName: string, timeout: number = 10): Promise<ContainerState> {
    const startTime = Date.now();
    
    try {
      // Get container state
      const containerState = this.containerStates.get(containerName);
      if (!containerState) {
        throw new Error(`Container state not found: ${containerName}`);
      }

      // Update state
      containerState.status = 'stopped';
      this.containerStates.set(containerName, containerState);

      // Stop container via Docker API
      await dockerService.stopContainer(containerState.id, timeout);

      // Update state
      containerState.stoppedAt = new Date().toISOString();
      this.containerStates.set(containerName, containerState);

      // Log lifecycle event
      this.logLifecycleEvent({
        timestamp: new Date().toISOString(),
        eventType: 'stopped',
        containerId: containerState.id,
        containerName,
        userId: containerState.userId,
        projectId: containerState.projectId,
        details: `Container stopped in ${Date.now() - startTime}ms`,
      });

      logger.info(`Container stopped successfully: ${containerName}`);
      return containerState;

    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
      
      // Update state with error
      const containerState = this.containerStates.get(containerName);
      if (containerState) {
        containerState.status = 'error';
        containerState.error = errorMessage;
        this.containerStates.set(containerName, containerState);
      }

      // Log error event
      this.logLifecycleEvent({
        timestamp: new Date().toISOString(),
        eventType: 'error',
        containerId: containerState?.id || 'unknown',
        containerName,
        userId: containerState?.userId || 'unknown',
        projectId: containerState?.projectId || 'unknown',
        error: errorMessage,
        details: 'Container stop failed',
      });

      logger.error(`Failed to stop container ${containerName}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Destroy a container
   */
  async destroyContainer(containerName: string, force: boolean = false): Promise<void> {
    const startTime = Date.now();
    
    try {
      // Get container state
      const containerState = this.containerStates.get(containerName);
      if (!containerState) {
        throw new Error(`Container state not found: ${containerName}`);
      }

      // Update state
      containerState.status = 'destroyed';
      this.containerStates.set(containerName, containerState);

      // Remove container via Docker API
      await dockerService.removeContainer(containerState.id, force);

      // Update state
      containerState.destroyedAt = new Date().toISOString();
      this.containerStates.set(containerName, containerState);

      // Log lifecycle event
      this.logLifecycleEvent({
        timestamp: new Date().toISOString(),
        eventType: 'destroyed',
        containerId: containerState.id,
        containerName,
        userId: containerState.userId,
        projectId: containerState.projectId,
        details: `Container destroyed in ${Date.now() - startTime}ms`,
      });

      logger.info(`Container destroyed successfully: ${containerName}`);

    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
      
      // Update state with error
      const containerState = this.containerStates.get(containerName);
      if (containerState) {
        containerState.status = 'error';
        containerState.error = errorMessage;
        this.containerStates.set(containerName, containerState);
      }

      // Log error event
      this.logLifecycleEvent({
        timestamp: new Date().toISOString(),
        eventType: 'error',
        containerId: containerState?.id || 'unknown',
        containerName,
        userId: containerState?.userId || 'unknown',
        projectId: containerState?.projectId || 'unknown',
        error: errorMessage,
        details: 'Container destruction failed',
      });

      logger.error(`Failed to destroy container ${containerName}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Get container state
   */
  getContainerState(containerName: string): ContainerState | undefined {
    return this.containerStates.get(containerName);
  }

  /**
   * Get all containers for a user
   */
  getUserContainers(userId: string): ContainerState[] {
    return Array.from(this.containerStates.values()).filter(
      container => container.userId === userId
    );
  }

  /**
   * Get all containers for a project
   */
  getProjectContainers(userId: string, projectId: string): ContainerState[] {
    return Array.from(this.containerStates.values()).filter(
      container => container.userId === userId && container.projectId === projectId
    );
  }

  /**
   * Get container resource usage
   */
  async getContainerResourceUsage(containerName: string): Promise<ContainerState['resourceUsage']> {
    try {
      const containerState = this.containerStates.get(containerName);
      if (!containerState) {
        throw new Error(`Container state not found: ${containerName}`);
      }

      const stats = await dockerService.getContainerStats(containerState.id);
      
      const resourceUsage = {
        cpu: stats.cpu,
        memory: stats.memory,
        memoryLimit: stats.memoryLimit,
      };

      // Update container state
      containerState.resourceUsage = resourceUsage;
      this.containerStates.set(containerName, containerState);

      return resourceUsage;
    } catch (error) {
      logger.error(`Failed to get resource usage for container ${containerName}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Sync container states with Docker daemon
   */
  async syncContainerStates(): Promise<void> {
    try {
      const containers = await dockerService.listContainers(true);
      
      // Update states based on actual container status
      for (const container of containers) {
        const parsed = containerNamingService.parseContainerName(container.name);
        if (parsed) {
          const existingState = this.containerStates.get(container.name);
          
          if (existingState) {
            // Update existing state
            existingState.status = container.state === 'running' ? 'running' : 'stopped';
            this.containerStates.set(container.name, existingState);
          } else {
                         // Create new state for existing container
             const newState: ContainerState = {
               id: container.id,
               name: container.name,
               status: container.state === 'running' ? 'running' : 'stopped',
               createdAt: new Date((container.created as number) * 1000).toISOString(),
               userId: parsed.userId,
               projectId: parsed.projectId,
               image: container.image,
               ports: (container.ports as { [key: string]: string }) || {},
               volumes: container.mounts || [],
               environment: [],
             };
            this.containerStates.set(container.name, newState);
          }
        }
      }

      logger.info(`Synced ${containers.length} container states`);
    } catch (error) {
      logger.error('Failed to sync container states:', undefined, error as Error);
      throw error;
    }
  }

  /**
   * Get lifecycle events
   */
  getLifecycleEvents(options?: {
    userId?: string;
    projectId?: string;
    eventType?: LifecycleEvent['eventType'];
    limit?: number;
  }): LifecycleEvent[] {
    let events = [...this.lifecycleEvents];

    // Filter by user
    if (options?.userId) {
      events = events.filter(event => event.userId === options.userId);
    }

    // Filter by project
    if (options?.projectId) {
      events = events.filter(event => event.projectId === options.projectId);
    }

    // Filter by event type
    if (options?.eventType) {
      events = events.filter(event => event.eventType === options.eventType);
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
   * Log lifecycle event
   */
  private logLifecycleEvent(event: LifecycleEvent): void {
    this.lifecycleEvents.push(event);
    
    // Maintain event limit
    if (this.lifecycleEvents.length > this.maxEvents) {
      this.lifecycleEvents = this.lifecycleEvents.slice(-this.maxEvents);
    }
  }

  /**
   * Cleanup old events
   */
  cleanupOldEvents(maxAgeHours: number = 24): void {
    const cutoffTime = new Date(Date.now() - maxAgeHours * 60 * 60 * 1000);
    this.lifecycleEvents = this.lifecycleEvents.filter(
      event => new Date(event.timestamp) > cutoffTime
    );
  }

  /**
   * Get service statistics
   */
  getServiceStats(): {
    totalContainers: number;
    runningContainers: number;
    stoppedContainers: number;
    errorContainers: number;
    totalEvents: number;
  } {
    const containers = Array.from(this.containerStates.values());
    
    return {
      totalContainers: containers.length,
      runningContainers: containers.filter(c => c.status === 'running').length,
      stoppedContainers: containers.filter(c => c.status === 'stopped').length,
      errorContainers: containers.filter(c => c.status === 'error').length,
      totalEvents: this.lifecycleEvents.length,
    };
  }
}

// Export singleton instance
export const containerLifecycleService = new ContainerLifecycleService(); 