import Docker from 'dockerode';
import { logger } from '../utils/logger';

export interface ContainerConfig {
  name: string;
  image: string;
  command?: string[];
  env?: string[];
  volumes?: string[];
  ports?: { [key: string]: string };
  network?: string;
  workingDir?: string;
  user?: string;
  labels?: { [key: string]: string };
  restartPolicy?: string;
  cpuShares?: number;
  memory?: number;
  memorySwap?: number;
}

export interface ContainerInfo {
  id: string;
  name: string;
  image: string;
  status: string;
  state: string;
  created: string;
  ports: any[];
  labels: { [key: string]: string };
  networkSettings: any;
  mounts: any[];
  config: any;
  hostConfig: any;
}

export interface ContainerStats {
  cpu: number;
  memory: number;
  memoryLimit: number;
  networkRx: number;
  networkTx: number;
  blockRead: number;
  blockWrite: number;
  timestamp: string;
}

export class DockerService {
  private docker: Docker;
  private connectionPool: Map<string, Docker>;
  private maxConnections: number = 10;
  private healthCheckInterval: NodeJS.Timeout | null = null;

  constructor() {
    this.docker = new Docker();
    this.connectionPool = new Map();
    this.startHealthCheck();
  }

  /**
   * Initialize Docker connection and verify daemon is accessible
   */
  async initialize(): Promise<void> {
    try {
      await this.docker.ping();
      logger.info('Docker daemon connection established successfully');
      
      const info = await this.docker.info();
      logger.info(`Docker version: ${info.ServerVersion}, Containers: ${info.Containers}, Images: ${info.Images}`);
    } catch (error) {
      logger.error('Failed to connect to Docker daemon:', undefined, error as Error);
      throw new Error('Docker daemon is not accessible. Please ensure Docker is running.');
    }
  }

  /**
   * Get Docker daemon information
   */
  async getInfo(): Promise<any> {
    try {
      return await this.docker.info();
    } catch (error) {
      logger.error('Failed to get Docker info:', undefined, error as Error);
      throw error;
    }
  }

  /**
   * Create a new container
   */
  async createContainer(config: ContainerConfig): Promise<string> {
    try {
      logger.info(`Creating container: ${config.name}`);

      const container = await this.docker.createContainer({
        Image: config.image,
        name: config.name,
        Cmd: config.command,
        Env: config.env,
        WorkingDir: config.workingDir,
        User: config.user,
        Labels: config.labels as Record<string, string>,
        HostConfig: {
          Binds: config.volumes,
          PortBindings: this.formatPortBindings(config.ports),
          NetworkMode: config.network,
          RestartPolicy: config.restartPolicy ? { Name: config.restartPolicy } : undefined,
          CpuShares: config.cpuShares,
          Memory: config.memory,
          MemorySwap: config.memorySwap,
        },
      });

      logger.info(`Container created successfully: ${container.id}`);
      return container.id;
    } catch (error) {
      logger.error(`Failed to create container ${config.name}:`, error);
      throw error;
    }
  }

  /**
   * Start a container
   */
  async startContainer(containerId: string): Promise<void> {
    try {
      logger.info(`Starting container: ${containerId}`);
      const container = this.docker.getContainer(containerId);
      await container.start();
      logger.info(`Container started successfully: ${containerId}`);
    } catch (error) {
      logger.error(`Failed to start container ${containerId}:`, error);
      throw error;
    }
  }

  /**
   * Stop a container
   */
  async stopContainer(containerId: string, timeout: number = 10): Promise<void> {
    try {
      logger.info(`Stopping container: ${containerId}`);
      const container = this.docker.getContainer(containerId);
      await container.stop({ t: timeout });
      logger.info(`Container stopped successfully: ${containerId}`);
    } catch (error) {
      logger.error(`Failed to stop container ${containerId}:`, error);
      throw error;
    }
  }

  /**
   * Remove a container
   */
  async removeContainer(containerId: string, force: boolean = false): Promise<void> {
    try {
      logger.info(`Removing container: ${containerId}`);
      const container = this.docker.getContainer(containerId);
      await container.remove({ force });
      logger.info(`Container removed successfully: ${containerId}`);
    } catch (error) {
      logger.error(`Failed to remove container ${containerId}:`, error);
      throw error;
    }
  }

  /**
   * Get container information
   */
  async getContainerInfo(containerId: string): Promise<ContainerInfo> {
    try {
      const container = this.docker.getContainer(containerId);
      const info = await container.inspect();
      
      return {
        id: info.Id,
        name: info.Name.replace(/^\//, ''),
        image: info.Config.Image,
        status: info.State.Status,
        state: info.State.Running ? 'running' : 'stopped',
        created: info.Created,
        ports: info.NetworkSettings.Ports,
        labels: info.Config.Labels || {},
        networkSettings: info.NetworkSettings,
        mounts: info.Mounts,
        config: info.Config,
        hostConfig: info.HostConfig,
      };
    } catch (error) {
      logger.error(`Failed to get container info for ${containerId}:`, error);
      throw error;
    }
  }

  /**
   * Get container statistics
   */
  async getContainerStats(containerId: string): Promise<ContainerStats> {
    try {
      const container = this.docker.getContainer(containerId);
      const stats = await container.stats({ stream: false });
      
      const cpuDelta = stats.cpu_stats.cpu_usage.total_usage - stats.precpu_stats.cpu_usage.total_usage;
      const systemDelta = stats.cpu_stats.system_cpu_usage - stats.precpu_stats.system_cpu_usage;
      const cpuPercent = (cpuDelta / systemDelta) * stats.cpu_stats.online_cpus * 100;

      return {
        cpu: Math.round(cpuPercent * 100) / 100,
        memory: stats.memory_stats.usage || 0,
        memoryLimit: stats.memory_stats.limit || 0,
        networkRx: stats.networks?.eth0?.rx_bytes || 0,
        networkTx: stats.networks?.eth0?.tx_bytes || 0,
        blockRead: stats.blkio_stats.io_service_bytes_recursive?.find(s => s.op === 'Read')?.value || 0,
        blockWrite: stats.blkio_stats.io_service_bytes_recursive?.find(s => s.op === 'Write')?.value || 0,
        timestamp: new Date().toISOString(),
      };
    } catch (error) {
      logger.error(`Failed to get container stats for ${containerId}:`, error);
      throw error;
    }
  }

  /**
   * List all containers
   */
  async listContainers(all: boolean = false): Promise<ContainerInfo[]> {
    try {
      const containers = await this.docker.listContainers({ all });
      
      return containers.map(container => ({
        id: container.Id,
        name: container.Names[0].replace(/^\//, ''),
        image: container.Image,
        status: container.Status,
        state: container.State,
        created: new Date(container.Created * 1000).toISOString(),
        ports: container.Ports,
        labels: container.Labels || {},
        networkSettings: container.NetworkSettings,
        mounts: container.Mounts,
        config: container.Command,
        hostConfig: container.SizeRw,
      }));
    } catch (error) {
      logger.error('Failed to list containers:', error);
      throw error;
    }
  }

  /**
   * Get container logs
   */
  async getContainerLogs(containerId: string, options: {
    tail?: number;
    since?: string;
    until?: string;
    follow?: boolean;
    timestamps?: boolean;
  } = {}): Promise<string> {
    try {
      const container = this.docker.getContainer(containerId);
      const logs = await container.logs({
        stdout: true,
        stderr: true,
        ...options,
      });
      
      return logs.toString('utf8');
    } catch (error) {
      logger.error(`Failed to get container logs for ${containerId}:`, error);
      throw error;
    }
  }

  /**
   * Execute command in running container
   */
  async execCommand(containerId: string, command: string[]): Promise<{
    exitCode: number;
    output: string;
    error: string;
  }> {
    try {
      const container = this.docker.getContainer(containerId);
      const exec = await container.exec({
        AttachStdout: true,
        AttachStderr: true,
        Cmd: command,
      });

      const stream = await exec.start({});
      
      return new Promise((resolve, reject) => {
        let output = '';
        let error = '';
        
        stream.on('data', (chunk: Buffer) => {
          const data = chunk.toString('utf8');
          if (data.startsWith('\x01')) {
            output += data.slice(1);
          } else if (data.startsWith('\x02')) {
            error += data.slice(1);
          }
        });

        stream.on('end', async () => {
          try {
            const inspect = await exec.inspect();
            resolve({
              exitCode: inspect.ExitCode || 0,
              output,
              error,
            });
          } catch (err) {
            reject(err);
          }
        });

        stream.on('error', reject);
      });
    } catch (error) {
      logger.error(`Failed to execute command in container ${containerId}:`, error);
      throw error;
    }
  }

  /**
   * Check if container exists
   */
  async containerExists(containerId: string): Promise<boolean> {
    try {
      const container = this.docker.getContainer(containerId);
      await container.inspect();
      return true;
    } catch (error) {
      return false;
    }
  }

  /**
   * Get connection from pool or create new one
   */
  private getConnection(key: string): Docker {
    if (this.connectionPool.has(key)) {
      return this.connectionPool.get(key)!;
    }

    if (this.connectionPool.size >= this.maxConnections) {
      // Remove oldest connection
      const oldestKey = this.connectionPool.keys().next().value;
      this.connectionPool.delete(oldestKey);
    }

    const connection = new Docker();
    this.connectionPool.set(key, connection);
    return connection;
  }

  /**
   * Format port bindings for Docker API
   */
  private formatPortBindings(ports?: { [key: string]: string }): any {
    if (!ports) return undefined;

    const bindings: any = {};
    for (const [containerPort, hostPort] of Object.entries(ports)) {
      bindings[containerPort] = [{ HostPort: hostPort }];
    }
    return bindings;
  }

  /**
   * Start health check for Docker daemon
   */
  private startHealthCheck(): void {
    this.healthCheckInterval = setInterval(async () => {
      try {
        await this.docker.ping();
      } catch (error) {
        logger.warn('Docker daemon health check failed:', error);
      }
    }, 30000); // Check every 30 seconds
  }

  /**
   * Stop health check
   */
  stopHealthCheck(): void {
    if (this.healthCheckInterval) {
      clearInterval(this.healthCheckInterval);
      this.healthCheckInterval = null;
    }
  }

  /**
   * Cleanup resources
   */
  async cleanup(): Promise<void> {
    this.stopHealthCheck();
    this.connectionPool.clear();
  }
}

// Export singleton instance
export const dockerService = new DockerService(); 