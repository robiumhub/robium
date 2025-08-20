import { templateEngine } from './TemplateEngine';
import { workspaceMountingService, WorkspaceConfig, MountPoint } from './WorkspaceMountingService';
import { logger } from '../utils/logger';
import fs from 'fs';
import path from 'path';
import { promises as fsPromises } from 'fs';

export interface ComposeService {
  name: string;
  image?: string;
  build?: {
    context: string;
    dockerfile?: string;
    args?: Record<string, string>;
  };
  ports?: string[];
  volumes?: string[];
  environment?: Record<string, string> | string[];
  env_file?: string[];
  depends_on?: string[];
  networks?: string[];
  command?: string | string[];
  entrypoint?: string | string[];
  user?: string;
  working_dir?: string;
  restart?: 'no' | 'always' | 'on-failure' | 'unless-stopped';
  healthcheck?: {
    test: string | string[];
    interval?: string;
    timeout?: string;
    retries?: number;
    start_period?: string;
  };
  labels?: Record<string, string>;
  deploy?: {
    replicas?: number;
    resources?: {
      limits?: {
        cpus?: string;
        memory?: string;
      };
      reservations?: {
        cpus?: string;
        memory?: string;
      };
    };
  };
}

export interface ComposeNetwork {
  name: string;
  driver?: string;
  driver_opts?: Record<string, string>;
  external?: boolean;
  labels?: Record<string, string>;
}

export interface ComposeVolume {
  name: string;
  driver?: string;
  driver_opts?: Record<string, string>;
  external?: boolean;
  labels?: Record<string, string>;
}

export interface ComposeConfiguration {
  id: string;
  name: string;
  version: string;
  description?: string;
  services: ComposeService[];
  networks?: ComposeNetwork[];
  volumes?: ComposeVolume[];
  configs?: Record<string, any>;
  secrets?: Record<string, any>;
  environment?: 'development' | 'production' | 'staging';
  variables?: Record<string, string>;
}

export interface ComposeOptions {
  template?: string;
  outputPath?: string;
  validateOnly?: boolean;
  includeComments?: boolean;
  format?: 'yaml' | 'json';
  environment?: 'development' | 'production' | 'staging';
  workspaceConfig?: WorkspaceConfig;
  enableWorkspaceMounting?: boolean;
}

export interface ComposeResult {
  content: string;
  path: string;
  errors: string[];
  warnings: string[];
  validationIssues: string[];
  services: string[];
  networks: string[];
  volumes: string[];
  generationTime: number;
  size: number;
}

export interface ComposeValidation {
  isValid: boolean;
  errors: string[];
  warnings: string[];
  suggestions: string[];
}

export class DockerComposeGenerationService {
  private composeConfigs: Map<string, ComposeConfiguration> = new Map();
  private generatedFiles: Map<string, ComposeResult> = new Map();
  private defaultOutputDir: string;

  constructor() {
    this.defaultOutputDir = process.env.COMPOSE_OUTPUT_DIR || './generated';
    this.initializeDefaultTemplates();
  }

  /**
   * Initialize default docker-compose templates
   */
  private initializeDefaultTemplates(): void {
    // Default docker-compose template
    const defaultComposeTemplate = `# Generated docker-compose.yml for {{project.name}}
version: '{{composeVersion}}'

services:
{{#each services}}
  {{name}}:
    {{#if image}}
    image: {{image}}
    {{/if}}
    {{#if build}}
    build:
      context: {{build.context}}
      {{#if build.dockerfile}}
      dockerfile: {{build.dockerfile}}
      {{/if}}
      {{#if build.args}}
      args:
        {{#each build.args}}
        {{@key}}: {{this}}
        {{/each}}
      {{/if}}
    {{/if}}
    {{#if ports}}
    ports:
      {{#each ports}}
      - "{{this}}"
      {{/each}}
    {{/if}}
    {{#if volumes}}
    volumes:
      {{#each volumes}}
      - {{this}}
      {{/each}}
    {{/if}}
    {{#if environment}}
    environment:
      {{#each environment}}
      {{@key}}: {{this}}
      {{/each}}
    {{/if}}
    {{#if env_file}}
    env_file:
      {{#each env_file}}
      - {{this}}
      {{/each}}
    {{/if}}
    {{#if depends_on}}
    depends_on:
      {{#each depends_on}}
      - {{this}}
      {{/each}}
    {{/if}}
    {{#if networks}}
    networks:
      {{#each networks}}
      - {{this}}
      {{/each}}
    {{/if}}
    {{#if command}}
    command: {{command}}
    {{/if}}
    {{#if entrypoint}}
    entrypoint: {{entrypoint}}
    {{/if}}
    {{#if user}}
    user: {{user}}
    {{/if}}
    {{#if working_dir}}
    working_dir: {{working_dir}}
    {{/if}}
    {{#if restart}}
    restart: {{restart}}
    {{/if}}
    {{#if healthcheck}}
    healthcheck:
      test: {{healthcheck.test}}
      {{#if healthcheck.interval}}
      interval: {{healthcheck.interval}}
      {{/if}}
      {{#if healthcheck.timeout}}
      timeout: {{healthcheck.timeout}}
      {{/if}}
      {{#if healthcheck.retries}}
      retries: {{healthcheck.retries}}
      {{/if}}
      {{#if healthcheck.start_period}}
      start_period: {{healthcheck.start_period}}
      {{/if}}
    {{/if}}
    {{#if labels}}
    labels:
      {{#each labels}}
      {{@key}}: {{this}}
      {{/each}}
    {{/if}}
    {{#if deploy}}
    deploy:
      {{#if deploy.replicas}}
      replicas: {{deploy.replicas}}
      {{/if}}
      {{#if deploy.resources}}
      resources:
        {{#if deploy.resources.limits}}
        limits:
          {{#if deploy.resources.limits.cpus}}
          cpus: {{deploy.resources.limits.cpus}}
          {{/if}}
          {{#if deploy.resources.limits.memory}}
          memory: {{deploy.resources.limits.memory}}
          {{/if}}
        {{/if}}
        {{#if deploy.resources.reservations}}
        reservations:
          {{#if deploy.resources.reservations.cpus}}
          cpus: {{deploy.resources.reservations.cpus}}
          {{/if}}
          {{#if deploy.resources.reservations.memory}}
          memory: {{deploy.resources.reservations.memory}}
          {{/if}}
        {{/if}}
      {{/if}}
    {{/if}}
{{/each}}

{{#if networks}}
networks:
  {{#each networks}}
  {{name}}:
    {{#if driver}}
    driver: {{driver}}
    {{/if}}
    {{#if driver_opts}}
    driver_opts:
      {{#each driver_opts}}
      {{@key}}: {{this}}
      {{/each}}
    {{/if}}
    {{#if external}}
    external: {{external}}
    {{/if}}
    {{#if labels}}
    labels:
      {{#each labels}}
      {{@key}}: {{this}}
      {{/each}}
    {{/if}}
  {{/each}}
{{/if}}

{{#if volumes}}
volumes:
  {{#each volumes}}
  {{name}}:
    {{#if driver}}
    driver: {{driver}}
    {{/if}}
    {{#if driver_opts}}
    driver_opts:
      {{#each driver_opts}}
      {{@key}}: {{this}}
      {{/each}}
    {{/if}}
    {{#if external}}
    external: {{external}}
    {{/if}}
    {{#if labels}}
    labels:
      {{#each labels}}
      {{@key}}: {{this}}
      {{/each}}
    {{/if}}
  {{/each}}
{{/if}}
`;

    // Development environment template
    const developmentComposeTemplate = `# Generated docker-compose.yml for {{project.name}} (Development)
version: '{{composeVersion}}'

services:
{{#each services}}
  {{name}}:
    {{#if build}}
    build:
      context: {{build.context}}
    {{/if}}
    {{#if ports}}
    ports:
      {{#each ports}}
      - "{{this}}"
      {{/each}}
    {{/if}}
    {{#if volumes}}
    volumes:
{{volumesString}}
    {{/if}}
    {{#if environment}}
    environment:
      {{#each environment}}
      {{@key}}: {{this}}
      {{/each}}
    {{/if}}
    restart: unless-stopped
{{/each}}

{{#if networks}}
networks:
  {{#each networks}}
  {{name}}:
    driver: {{driver}}
  {{/each}}
{{/if}}

{{#if volumes}}
volumes:
  {{#each volumes}}
  {{name}}:
    driver: {{driver}}
  {{/each}}
{{/if}}
`;

    // Production environment template
    const productionComposeTemplate = `# Generated docker-compose.yml for {{project.name}} (Production)
version: '{{composeVersion}}'

services:
{{#each services}}
  {{name}}:
    {{#if image}}image: {{image}}{{/if}}
    {{#if build}}build:
      context: {{build.context}}
      {{#if build.dockerfile}}dockerfile: {{build.dockerfile}}{{/if}}
      {{#if build.args}}args:
        {{#each build.args}}
        {{@key}}: {{this}}
        {{/each}}{{/if}}
    {{/if}}
    {{#if ports}}ports:
      {{#each ports}}
      - "{{this}}"
      {{/each}}{{/if}}
    {{#if volumes}}volumes:
      {{#each volumes}}
      - {{this}}
      {{/each}}{{/if}}
    {{#if environment}}environment:
      {{#each environment}}
      {{@key}}: {{this}}
      {{/each}}{{/if}}
    {{#if env_file}}env_file:
      {{#each env_file}}
      - {{this}}
      {{/each}}{{/if}}
    {{#if depends_on}}depends_on:
      {{#each depends_on}}
      - {{this}}
      {{/each}}{{/if}}
    {{#if networks}}networks:
      {{#each networks}}
      - {{this}}
      {{/each}}{{/if}}
    {{#if command}}command: {{command}}{{/if}}
    {{#if entrypoint}}entrypoint: {{entrypoint}}{{/if}}
    {{#if user}}user: {{user}}{{/if}}
    {{#if working_dir}}working_dir: {{working_dir}}{{/if}}
    restart: always
    {{#if healthcheck}}healthcheck:
      test: {{healthcheck.test}}
      {{#if healthcheck.interval}}interval: {{healthcheck.interval}}{{/if}}
      {{#if healthcheck.timeout}}timeout: {{healthcheck.timeout}}{{/if}}
      {{#if healthcheck.retries}}retries: {{healthcheck.retries}}{{/if}}
      {{#if healthcheck.start_period}}start_period: {{healthcheck.start_period}}{{/if}}
    {{/if}}
    {{#if labels}}labels:
      {{#each labels}}
      {{@key}}: {{this}}
      {{/each}}{{/if}}
    {{#if deploy}}deploy:
      {{#if deploy.replicas}}replicas: {{deploy.replicas}}{{/if}}
      {{#if deploy.resources}}resources:
        {{#if deploy.resources.limits}}limits:
          {{#if deploy.resources.limits.cpus}}cpus: {{deploy.resources.limits.cpus}}{{/if}}
          {{#if deploy.resources.limits.memory}}memory: {{deploy.resources.limits.memory}}{{/if}}
        {{/if}}
        {{#if deploy.resources.reservations}}reservations:
          {{#if deploy.resources.reservations.cpus}}cpus: {{deploy.resources.reservations.cpus}}{{/if}}
          {{#if deploy.resources.reservations.memory}}memory: {{deploy.resources.reservations.memory}}{{/if}}
        {{/if}}
      {{/if}}
    {{/if}}
{{/each}}

{{#if networks}}networks:
  {{#each networks}}
  {{name}}:
    {{#if driver}}driver: {{driver}}{{/if}}
    {{#if driver_opts}}driver_opts:
      {{#each driver_opts}}
      {{@key}}: {{this}}
      {{/each}}{{/if}}
    {{#if external}}external: {{external}}{{/if}}
    {{#if labels}}labels:
      {{#each labels}}
      {{@key}}: {{this}}
      {{/each}}{{/if}}
  {{/each}}{{/if}}

{{#if volumes}}volumes:
  {{#each volumes}}
  {{name}}:
    {{#if driver}}driver: {{driver}}{{/if}}
    {{#if driver_opts}}driver_opts:
      {{#each driver_opts}}
      {{@key}}: {{this}}
      {{/each}}{{/if}}
    {{#if external}}external: {{external}}{{/if}}
    {{#if labels}}labels:
      {{#each labels}}
      {{@key}}: {{this}}
      {{/each}}{{/if}}
  {{/each}}{{/if}}
`;

    templateEngine.registerTemplate('docker-compose-default', defaultComposeTemplate);
    templateEngine.registerTemplate('docker-compose-development', developmentComposeTemplate);
    templateEngine.registerTemplate('docker-compose-production', productionComposeTemplate);

    logger.info('Initialized default docker-compose templates');
  }

  /**
   * Register a compose configuration
   */
  registerComposeConfig(config: ComposeConfiguration): void {
    this.composeConfigs.set(config.id, config);
    logger.info(`Registered compose configuration: ${config.name}`);
  }

  /**
   * Get compose configuration
   */
  getComposeConfig(configId: string): ComposeConfiguration | undefined {
    return this.composeConfigs.get(configId);
  }

  /**
   * Generate docker-compose.yml for a configuration
   */
  async generateCompose(
    configId: string,
    options: ComposeOptions = {}
  ): Promise<ComposeResult> {
    const startTime = Date.now();
    const errors: string[] = [];
    const warnings: string[] = [];
    const validationIssues: string[] = [];

    try {
      // Get compose configuration
      const config = this.composeConfigs.get(configId);
      if (!config) {
        throw new Error(`Compose configuration not found: ${configId}`);
      }

      // Configure workspace mounting if enabled
      let finalConfig = config;
      if (options.enableWorkspaceMounting && options.workspaceConfig) {
        finalConfig = await this.configureWorkspaceMounting(config, options.workspaceConfig);
        logger.info(`Workspace mounting enabled for ${config.name}`);
      }

      // Determine template to use
      const templateName = options.template || this.getDefaultTemplate(finalConfig.environment || options.environment);
      logger.info(`Using template: ${templateName}`);

      // Prepare context for template processing
      const context = this.prepareTemplateContext(finalConfig, options);
      logger.info(`Template context:`, context);

      // Process template
      const templateResult = templateEngine.processTemplate(templateName, context, {
        validateOnly: options.validateOnly
      });
      logger.info(`Template result:`, {
        contentLength: templateResult.content.length,
        errors: templateResult.errors.length,
        warnings: templateResult.warnings.length
      });

      if (templateResult.errors.length > 0) {
        logger.error(`Template processing errors:`, { errors: templateResult.errors });
        errors.push(...templateResult.errors.map(e => e.message));
      }

      if (templateResult.warnings.length > 0) {
        warnings.push(...templateResult.warnings);
      }

      // Validate generated compose file
      const validation = this.validateCompose(templateResult.content, finalConfig);
      errors.push(...validation.errors);
      warnings.push(...validation.warnings);
      validationIssues.push(...validation.suggestions);

      // Add comments if requested
      let finalContent = templateResult.content;
      if (options.includeComments) {
        finalContent = this.addComments(finalContent, finalConfig);
      }

      // Determine output path
      const outputPath = options.outputPath || this.generateOutputPath(configId, finalConfig, options);

      // Write file if not validate-only
      if (!options.validateOnly) {
        await this.writeComposeFile(outputPath, finalContent);
      }

      const result: ComposeResult = {
        content: finalContent,
        path: outputPath,
        errors,
        warnings,
        validationIssues,
        services: finalConfig.services.map(s => s.name),
        networks: finalConfig.networks?.map(n => n.name) || [],
        volumes: finalConfig.volumes?.map(v => v.name) || [],
        generationTime: Date.now() - startTime,
        size: finalContent.length,
      };

      // Store result
      this.generatedFiles.set(configId, result);

      logger.info(`Docker-compose generated for ${config.name} in ${result.generationTime}ms`);
      return result;

    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
      errors.push(errorMessage);

      return {
        content: '',
        path: '',
        errors,
        warnings,
        validationIssues,
        services: [],
        networks: [],
        volumes: [],
        generationTime: Date.now() - startTime,
        size: 0,
      };
    }
  }

  /**
   * Get default template for environment
   */
  private getDefaultTemplate(environment?: string): string {
    switch (environment) {
      case 'development':
        return 'docker-compose-development';
      case 'production':
        return 'docker-compose-production';
      case 'staging':
        return 'docker-compose-production'; // Use production template for staging
      default:
        return 'docker-compose-default';
    }
  }

  /**
   * Prepare context for template processing
   */
  private prepareTemplateContext(config: ComposeConfiguration, options: ComposeOptions): any {
    return {
      project: {
        name: config.name,
        version: config.version,
        description: config.description,
      },
      composeVersion: '3.8',
      services: config.services.map(service => ({
        ...service,
        isArray: Array.isArray(service.environment),
        // Ensure volumes are properly formatted as strings for template processing
        volumes: service.volumes || [],
        // Pre-format volumes as a string for the template
        volumesString: (service.volumes || []).map(vol => `      - ${vol}`).join('\n'),
      })),
      networks: config.networks || [],
      volumes: config.volumes || [],
      configs: config.configs || {},
      secrets: config.secrets || {},
      environment: config.environment || options.environment || 'development',
      variables: config.variables || {},
    };
  }

  /**
   * Validate generated docker-compose file
   */
  private validateCompose(content: string, config: ComposeConfiguration): ComposeValidation {
    const errors: string[] = [];
    const warnings: string[] = [];
    const suggestions: string[] = [];

    // Check for required sections
    if (!content.includes('version:')) {
      errors.push('Missing version specification');
    }

    if (!content.includes('services:')) {
      errors.push('Missing services section');
    }

    // Check for service definitions
    for (const service of config.services) {
      if (!content.includes(`  ${service.name}:`)) {
        errors.push(`Service ${service.name} not found in generated content`);
      }

      // Check for required service properties
      if (!service.image && !service.build) {
        warnings.push(`Service ${service.name} has neither image nor build configuration`);
      }

      // Check for port conflicts
      if (service.ports) {
        for (const port of service.ports) {
          if (port.includes(':')) {
            const hostPort = port.split(':')[0];
            // Check for port conflicts with other services
            for (const otherService of config.services) {
              if (otherService.name !== service.name && otherService.ports) {
                for (const otherPort of otherService.ports) {
                  if (otherPort.startsWith(hostPort + ':')) {
                    warnings.push(`Port conflict detected: ${hostPort} used by services ${service.name} and ${otherService.name}`);
                  }
                }
              }
            }
          }
        }
      }
    }

    // Check for network references
    for (const service of config.services) {
      if (service.networks) {
        for (const network of service.networks) {
          const networkExists = config.networks?.some(n => n.name === network);
          if (!networkExists) {
            warnings.push(`Service ${service.name} references undefined network: ${network}`);
          }
        }
      }
    }

    // Check for volume references
    for (const service of config.services) {
      if (service.volumes) {
        for (const volume of service.volumes) {
          if (volume.includes(':')) {
            const volumeName = volume.split(':')[0];
            const volumeExists = config.volumes?.some(v => v.name === volumeName);
            if (!volumeExists && !volumeName.startsWith('./') && !volumeName.startsWith('/')) {
              warnings.push(`Service ${service.name} references undefined volume: ${volumeName}`);
            }
          }
        }
      }
    }

    // Check for dependency cycles
    const dependencyGraph = this.buildDependencyGraph(config.services);
    const cycles = this.detectCycles(dependencyGraph);
    if (cycles.length > 0) {
      errors.push(`Circular dependencies detected: ${cycles.join(', ')}`);
    }

    // Optimization suggestions
    if (config.services.length > 1 && !config.networks?.length) {
      suggestions.push('Consider defining custom networks for better service isolation');
    }

    if (config.services.some(s => s.volumes?.some(v => v.includes('.:/app')))) {
      suggestions.push('Consider using named volumes for better performance in production');
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      suggestions,
    };
  }

  /**
   * Build dependency graph for services
   */
  private buildDependencyGraph(services: ComposeService[]): Map<string, string[]> {
    const graph = new Map<string, string[]>();

    for (const service of services) {
      graph.set(service.name, service.depends_on || []);
    }

    return graph;
  }

  /**
   * Detect cycles in dependency graph
   */
  private detectCycles(graph: Map<string, string[]>): string[] {
    const visited = new Set<string>();
    const recursionStack = new Set<string>();
    const cycles: string[] = [];

    const dfs = (node: string, path: string[]): void => {
      if (recursionStack.has(node)) {
        const cycleStart = path.indexOf(node);
        const cycle = path.slice(cycleStart).concat(node);
        cycles.push(cycle.join(' -> '));
        return;
      }

      if (visited.has(node)) {
        return;
      }

      visited.add(node);
      recursionStack.add(node);
      path.push(node);

      const dependencies = graph.get(node) || [];
      for (const dep of dependencies) {
        dfs(dep, [...path]);
      }

      recursionStack.delete(node);
    };

    for (const node of graph.keys()) {
      if (!visited.has(node)) {
        dfs(node, []);
      }
    }

    return cycles;
  }

  /**
   * Add comments to docker-compose file
   */
  private addComments(content: string, config: ComposeConfiguration): string {
    let commented = content;

    // Add header comment
    const header = `# Generated docker-compose.yml for ${config.name}
# Project: ${config.name} v${config.version}
# Environment: ${config.environment || 'development'}
# Generated: ${new Date().toISOString()}
#
`;

    commented = header + commented;

    return commented;
  }

  /**
   * Generate output path for docker-compose file
   */
  private generateOutputPath(configId: string, config: ComposeConfiguration, options: ComposeOptions): string {
    const environment = config.environment || options.environment || 'development';
    const filename = `docker-compose.${environment}.yml`;
    return path.join(this.defaultOutputDir, configId, filename);
  }

  /**
   * Write docker-compose file to disk
   */
  private async writeComposeFile(filePath: string, content: string): Promise<void> {
    try {
      // Ensure directory exists
      const dir = path.dirname(filePath);
      await fsPromises.mkdir(dir, { recursive: true });

      // Write file
      await fsPromises.writeFile(filePath, content, 'utf8');
      logger.info(`Docker-compose file written to: ${filePath}`);
    } catch (error) {
      logger.error(`Failed to write docker-compose file to ${filePath}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Configure workspace mounting for compose services
   */
  private async configureWorkspaceMounting(
    config: ComposeConfiguration,
    workspaceConfig: WorkspaceConfig
  ): Promise<ComposeConfiguration> {
    try {
      // Create workspace mount point
      const mountPoint = await workspaceMountingService.createWorkspace(workspaceConfig);
      
      // Update services with workspace volumes
      const updatedServices = config.services.map(service => ({
        ...service,
        volumes: [
          ...(service.volumes || []),
          workspaceMountingService.generateMountConfig(mountPoint)
        ]
      }));

      // Create updated configuration
      const updatedConfig: ComposeConfiguration = {
        ...config,
        services: updatedServices
      };

      logger.info(`Workspace mounting configured for ${config.name}: ${mountPoint.hostPath} -> ${mountPoint.containerPath}`);
      return updatedConfig;

    } catch (error) {
      logger.error(`Failed to configure workspace mounting for ${config.name}:`, undefined, error as Error);
      throw error;
    }
  }

  /**
   * Get workspace mount points for a configuration
   */
  getWorkspaceMountPoints(configId: string): MountPoint[] {
    const config = this.composeConfigs.get(configId);
    if (!config) {
      return [];
    }

    // Use the new method to get mount points by project ID
    return workspaceMountingService.getMountPointsByProjectId(configId);
  }

  /**
   * Create workspace backup for a configuration
   */
  async createWorkspaceBackup(configId: string): Promise<any> {
    const mountPoints = this.getWorkspaceMountPoints(configId);
    if (mountPoints.length === 0) {
      throw new Error(`No workspace mount points found for configuration: ${configId}`);
    }

    const backups = [];
    for (const mountPoint of mountPoints) {
      const backup = await workspaceMountingService.createBackup(mountPoint.id);
      backups.push(backup);
    }

    return backups;
  }

  /**
   * Get workspace statistics for a configuration
   */
  async getWorkspaceStats(configId: string): Promise<any[]> {
    const mountPoints = this.getWorkspaceMountPoints(configId);
    const stats = [];

    for (const mountPoint of mountPoints) {
      const stat = await workspaceMountingService.getWorkspaceStats(mountPoint.id);
      stats.push({
        mountPointId: mountPoint.id,
        ...stat
      });
    }

    return stats;
  }

  /**
   * Get generated docker-compose result
   */
  getGeneratedCompose(configId: string): ComposeResult | undefined {
    return this.generatedFiles.get(configId);
  }

  /**
   * List all generated docker-compose files
   */
  listGeneratedComposeFiles(): string[] {
    return Array.from(this.generatedFiles.keys());
  }

  /**
   * Remove generated docker-compose file
   */
  removeGeneratedCompose(configId: string): boolean {
    const result = this.generatedFiles.get(configId);
    if (result && result.path) {
      try {
        fs.unlinkSync(result.path);
        logger.info(`Removed docker-compose file: ${result.path}`);
      } catch (error) {
        logger.warn(`Failed to remove docker-compose file ${result.path}: ${error}`);
      }
    }

    const removed = this.generatedFiles.delete(configId);
    if (removed) {
      logger.info(`Removed generated docker-compose file for config: ${configId}`);
    }

    return removed;
  }

  /**
   * Get service statistics
   */
  getServiceStats(): {
    totalConfigs: number;
    totalGeneratedFiles: number;
    totalErrors: number;
    totalWarnings: number;
  } {
    let totalErrors = 0;
    let totalWarnings = 0;

    for (const result of this.generatedFiles.values()) {
      totalErrors += result.errors.length;
      totalWarnings += result.warnings.length;
    }

    return {
      totalConfigs: this.composeConfigs.size,
      totalGeneratedFiles: this.generatedFiles.size,
      totalErrors,
      totalWarnings,
    };
  }

  /**
   * Clean up old generated files
   */
  cleanupOldFiles(maxAgeDays: number = 7): void {
    const cutoffTime = Date.now() - maxAgeDays * 24 * 60 * 60 * 1000;
    let cleanedCount = 0;

    for (const [configId, result] of this.generatedFiles.entries()) {
      try {
        const stats = fs.statSync(result.path);
        if (stats.mtime.getTime() < cutoffTime) {
          this.removeGeneratedCompose(configId);
          cleanedCount++;
        }
      } catch (error) {
        // File might not exist, remove from tracking
        this.generatedFiles.delete(configId);
      }
    }

    logger.info(`Cleaned up ${cleanedCount} old generated docker-compose files`);
  }
}

// Export singleton instance
export const dockerComposeGenerationService = new DockerComposeGenerationService(); 