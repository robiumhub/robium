import { templateEngine } from './TemplateEngine';
import { logger } from '../utils/logger';
import fs from 'fs';
import path from 'path';
import { promises as fsPromises } from 'fs';
import {
  ModuleLoader,
  ROSProjectConfig,
  ModuleSpec,
  RobotSpec,
} from '@robium/shared';
import Handlebars from 'handlebars';

export interface ProjectConfiguration {
  id: string;
  name: string;
  version: string;
  description?: string;
  type: 'python' | 'nodejs' | 'java' | 'golang' | 'rust' | 'cpp' | 'custom';
  baseImage: string;
  workdir: string;
  systemDependencies?: string[];
  pythonDependencies?: string[];
  nodeDependencies?: string[];
  javaDependencies?: string[];
  golangDependencies?: string[];
  rustDependencies?: string[];
  cppDependencies?: string[];
  customDependencies?: string[];
  environmentVariables?: Record<string, string>;
  ports?: string[];
  volumes?: string[];
  command?: string;
  entrypoint?: string;
  user?: string;
  healthCheck?: {
    command: string;
    interval?: string;
    timeout?: string;
    retries?: number;
    startPeriod?: string;
  };
  buildArgs?: Record<string, string>;
  labels?: Record<string, string>;
  multiStage?: boolean;
  stages?: {
    name: string;
    baseImage: string;
    dependencies?: string[];
    buildSteps?: string[];
    copyFrom?: string;
  }[];
}

export interface DockerfileOptions {
  template?: string;
  outputPath?: string;
  validateOnly?: boolean;
  optimize?: boolean;
  includeComments?: boolean;
  securityScan?: boolean;
  context?: any;
}

export interface DockerfileResult {
  content: string;
  path: string;
  errors: string[];
  warnings: string[];
  optimizationSuggestions: string[];
  securityIssues: string[];
  buildTime: number;
  size: number;
}

export interface DockerfileValidation {
  isValid: boolean;
  errors: string[];
  warnings: string[];
  suggestions: string[];
}

// ROS-specific interfaces
export interface ROSDockerfileOptions extends DockerfileOptions {
  buildProfile?: 'development' | 'production';
  includeCompose?: boolean;
  includeBake?: boolean;
  outputDir?: string;
}

export interface ROSDockerfileResult extends DockerfileResult {
  composeContent?: string;
  bakeContent?: string;
  composePath?: string;
  bakePath?: string;
}

export interface ROSGenerationContext {
  distro: string;
  robot: RobotSpec;
  modules: ModuleSpec[];
  customizations: {
    udevRules: string[];
    env: Record<string, string>;
    expose: number[];
    files: Array<{
      path: string;
      contents: string;
    }>;
    bashrcAliases: string[];
  };
  buildProfile: 'development' | 'production';
  workdir: string;
  user: string;
}

export class DockerfileGenerationService {
  private projectConfigs: Map<string, ProjectConfiguration> = new Map();
  private generatedFiles: Map<string, DockerfileResult> = new Map();
  private defaultOutputDir: string;

  constructor() {
    this.defaultOutputDir = process.env.DOCKERFILE_OUTPUT_DIR || './generated';
    this.initializeDefaultTemplates();
  }

  /**
   * Initialize default Dockerfile templates
   */
  private initializeDefaultTemplates(): void {
    // Python-specific template
    const pythonTemplate = `# Generated Dockerfile for {{project.name}} (Python)
FROM {{baseImage}}

# Set working directory
WORKDIR {{workdir}}

# Install system dependencies
{{#if systemDependencies}}
RUN apt-get update && apt-get install -y \\
{{#each systemDependencies}}
    {{this}} \\
{{/each}}
    && rm -rf /var/lib/apt/lists/*
{{/if}}

# Set Python environment
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1

# Install Python dependencies
{{#if pythonDependencies}}
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
{{/if}}

# Copy application code
COPY . .

# Set environment variables
{{#each environmentVariables}}
ENV {{@key}}={{this}}
{{/each}}

# Expose ports
{{#each ports}}
EXPOSE {{this}}
{{/each}}

# Health check
{{#if healthCheck}}
HEALTHCHECK --interval={{healthCheck.interval}} --timeout={{healthCheck.timeout}} --start-period={{healthCheck.startPeriod}} --retries={{healthCheck.retries}} \\
    CMD {{healthCheck.command}}
{{/if}}

# Set user
{{#if user}}
USER {{user}}
{{/if}}

# Set entrypoint
{{#if entrypoint}}
ENTRYPOINT ["{{entrypoint}}"]
{{/if}}

# Set default command
{{#if command}}
CMD {{command}}
{{else}}
CMD ["python", "app.py"]
{{/if}}`;

    // Node.js-specific template
    const nodejsTemplate = `# Generated Dockerfile for {{project.name}} (Node.js)
FROM {{baseImage}}

# Set working directory
WORKDIR {{workdir}}

# Install system dependencies
{{#if systemDependencies}}
RUN apt-get update && apt-get install -y \\
{{#each systemDependencies}}
    {{this}} \\
{{/each}}
    && rm -rf /var/lib/apt/lists/*
{{/if}}

# Copy package files
COPY package*.json ./

# Install Node.js dependencies
{{#if nodeDependencies}}
RUN npm ci --only=production
{{else}}
RUN npm install
{{/if}}

# Copy application code
COPY . .

# Set environment variables
{{#each environmentVariables}}
ENV {{@key}}={{this}}
{{/each}}

# Expose ports
{{#each ports}}
EXPOSE {{this}}
{{/each}}

# Health check
{{#if healthCheck}}
HEALTHCHECK --interval={{healthCheck.interval}} --timeout={{healthCheck.timeout}} --start-period={{healthCheck.startPeriod}} --retries={{healthCheck.retries}} \\
    CMD {{healthCheck.command}}
{{/if}}

# Set user
{{#if user}}
USER {{user}}
{{/if}}

# Set entrypoint
{{#if entrypoint}}
ENTRYPOINT ["{{entrypoint}}"]
{{/if}}

# Set default command
{{#if command}}
CMD {{command}}
{{else}}
CMD ["npm", "start"]
{{/if}}`;

    // Multi-stage template
    const multiStageTemplate = `# Generated multi-stage Dockerfile for {{project.name}}
{{#each stages}}
# Stage: {{name}}
FROM {{baseImage}} as {{name}}

WORKDIR {{workdir}}

{{#if dependencies}}
# Install dependencies
{{#each dependencies}}
RUN {{this}}
{{/each}}
{{/if}}

{{#if buildSteps}}
# Build steps
{{#each buildSteps}}
RUN {{this}}
{{/each}}
{{/if}}

{{#if copyFrom}}
# Copy from previous stage
COPY --from={{copyFrom}} {{workdir}}/{{copyFrom}} {{workdir}}/
{{/if}}
e
{{/each}}

# Final stage
FROM {{baseImage}}

WORKDIR {{workdir}}

# Copy built artifacts
{{#each stages}}
{{#unless @first}}
COPY --from={{name}} {{workdir}}/{{name}} {{workdir}}/
{{/unless}}
{{/each}}

# Set environment variables
{{#each environmentVariables}}
ENV {{@key}}={{this}}
{{/each}}

# Expose ports
{{#each ports}}
EXPOSE {{this}}
{{/each}}

# Set default command
{{#if command}}
CMD {{command}}
{{else}}
CMD ["bash"]
{{/if}}`;

    templateEngine.registerTemplate('python-dockerfile', pythonTemplate);
    templateEngine.registerTemplate('nodejs-dockerfile', nodejsTemplate);
    templateEngine.registerTemplate(
      'multistage-dockerfile',
      multiStageTemplate
    );

    logger.info('Initialized default Dockerfile templates');
  }

  /**
   * Register a project configuration
   */
  registerProjectConfig(config: ProjectConfiguration): void {
    this.projectConfigs.set(config.id, config);
    logger.info(`Registered project configuration: ${config.name}`);
  }

  /**
   * Get project configuration
   */
  getProjectConfig(projectId: string): ProjectConfiguration | undefined {
    return this.projectConfigs.get(projectId);
  }

  /**
   * Generate Dockerfile for a project
   */
  async generateDockerfile(
    projectId: string,
    options: DockerfileOptions = {}
  ): Promise<DockerfileResult> {
    const startTime = Date.now();
    const errors: string[] = [];
    const warnings: string[] = [];
    const optimizationSuggestions: string[] = [];
    const securityIssues: string[] = [];

    try {
      // Get project configuration
      const config = this.projectConfigs.get(projectId);
      if (!config) {
        throw new Error(`Project configuration not found: ${projectId}`);
      }

      // Determine template to use
      const templateName =
        options.template || this.getDefaultTemplate(config.type);

      // Prepare context for template processing
      const context = this.prepareTemplateContext(config);

      // Process template
      const templateResult = templateEngine.processTemplate(
        templateName,
        context,
        {
          validateOnly: options.validateOnly,
        }
      );

      if (templateResult.errors.length > 0) {
        errors.push(...templateResult.errors.map((e) => e.message));
      }

      if (templateResult.warnings.length > 0) {
        warnings.push(...templateResult.warnings);
      }

      // Validate generated Dockerfile
      const validation = this.validateDockerfile(
        templateResult.content,
        config
      );
      errors.push(...validation.errors);
      warnings.push(...validation.warnings);
      optimizationSuggestions.push(...validation.suggestions);

      // Security scan if requested
      if (options.securityScan) {
        const securityScan = this.securityScanDockerfile(
          templateResult.content
        );
        securityIssues.push(...securityScan);
      }

      // Optimize if requested
      let finalContent = templateResult.content;
      if (options.optimize) {
        finalContent = this.optimizeDockerfile(finalContent);
      }

      // Add comments if requested
      if (options.includeComments) {
        finalContent = this.addComments(finalContent, config);
      }

      // Determine output path
      const outputPath =
        options.outputPath || this.generateOutputPath(projectId, config);

      // Write file if not validate-only
      if (!options.validateOnly) {
        await this.writeDockerfile(outputPath, finalContent);
      }

      const result: DockerfileResult = {
        content: finalContent,
        path: outputPath,
        errors,
        warnings,
        optimizationSuggestions,
        securityIssues,
        buildTime: Date.now() - startTime,
        size: finalContent.length,
      };

      // Store result
      this.generatedFiles.set(projectId, result);

      logger.info(
        `Dockerfile generated for project ${config.name} in ${result.buildTime}ms`
      );
      return result;
    } catch (error) {
      const errorMessage =
        error instanceof Error ? error.message : 'Unknown error';
      errors.push(errorMessage);

      return {
        content: '',
        path: '',
        errors,
        warnings,
        optimizationSuggestions,
        securityIssues,
        buildTime: Date.now() - startTime,
        size: 0,
      };
    }
  }

  /**
   * Generate Dockerfile based on new project configuration schema
   */
  async generateFromProjectConfig(
    projectId: string,
    config: any,
    options: DockerfileOptions = {}
  ): Promise<DockerfileResult> {
    try {
      logger.info(
        `Generating Dockerfile for project ${projectId} with new config schema`
      );

      // Map config to Dockerfile generation parameters
      const dockerfileConfig = this.mapConfigToDockerfileConfig(config);

      // Generate base Dockerfile content directly
      const result: DockerfileResult = {
        content: this.generateBaseDockerfileContent(dockerfileConfig),
        path: '',
        errors: [],
        warnings: [],
        optimizationSuggestions: [],
        securityIssues: [],
        buildTime: 0,
        size: 0,
      };

      // Add RMW configuration
      if (config.rmw) {
        result.content = this.addRMWConfiguration(result.content, config.rmw);
      }

      // Add simulation packages if needed
      if (config.simulation && config.simulation !== 'none') {
        result.content = this.addSimulationPackages(
          result.content,
          config.simulation
        );
      }

      // Add teleop packages if needed
      if (config.teleopKeyboard || config.teleopJoystick) {
        result.content = this.addTeleopPackages(result.content, config);
      }

      // Add Foxglove if enabled
      if (config.foxglove) {
        result.content = this.addFoxgloveConfiguration(result.content);
      }

      return result;
    } catch (error) {
      logger.error(`Error generating Dockerfile from project config: ${error}`);
      throw error;
    }
  }

  /**
   * Generate base Dockerfile content from configuration
   */
  private generateBaseDockerfileContent(config: ProjectConfiguration): string {
    let content = `# Generated Dockerfile for ${config.name}
# Project: ${config.name}
# Type: ${config.type}
# Generated: ${new Date().toISOString()}

FROM ${config.baseImage}

# Set working directory
WORKDIR ${config.workdir}

# Install system dependencies
RUN apt-get update && apt-get install -y \\
    ${
      config.systemDependencies?.join(' \\\n    ') ||
      'build-essential git wget curl'
    } \\
    && rm -rf /var/lib/apt/lists/*

# Set environment variables
${Object.entries(config.environmentVariables || {})
  .map(([key, value]) => `ENV ${key}=${value}`)
  .join('\n')}

# Expose ports
${config.ports?.map((port) => `EXPOSE ${port}`).join('\n') || ''}

# Copy application code
COPY . .

# Set default command
CMD ${config.command || '["bash"]'}
`;

    return content;
  }

  /**
   * Map new project config to Dockerfile generation parameters
   */
  private mapConfigToDockerfileConfig(config: any): ProjectConfiguration {
    // Map base image
    let baseImage: string;
    switch (config.baseImage) {
      case 'ros_humble':
        baseImage = 'ros:humble-ros-base-jammy';
        break;
      case 'cuda_ubuntu2204':
        baseImage = 'nvidia/cuda:11.8-devel-ubuntu22.04';
        break;
      case 'jetson_l4t_ros':
        baseImage = 'nvcr.io/nvidia/l4t-ros:r35.4.1-ros2-humble';
        break;
      default:
        baseImage = 'ros:humble-ros-base-jammy';
    }

    // Map ROS version
    const rosVersion = config.rosVersion || 'humble';

    // Determine system dependencies based on configuration
    const systemDependencies: string[] = [
      'build-essential',
      'cmake',
      'git',
      'wget',
      'curl',
      'python3-pip',
      'python3-dev',
    ];

    // Add simulation dependencies
    if (config.simulation === 'gazebo') {
      systemDependencies.push('gazebo', 'libgazebo-dev');
    } else if (config.simulation === 'isaac') {
      systemDependencies.push('nvidia-container-toolkit');
    }

    // Add teleop dependencies
    if (config.teleopJoystick) {
      systemDependencies.push('joystick', 'jstest-gtk');
    }

    return {
      id: 'project-config',
      name: 'Project Configuration',
      version: '1.0.0',
      type: 'custom',
      baseImage,
      workdir: '/workspace',
      systemDependencies,
      environmentVariables: {
        ROS_DISTRO: rosVersion,
        RMW_IMPLEMENTATION: config.rmw || 'cyclonedds',
        ...(config.simulation === 'isaac' && { NVIDIA_VISIBLE_DEVICES: 'all' }),
      },
      ports: config.foxglove ? ['9090'] : [],
      command: this.getStartupCommand(config),
    };
  }

  /**
   * Add RMW configuration to Dockerfile
   */
  private addRMWConfiguration(dockerfileContent: string, rmw: string): string {
    const rmwConfig = `
# Configure RMW implementation
ENV RMW_IMPLEMENTATION=${rmw}
`;

    // Insert after FROM line
    const fromIndex = dockerfileContent.indexOf('FROM');
    const insertIndex = dockerfileContent.indexOf('\n', fromIndex) + 1;

    return (
      dockerfileContent.slice(0, insertIndex) +
      rmwConfig +
      dockerfileContent.slice(insertIndex)
    );
  }

  /**
   * Add simulation packages to Dockerfile
   */
  private addSimulationPackages(
    dockerfileContent: string,
    simulation: string
  ): string {
    let packages = '';

    if (simulation === 'gazebo') {
      packages = `
# Install Gazebo simulation packages
RUN apt-get update && apt-get install -y \\
    gazebo \\
    libgazebo-dev \\
    ros-\${ROS_DISTRO}-gazebo-ros-pkgs \\
    ros-\${ROS_DISTRO}-gazebo-ros-control \\
    && rm -rf /var/lib/apt/lists/*
`;
    } else if (simulation === 'isaac') {
      packages = `
# Install Isaac Sim dependencies
RUN apt-get update && apt-get install -y \\
    nvidia-container-toolkit \\
    && rm -rf /var/lib/apt/lists/*
`;
    }

    // Insert before WORKDIR
    const workdirIndex = dockerfileContent.indexOf('WORKDIR');
    const insertIndex = workdirIndex;

    return (
      dockerfileContent.slice(0, insertIndex) +
      packages +
      dockerfileContent.slice(insertIndex)
    );
  }

  /**
   * Add teleop packages to Dockerfile
   */
  private addTeleopPackages(dockerfileContent: string, config: any): string {
    let packages = '';

    if (config.teleopKeyboard) {
      packages += `
# Install keyboard teleop
RUN apt-get update && apt-get install -y \\
    ros-\${ROS_DISTRO}-teleop-twist-keyboard \\
    && rm -rf /var/lib/apt/lists/*
`;
    }

    if (config.teleopJoystick) {
      packages += `
# Install joystick teleop
RUN apt-get update && apt-get install -y \\
    joystick \\
    jstest-gtk \\
    ros-\${ROS_DISTRO}-joy \\
    ros-\${ROS_DISTRO}-teleop-twist-joy \\
    && rm -rf /var/lib/apt/lists/*
`;
    }

    // Insert before WORKDIR
    const workdirIndex = dockerfileContent.indexOf('WORKDIR');
    const insertIndex = workdirIndex;

    return (
      dockerfileContent.slice(0, insertIndex) +
      packages +
      dockerfileContent.slice(insertIndex)
    );
  }

  /**
   * Add Foxglove configuration to Dockerfile
   */
  private addFoxgloveConfiguration(dockerfileContent: string): string {
    const foxgloveConfig = `
# Install Foxglove Studio Bridge
RUN pip3 install foxglove-bridge

# Expose Foxglove port
EXPOSE 9090
`;

    // Insert before CMD
    const cmdIndex = dockerfileContent.indexOf('CMD');
    const insertIndex = cmdIndex;

    return (
      dockerfileContent.slice(0, insertIndex) +
      foxgloveConfig +
      dockerfileContent.slice(insertIndex)
    );
  }

  /**
   * Get startup command based on configuration
   */
  private getStartupCommand(config: any): string {
    const commands: string[] = [];

    // Add simulation startup
    if (config.simulation === 'gazebo') {
      commands.push('roslaunch gazebo_ros empty_world.launch');
    } else if (config.simulation === 'isaac') {
      commands.push('isaac-sim');
    }

    // Add teleop startup
    if (config.teleopKeyboard) {
      commands.push('rosrun teleop_twist_keyboard teleop_twist_keyboard.py');
    }
    if (config.teleopJoystick) {
      commands.push('rosrun joy joy_node');
    }

    // Add Foxglove bridge
    if (config.foxglove) {
      commands.push('foxglove-bridge');
    }

    // Default to bash if no specific commands
    if (commands.length === 0) {
      commands.push('bash');
    }

    return commands.join(' & ');
  }

  /**
   * Get default template for project type
   */
  private getDefaultTemplate(projectType: string): string {
    switch (projectType) {
      case 'python':
        return 'python-dockerfile';
      case 'nodejs':
        return 'nodejs-dockerfile';
      case 'java':
        return 'python-dockerfile'; // Use Python template as fallback
      case 'golang':
        return 'python-dockerfile'; // Use Python template as fallback
      case 'rust':
        return 'python-dockerfile'; // Use Python template as fallback
      case 'cpp':
        return 'python-dockerfile'; // Use Python template as fallback
      case 'custom':
        return 'python-dockerfile'; // Use Python template as fallback
      default:
        return 'python-dockerfile'; // Use Python template as fallback
    }
  }

  /**
   * Prepare context for template processing
   */
  private prepareTemplateContext(config: ProjectConfiguration): any {
    return {
      project: {
        name: config.name,
        version: config.version,
        description: config.description,
        type: config.type,
      },
      baseImage: config.baseImage,
      workdir: config.workdir,
      systemDependencies: config.systemDependencies || [],
      pythonDependencies: config.pythonDependencies || [],
      nodeDependencies: config.nodeDependencies || [],
      javaDependencies: config.javaDependencies || [],
      golangDependencies: config.golangDependencies || [],
      rustDependencies: config.rustDependencies || [],
      cppDependencies: config.cppDependencies || [],
      customDependencies: config.customDependencies || [],
      environmentVariables: config.environmentVariables || {},
      ports: config.ports || [],
      volumes: config.volumes || [],
      command: config.command,
      entrypoint: config.entrypoint,
      user: config.user,
      healthCheck: config.healthCheck,
      buildArgs: config.buildArgs || {},
      labels: config.labels || {},
      multiStage: config.multiStage || false,
      stages: config.stages || [],
    };
  }

  /**
   * Validate generated Dockerfile
   */
  private validateDockerfile(
    content: string,
    config: ProjectConfiguration
  ): DockerfileValidation {
    const errors: string[] = [];
    const warnings: string[] = [];
    const suggestions: string[] = [];

    // Check for required sections
    if (!content.includes('FROM')) {
      errors.push('Missing FROM instruction');
    }

    if (!content.includes('WORKDIR')) {
      warnings.push('Missing WORKDIR instruction');
    }

    // Check for security issues
    if (
      content.includes('RUN apt-get update') &&
      !content.includes('rm -rf /var/lib/apt/lists/')
    ) {
      warnings.push('Consider cleaning up apt cache to reduce image size');
    }

    // Check for optimization opportunities
    if (
      content.includes('COPY . .') &&
      content.includes('COPY package*.json')
    ) {
      suggestions.push(
        'Consider copying package files before copying all files for better layer caching'
      );
    }

    if (content.includes('USER root') || !content.includes('USER')) {
      suggestions.push('Consider running as non-root user for security');
    }

    // Check for project-specific validations
    if (config.type === 'python' && !content.includes('pip install')) {
      warnings.push(
        'Python project should include pip install for dependencies'
      );
    }

    if (config.type === 'nodejs' && !content.includes('npm install')) {
      warnings.push(
        'Node.js project should include npm install for dependencies'
      );
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      suggestions,
    };
  }

  /**
   * Security scan for Dockerfile
   */
  private securityScanDockerfile(content: string): string[] {
    const issues: string[] = [];

    // Check for security issues
    if (content.includes('USER root')) {
      issues.push('Running as root user is a security risk');
    }

    if (content.includes('--no-sandbox')) {
      issues.push('--no-sandbox flag can be a security risk');
    }

    if (content.includes('chmod 777')) {
      issues.push('chmod 777 gives excessive permissions');
    }

    if (content.includes('curl -s') && content.includes('| bash')) {
      issues.push('Piping curl to bash can be dangerous');
    }

    if (content.includes('wget -O-') && content.includes('| bash')) {
      issues.push('Piping wget to bash can be dangerous');
    }

    return issues;
  }

  /**
   * Optimize Dockerfile content
   */
  private optimizeDockerfile(content: string): string {
    let optimized = content;

    // Combine RUN commands to reduce layers
    const runCommands = optimized.match(/RUN[^\n]+/g) || [];
    if (runCommands.length > 1) {
      // This is a simplified optimization - in practice, you'd need more sophisticated logic
      // optimizationSuggestions.push('Consider combining RUN commands to reduce layers'); // This line was removed from the new_code, so it's removed here.
    }

    // Remove unnecessary whitespace
    optimized = optimized.replace(/\n\s*\n\s*\n/g, '\n\n');

    return optimized;
  }

  /**
   * Add comments to Dockerfile
   */
  private addComments(content: string, config: ProjectConfiguration): string {
    let commented = content;

    // Add header comment
    const header = `# Generated Dockerfile for ${config.name}
# Project: ${config.name} v${config.version}
# Type: ${config.type}
# Generated: ${new Date().toISOString()}
# 
`;

    commented = header + commented;

    return commented;
  }

  /**
   * Generate output path for Dockerfile
   */
  private generateOutputPath(
    projectId: string,
    config: ProjectConfiguration
  ): string {
    const filename = `Dockerfile.${config.name
      .toLowerCase()
      .replace(/[^a-z0-9]/g, '-')}`;
    return path.join(this.defaultOutputDir, projectId, filename);
  }

  /**
   * Write Dockerfile to disk
   */
  private async writeDockerfile(
    filePath: string,
    content: string
  ): Promise<void> {
    try {
      // Ensure directory exists
      const dir = path.dirname(filePath);
      await fsPromises.mkdir(dir, { recursive: true });

      // Write file
      await fsPromises.writeFile(filePath, content, 'utf8');
      logger.info(`Dockerfile written to: ${filePath}`);
    } catch (error) {
      logger.error(
        `Failed to write Dockerfile to ${filePath}:`,
        undefined,
        error as Error
      );
      throw error;
    }
  }

  /**
   * Get generated Dockerfile result
   */
  getGeneratedDockerfile(projectId: string): DockerfileResult | undefined {
    return this.generatedFiles.get(projectId);
  }

  /**
   * List all generated Dockerfiles
   */
  listGeneratedDockerfiles(): string[] {
    return Array.from(this.generatedFiles.keys());
  }

  /**
   * Remove generated Dockerfile
   */
  removeGeneratedDockerfile(projectId: string): boolean {
    const result = this.generatedFiles.get(projectId);
    if (result && result.path) {
      try {
        fs.unlinkSync(result.path);
        logger.info(`Removed Dockerfile: ${result.path}`);
      } catch (error) {
        logger.warn(`Failed to remove Dockerfile ${result.path}: ${error}`);
      }
    }

    const removed = this.generatedFiles.delete(projectId);
    if (removed) {
      logger.info(`Removed generated Dockerfile for project: ${projectId}`);
    }

    return removed;
  }

  /**
   * Get service statistics
   */
  getServiceStats(): {
    totalProjects: number;
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
      totalProjects: this.projectConfigs.size,
      totalGeneratedFiles: this.generatedFiles.size,
      totalErrors,
      totalWarnings,
    };
  }

  // ROS-specific methods
  private moduleLoader: ModuleLoader = new ModuleLoader();

  /**
   * Initialize ROS module loader
   */
  async initializeROSModules(): Promise<void> {
    try {
      await this.moduleLoader.loadModules();
      logger.info('ROS module loader initialized successfully');
    } catch (error) {
      logger.error(
        'Failed to initialize ROS module loader:',
        undefined,
        error as Error
      );
      throw error;
    }
  }

  /**
   * Generate ROS Dockerfiles with multi-stage support
   */
  async generateROSDockerfiles(
    config: ROSProjectConfig,
    options: ROSDockerfileOptions = {}
  ): Promise<ROSDockerfileResult> {
    const startTime = Date.now();
    const errors: string[] = [];
    const warnings: string[] = [];
    const optimizationSuggestions: string[] = [];
    const securityIssues: string[] = [];

    try {
      // Ensure module loader is initialized
      await this.initializeROSModules();

      // Get robot and modules
      const robot = this.moduleLoader.getRobot(config.robot);
      if (!robot) {
        throw new Error(`Robot not found: ${config.robot}`);
      }

      const modules = config.modules
        .map((id) => this.moduleLoader.getModule(id))
        .filter(Boolean) as ModuleSpec[];

      // Resolve dependencies
      const resolvedModuleIds = this.moduleLoader.resolveDependencies(
        config.modules
      );
      const resolvedModules = resolvedModuleIds
        .map((id) => this.moduleLoader.getModule(id))
        .filter(Boolean) as ModuleSpec[];

      // Prepare generation context
      const context: ROSGenerationContext = {
        distro: config.distro,
        robot,
        modules: resolvedModules,
        customizations: config.customizations,
        buildProfile: config.buildProfile || 'development',
        workdir: '/ros_workspace',
        user: 'rosuser',
      };

      // Generate multi-stage Dockerfile
      const dockerfileContent = await this.generateMultiStageDockerfile(
        context
      );

      // Generate docker-compose.yml if requested
      let composeContent: string | undefined;
      let composePath: string | undefined;
      if (options.includeCompose) {
        composeContent = await this.generateDockerCompose(context);
        composePath = path.join(
          options.outputDir || this.defaultOutputDir,
          'docker-compose.yml'
        );
      }

      // Generate docker-bake.hcl if requested
      let bakeContent: string | undefined;
      let bakePath: string | undefined;
      if (options.includeBake) {
        bakeContent = await this.generateDockerBake(context);
        bakePath = path.join(
          options.outputDir || this.defaultOutputDir,
          'docker-bake.hcl'
        );
      }

      // Write files if not validate-only
      if (!options.validateOnly) {
        const outputDir = options.outputDir || this.defaultOutputDir;
        await fsPromises.mkdir(outputDir, { recursive: true });

        // Write Dockerfile
        const dockerfilePath = path.join(outputDir, 'Dockerfile');
        await fsPromises.writeFile(dockerfilePath, dockerfileContent, 'utf8');

        // Write docker-compose.yml
        if (composeContent && composePath) {
          await fsPromises.writeFile(composePath, composeContent, 'utf8');
        }

        // Write docker-bake.hcl
        if (bakeContent && bakePath) {
          await fsPromises.writeFile(bakePath, bakeContent, 'utf8');
        }
      }

      const result: ROSDockerfileResult = {
        content: dockerfileContent,
        path: path.join(
          options.outputDir || this.defaultOutputDir,
          'Dockerfile'
        ),
        errors,
        warnings,
        optimizationSuggestions,
        securityIssues,
        buildTime: Date.now() - startTime,
        size: dockerfileContent.length,
        composeContent,
        bakeContent,
        composePath,
        bakePath,
      };

      logger.info(
        `ROS Dockerfiles generated for ${robot.title} in ${result.buildTime}ms`
      );
      return result;
    } catch (error) {
      const errorMessage =
        error instanceof Error ? error.message : 'Unknown error';
      errors.push(errorMessage);

      return {
        content: '',
        path: '',
        errors,
        warnings,
        optimizationSuggestions,
        securityIssues,
        buildTime: Date.now() - startTime,
        size: 0,
      };
    }
  }

  /**
   * Generate multi-stage Dockerfile for ROS
   */
  private async generateMultiStageDockerfile(
    context: ROSGenerationContext
  ): Promise<string> {
    const { robot, modules, customizations, distro, workdir, user } = context;

    // Load templates
    const baseTemplate = await this.loadTemplate('base.hbs');
    const robotTemplate = await this.loadTemplate('robot.hbs');
    const moduleTemplate = await this.loadTemplate('module.hbs');
    const customizationsTemplate = await this.loadTemplate(
      'customizations.hbs'
    );

    // Generate base stage
    const baseContext = {
      distro,
      baseImage: robot.baseImage,
      workdir,
      apt: robot.apt,
      pip: robot.pip,
    };
    const baseStage = baseTemplate(baseContext);

    // Generate robot stage
    const robotContext = {
      robotName: robot.title,
      baseImage: 'base',
      apt: robot.apt,
      pip: robot.pip,
      env: robot.env,
      setupCommands: robot.setupCommands,
    };
    const robotStage = robotTemplate(robotContext);

    // Generate module stages
    const moduleStages = modules
      .map((module) => {
        const moduleContext = {
          moduleName: module.title,
          baseImage: 'robot',
          apt: module.apt,
          pip: module.pip,
          env: module.env,
          setupCommands: module.setupCommands,
        };
        return moduleTemplate(moduleContext);
      })
      .join('\n\n');

    // Generate customizations stage
    const customizationsContext = {
      baseImage: modules.length > 0 ? 'module' : 'robot',
      workdir,
      udevRules: [...(robot.udevRules || []), ...customizations.udevRules],
      files: [...(robot.files || []), ...customizations.files],
      rosDistro: distro,
      user,
      bashrcAliases: [
        ...(robot.bashrcAliases || []),
        ...customizations.bashrcAliases,
      ],
      env: { ...robot.env, ...customizations.env },
      expose: [...(robot.expose || []), ...customizations.expose],
    };
    const customizationsStage = customizationsTemplate(customizationsContext);

    // Combine all stages
    return `${baseStage}\n\n${robotStage}\n\n${moduleStages}\n\n${customizationsStage}`;
  }

  /**
   * Generate docker-compose.yml for ROS
   */
  private async generateDockerCompose(
    context: ROSGenerationContext
  ): Promise<string> {
    const { robot, customizations, workdir } = context;

    const composeTemplate = `version: '3.8'

services:
  ros-workspace:
    build:
      context: .
      dockerfile: Dockerfile
    container_name: robium-ros-${robot.id}
    volumes:
      - .:${workdir}
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    environment:
      - DISPLAY=\${DISPLAY}
      - QT_X11_NO_MITSHM=1
{{#each env}}
      - {{@key}}={{this}}
{{/each}}
    working_dir: ${workdir}
    command: tail -f /dev/null
    stdin_open: true
    tty: true
    extra_hosts:
      - "host.docker.internal:host-gateway"
{{#each expose}}
    ports:
      - "{{this}}:{{this}}"
{{/each}}`;

    const template = Handlebars.compile(composeTemplate);
    return template({
      env: { ...robot.env, ...customizations.env },
      expose: [...(robot.expose || []), ...customizations.expose],
    });
  }

  /**
   * Generate docker-bake.hcl for ROS
   */
  private async generateDockerBake(
    context: ROSGenerationContext
  ): Promise<string> {
    const { robot, modules, distro } = context;

    const bakeTemplate = await this.loadTemplate('bake.hcl');

    const stages = [
      {
        id: 'base',
        dockerfile: 'Dockerfile',
        args: { BASE: robot.baseImage },
        tags: [`robium/ros-${robot.id}:${distro}-base`],
      },
      {
        id: 'robot',
        dockerfile: 'Dockerfile',
        args: { BASE: 'base' },
        tags: [`robium/ros-${robot.id}:${distro}-robot`],
      },
      ...modules.map((module) => ({
        id: module.id,
        dockerfile: 'Dockerfile',
        args: { BASE: 'robot' },
        tags: [`robium/ros-${robot.id}:${distro}-${module.id}`],
      })),
      {
        id: 'final',
        dockerfile: 'Dockerfile',
        args: {
          BASE: modules.length > 0 ? modules[modules.length - 1].id : 'robot',
        },
        tags: [`robium/ros-${robot.id}:${distro}-latest`],
      },
    ];

    return bakeTemplate({
      targets: stages.map((s) => s.id),
      stages,
    });
  }

  /**
   * Load Handlebars template from file
   */
  private async loadTemplate(
    templateName: string
  ): Promise<Handlebars.TemplateDelegate> {
    const templatePath = path.join(
      __dirname,
      '..',
      'templates',
      'dockerfile',
      templateName
    );
    const templateContent = await fsPromises.readFile(templatePath, 'utf8');
    return Handlebars.compile(templateContent);
  }

  /**
   * Get available ROS distributions
   */
  getROSDistros(): string[] {
    return ['humble', 'foxy', 'noetic', 'iron'];
  }

  /**
   * Get available robots for a distribution
   */
  getRobotsByDistro(distro: string): RobotSpec[] {
    return this.moduleLoader.getRobotsByDistro(distro);
  }

  /**
   * Get compatible modules for a robot
   */
  getCompatibleModules(robotId: string): ModuleSpec[] {
    return this.moduleLoader.getCompatibleModules(robotId);
  }

  /**
   * Clean up old generated files
   */
  cleanupOldFiles(maxAgeDays: number = 7): void {
    const cutoffTime = Date.now() - maxAgeDays * 24 * 60 * 60 * 1000;
    let cleanedCount = 0;

    for (const [projectId, result] of this.generatedFiles.entries()) {
      try {
        const stats = fs.statSync(result.path);
        if (stats.mtime.getTime() < cutoffTime) {
          this.removeGeneratedDockerfile(projectId);
          cleanedCount++;
        }
      } catch (error) {
        // File might not exist, remove from tracking
        this.generatedFiles.delete(projectId);
      }
    }

    logger.info(`Cleaned up ${cleanedCount} old generated files`);
  }
}

// Export singleton instance
export const dockerfileGenerationService = new DockerfileGenerationService();
