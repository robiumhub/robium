import { templateEngine } from './TemplateEngine';
import { logger } from '../utils/logger';
import fs from 'fs';
import path from 'path';
import { promises as fsPromises } from 'fs';

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
    templateEngine.registerTemplate('multistage-dockerfile', multiStageTemplate);

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
      const templateName = options.template || this.getDefaultTemplate(config.type);
      
      // Prepare context for template processing
      const context = this.prepareTemplateContext(config);

      // Process template
      const templateResult = templateEngine.processTemplate(templateName, context, {
        validateOnly: options.validateOnly
      });

      if (templateResult.errors.length > 0) {
        errors.push(...templateResult.errors.map(e => e.message));
      }

      if (templateResult.warnings.length > 0) {
        warnings.push(...templateResult.warnings);
      }

      // Validate generated Dockerfile
      const validation = this.validateDockerfile(templateResult.content, config);
      errors.push(...validation.errors);
      warnings.push(...validation.warnings);
      optimizationSuggestions.push(...validation.suggestions);

      // Security scan if requested
      if (options.securityScan) {
        const securityScan = this.securityScanDockerfile(templateResult.content);
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
      const outputPath = options.outputPath || this.generateOutputPath(projectId, config);

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

      logger.info(`Dockerfile generated for project ${config.name} in ${result.buildTime}ms`);
      return result;

    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
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
  private validateDockerfile(content: string, config: ProjectConfiguration): DockerfileValidation {
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
    if (content.includes('RUN apt-get update') && !content.includes('rm -rf /var/lib/apt/lists/')) {
      warnings.push('Consider cleaning up apt cache to reduce image size');
    }

    // Check for optimization opportunities
    if (content.includes('COPY . .') && content.includes('COPY package*.json')) {
      suggestions.push('Consider copying package files before copying all files for better layer caching');
    }

    if (content.includes('USER root') || !content.includes('USER')) {
      suggestions.push('Consider running as non-root user for security');
    }

    // Check for project-specific validations
    if (config.type === 'python' && !content.includes('pip install')) {
      warnings.push('Python project should include pip install for dependencies');
    }

    if (config.type === 'nodejs' && !content.includes('npm install')) {
      warnings.push('Node.js project should include npm install for dependencies');
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
  private generateOutputPath(projectId: string, config: ProjectConfiguration): string {
    const filename = `Dockerfile.${config.name.toLowerCase().replace(/[^a-z0-9]/g, '-')}`;
    return path.join(this.defaultOutputDir, projectId, filename);
  }

  /**
   * Write Dockerfile to disk
   */
  private async writeDockerfile(filePath: string, content: string): Promise<void> {
    try {
      // Ensure directory exists
      const dir = path.dirname(filePath);
      await fsPromises.mkdir(dir, { recursive: true });

      // Write file
      await fsPromises.writeFile(filePath, content, 'utf8');
      logger.info(`Dockerfile written to: ${filePath}`);
    } catch (error) {
      logger.error(`Failed to write Dockerfile to ${filePath}:`, undefined, error as Error);
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