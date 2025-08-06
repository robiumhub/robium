import { logger } from '../utils/logger';

export interface TemplateVariable {
  name: string;
  value: string | number | boolean | object;
  type: 'string' | 'number' | 'boolean' | 'object' | 'array';
  required?: boolean;
  default?: any;
  description?: string;
}

export interface TemplateCondition {
  variable: string;
  operator: 'eq' | 'ne' | 'gt' | 'lt' | 'gte' | 'lte' | 'exists' | 'not_exists' | 'in' | 'not_in';
  value?: any;
  values?: any[];
}

export interface TemplateIteration {
  variable: string;
  items: string | any[];
  template: string;
  separator?: string;
}

export interface TemplateConfig {
  variables: TemplateVariable[];
  conditions: TemplateCondition[];
  iterations: TemplateIteration[];
  includes?: string[];
  extends?: string;
}

export interface TemplateContext {
  [key: string]: any;
}

export interface TemplateError {
  line: number;
  column: number;
  message: string;
  type: 'syntax' | 'variable' | 'condition' | 'iteration' | 'validation';
}

export interface TemplateResult {
  content: string;
  errors: TemplateError[];
  warnings: string[];
  variables: string[];
  processingTime: number;
}

export class TemplateEngine {
  private templates: Map<string, string> = new Map();
  private configs: Map<string, TemplateConfig> = new Map();
  private cache: Map<string, TemplateResult> = new Map();
  private maxCacheSize: number = 100;

  constructor() {
    this.initializeDefaultTemplates();
  }

  /**
   * Initialize default templates
   */
  private initializeDefaultTemplates(): void {
    // Default Dockerfile template
    const defaultDockerfile = `# Generated Dockerfile for {{project.name}}
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

# Install Python dependencies
{{#if pythonDependencies}}
COPY requirements.txt .
RUN pip install -r requirements.txt
{{/if}}

# Install Node.js dependencies
{{#if nodeDependencies}}
COPY package*.json ./
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

# Set default command
{{#if command}}
CMD {{command}}
{{else}}
CMD ["bash"]
{{/if}}`;

    // Default docker-compose template
    const defaultCompose = `# Generated docker-compose.yml for {{project.name}}
version: '{{composeVersion}}'

services:
  {{project.name}}:
    build:
      context: .
      dockerfile: Dockerfile
    container_name: {{containerName}}
    environment:
{{#each environmentVariables}}
      - {{@key}}={{this}}
{{/each}}
    ports:
{{#each ports}}
      - "{{this}}"
{{/each}}
    volumes:
{{#each volumes}}
      - {{this}}
{{/each}}
    networks:
      - {{networkName}}
    {{#if restartPolicy}}
    restart: {{restartPolicy}}
    {{/if}}
    {{#if dependsOn}}
    depends_on:
{{#each dependsOn}}
      - {{this}}
{{/each}}
    {{/if}}

networks:
  {{networkName}}:
    driver: bridge

volumes:
{{#each namedVolumes}}
  {{@key}}:
    driver: local
{{/each}}`;

    this.registerTemplate('default-dockerfile', defaultDockerfile);
    this.registerTemplate('default-compose', defaultCompose);

    logger.info('Initialized default templates');
  }

  /**
   * Register a template with the engine
   */
  registerTemplate(name: string, template: string, config?: TemplateConfig): void {
    this.templates.set(name, template);
    if (config) {
      this.configs.set(name, config);
    }
    logger.info(`Registered template: ${name}`);
  }

  /**
   * Get a registered template
   */
  getTemplate(name: string): string | undefined {
    return this.templates.get(name);
  }

  /**
   * Process a template with context data
   */
  processTemplate(
    templateName: string,
    context: TemplateContext,
    options?: {
      cacheKey?: string;
      validateOnly?: boolean;
    }
  ): TemplateResult {
    const startTime = Date.now();
    const errors: TemplateError[] = [];
    const warnings: string[] = [];
    const variables: string[] = [];

    try {
      // Check cache first
      const cacheKey = options?.cacheKey || this.generateCacheKey(templateName, context);
      if (!options?.validateOnly && this.cache.has(cacheKey)) {
        const cached = this.cache.get(cacheKey)!;
        logger.info(`Template cache hit: ${templateName}`);
        return {
          ...cached,
          processingTime: Date.now() - startTime,
        };
      }

      // Get template
      const template = this.templates.get(templateName);
      if (!template) {
        errors.push({
          line: 0,
          column: 0,
          message: `Template not found: ${templateName}`,
          type: 'syntax',
        });
        return { content: '', errors, warnings, variables, processingTime: Date.now() - startTime };
      }

      // Validate template syntax
      const syntaxErrors = this.validateTemplateSyntax(template);
      errors.push(...syntaxErrors);

      if (errors.length > 0) {
        return { content: '', errors, warnings, variables, processingTime: Date.now() - startTime };
      }

      // Extract variables from template
      const templateVariables = this.extractVariables(template);
      variables.push(...templateVariables);

      if (options?.validateOnly) {
        // For validation only, check variables before processing
        const validationErrors = this.validateVariables(templateVariables, context);
        errors.push(...validationErrors);
        return { content: '', errors, warnings, variables, processingTime: Date.now() - startTime };
      }

      // Process template
      let content = template;

      // Process variable substitutions
      content = this.processVariableSubstitutions(content, context);

      // Process conditional blocks
      content = this.processConditionalBlocks(content, context);

      // Process iterations
      content = this.processIterations(content, context);

      // Process includes
      content = this.processIncludes(content, context);

      // After processing, check for any remaining unresolved variables
      const remainingVariables = this.extractVariables(content);
      const validationErrors = this.validateVariables(remainingVariables, context);
      errors.push(...validationErrors);

      // Clean up empty lines and formatting
      content = this.cleanupContent(content);

      const result: TemplateResult = {
        content,
        errors,
        warnings,
        variables,
        processingTime: Date.now() - startTime,
      };

      // Cache result
      this.cacheResult(cacheKey, result);

      logger.info(`Template processed successfully: ${templateName} (${result.processingTime}ms)`);
      return result;

    } catch (error) {
      errors.push({
        line: 0,
        column: 0,
        message: error instanceof Error ? error.message : 'Unknown error',
        type: 'syntax',
      });
      return { content: '', errors, warnings, variables, processingTime: Date.now() - startTime };
    }
  }

  /**
   * Process variable substitutions {{variable}}
   */
  private processVariableSubstitutions(template: string, context: TemplateContext): string {
    return template.replace(/\{\{([^}]+)\}\}/g, (match, variable) => {
      const trimmed = variable.trim();
      
      // Handle nested object access (e.g., project.name)
      const value = this.getNestedValue(context, trimmed);
      
      if (value === undefined) {
        return match; // Keep original if variable not found
      }
      
      return String(value);
    });
  }

  /**
   * Process conditional blocks {{#if condition}}...{{/if}}
   */
  private processConditionalBlocks(template: string, context: TemplateContext): string {
    return template.replace(/\{\{#if\s+([^}]+)\}\}([\s\S]*?)\{\{\/if\}\}/g, (match, condition, content) => {
      const trimmed = condition.trim();
      const value = this.getNestedValue(context, trimmed);
      
      if (this.evaluateCondition(value)) {
        return content;
      }
      
      return '';
    });
  }

  /**
   * Process iterations {{#each items}}...{{/each}}
   */
  private processIterations(template: string, context: TemplateContext): string {
    let result = template;
    let processed = false;
    
    // Process iterations from innermost to outermost
    do {
      processed = false;
      result = result.replace(/\{\{#each\s+([^}]+)\}\}([\s\S]*?)\{\{\/each\}\}/g, (match, items, content) => {
        const trimmed = items.trim();
        const value = this.getNestedValue(context, trimmed);
        
        if (!Array.isArray(value)) {
          return '';
        }
        
        processed = true;
        
        return value.map((item, index) => {
          let itemContent = content;
          
          // Create a context for this iteration that includes the current item
          const iterationContext = { ...context, ...item };
          
          // Replace {{this}} with current item
          const itemString = String(item);
          itemContent = itemContent.replace(/\{\{this\}\}/g, itemString);
          
          // Replace {{@key}} with index
          itemContent = itemContent.replace(/\{\{@key\}\}/g, String(index));
          
          // Process nested iterations and conditionals recursively
          itemContent = this.processIterations(itemContent, iterationContext);
          itemContent = this.processConditionalBlocks(itemContent, iterationContext);
          
          // Replace other variables using the iteration context
          itemContent = this.processVariableSubstitutions(itemContent, iterationContext);
          
          return itemContent;
        }).join('\n');
      });
    } while (processed);
    
    return result;
  }

  /**
   * Process includes {{#include templateName}}
   */
  private processIncludes(template: string, context: TemplateContext): string {
    return template.replace(/\{\{#include\s+([^}]+)\}\}/g, (match, templateName) => {
      const trimmed = templateName.trim();
      const includedTemplate = this.templates.get(trimmed);
      
      if (includedTemplate) {
        return this.processTemplate(trimmed, context).content;
      }
      
      return '';
    });
  }

  /**
   * Get nested object value
   */
  private getNestedValue(obj: any, path: string): any {
    return path.split('.').reduce((current, key) => {
      return current && current[key] !== undefined ? current[key] : undefined;
    }, obj);
  }

  /**
   * Evaluate condition
   */
  private evaluateCondition(value: any): boolean {
    if (typeof value === 'boolean') {
      return value;
    }
    if (typeof value === 'string') {
      return value.length > 0;
    }
    if (typeof value === 'number') {
      return value !== 0;
    }
    if (Array.isArray(value)) {
      return value.length > 0;
    }
    if (typeof value === 'object') {
      return value !== null;
    }
    return false;
  }

  /**
   * Extract variables from template
   */
  private extractVariables(template: string): string[] {
    const variables: string[] = [];
    const variableRegex = /\{\{([^}]+)\}\}/g;
    let match;

    while ((match = variableRegex.exec(template)) !== null) {
      const variable = match[1].trim();
      
      // Skip control structures and closing tags
      if (!variable.startsWith('#if') && 
          !variable.startsWith('#each') && 
          !variable.startsWith('#include') &&
          !variable.startsWith('/if') &&
          !variable.startsWith('/each') &&
          !variable.startsWith('/include')) {
        variables.push(variable);
      }
    }

    return [...new Set(variables)]; // Remove duplicates
  }

  /**
   * Validate template syntax
   */
  private validateTemplateSyntax(template: string): TemplateError[] {
    const errors: TemplateError[] = [];
    const lines = template.split('\n');

    // Track overall nesting structure
    let ifStack: number[] = [];
    let eachStack: number[] = [];

    for (let i = 0; i < lines.length; i++) {
      const line = lines[i];
      const lineNumber = i + 1;

      // Check for opening and closing tags
      const ifMatches = line.match(/\{\{#if/g) || [];
      const endIfMatches = line.match(/\{\{\/if\}\}/g) || [];
      const eachMatches = line.match(/\{\{#each/g) || [];
      const endEachMatches = line.match(/\{\{\/each\}\}/g) || [];

      // Process opening tags
      for (let j = 0; j < ifMatches.length; j++) {
        ifStack.push(lineNumber);
      }
      for (let j = 0; j < eachMatches.length; j++) {
        eachStack.push(lineNumber);
      }

      // Process closing tags
      for (let j = 0; j < endIfMatches.length; j++) {
        if (ifStack.length === 0) {
          errors.push({
            line: lineNumber,
            column: line.indexOf('{{/if}}') + 1,
            message: 'Unmatched {{/if}}',
            type: 'syntax',
          });
        } else {
          ifStack.pop();
        }
      }

      for (let j = 0; j < endEachMatches.length; j++) {
        if (eachStack.length === 0) {
          errors.push({
            line: lineNumber,
            column: line.indexOf('{{/each}}') + 1,
            message: 'Unmatched {{/each}}',
            type: 'syntax',
          });
        } else {
          eachStack.pop();
        }
      }
    }

    // Check for unclosed tags at the end
    for (const lineNum of ifStack) {
      errors.push({
        line: lineNum,
        column: 1,
        message: 'Unclosed {{#if}}',
        type: 'syntax',
      });
    }

    for (const lineNum of eachStack) {
      errors.push({
        line: lineNum,
        column: 1,
        message: 'Unclosed {{#each}}',
        type: 'syntax',
      });
    }

    return errors;
  }

  /**
   * Validate variables against context
   */
  private validateVariables(variables: string[], context: TemplateContext): TemplateError[] {
    const errors: TemplateError[] = [];

    for (const variable of variables) {
      const value = this.getNestedValue(context, variable);
      
      if (value === undefined) {
        errors.push({
          line: 0,
          column: 0,
          message: `Required variable not found: ${variable}`,
          type: 'variable',
        });
      }
    }

    return errors;
  }

  /**
   * Clean up generated content
   */
  private cleanupContent(content: string): string {
    // Remove empty lines
    let cleaned = content.replace(/^\s*[\r\n]/gm, '');
    
    // Remove trailing whitespace
    cleaned = cleaned.replace(/[ \t]+$/gm, '');
    
    // Ensure proper line endings
    cleaned = cleaned.replace(/\r\n/g, '\n');
    
    return cleaned;
  }

  /**
   * Generate cache key
   */
  private generateCacheKey(templateName: string, context: TemplateContext): string {
    const contextHash = JSON.stringify(context);
    return `${templateName}_${this.hashString(contextHash)}`;
  }

  /**
   * Simple string hash function
   */
  private hashString(str: string): string {
    let hash = 0;
    for (let i = 0; i < str.length; i++) {
      const char = str.charCodeAt(i);
      hash = ((hash << 5) - hash) + char;
      hash = hash & hash; // Convert to 32-bit integer
    }
    return hash.toString(36);
  }

  /**
   * Cache result
   */
  private cacheResult(key: string, result: TemplateResult): void {
         // Implement LRU cache
     if (this.cache.size >= this.maxCacheSize) {
       const firstKey = this.cache.keys().next().value;
       if (firstKey) {
         this.cache.delete(firstKey);
       }
     }
    
    this.cache.set(key, result);
  }

  /**
   * Clear cache
   */
  clearCache(): void {
    this.cache.clear();
    logger.info('Template cache cleared');
  }

  /**
   * Get cache statistics
   */
  getCacheStats(): {
    size: number;
    maxSize: number;
    hitRate: number;
  } {
    // This is a simplified implementation
    return {
      size: this.cache.size,
      maxSize: this.maxCacheSize,
      hitRate: 0.8, // Would need to track actual hits/misses
    };
  }

  /**
   * List all registered templates
   */
  listTemplates(): string[] {
    return Array.from(this.templates.keys());
  }

  /**
   * Remove template
   */
  removeTemplate(name: string): boolean {
    const removed = this.templates.delete(name);
    this.configs.delete(name);
    
    if (removed) {
      logger.info(`Removed template: ${name}`);
    }
    
    return removed;
  }

  /**
   * Get template configuration
   */
  getTemplateConfig(name: string): TemplateConfig | undefined {
    return this.configs.get(name);
  }

  /**
   * Set template configuration
   */
  setTemplateConfig(name: string, config: TemplateConfig): void {
    this.configs.set(name, config);
    logger.info(`Updated template config: ${name}`);
  }
}

// Export singleton instance
export const templateEngine = new TemplateEngine(); 