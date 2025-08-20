import { logger } from '../utils/logger';
import { environmentVariableService, EnvironmentConfig } from './EnvironmentVariableService';
import { ProjectConfiguration } from './DockerfileGenerationService';
import { ComposeConfiguration } from './DockerComposeGenerationService';
import fs from 'fs';
import path from 'path';

export interface ValidationResult {
  isValid: boolean;
  errors: ValidationError[];
  warnings: ValidationWarning[];
  suggestions: ValidationSuggestion[];
  metadata: ValidationMetadata;
}

export interface ValidationError {
  type: 'error';
  code: string;
  message: string;
  location?: string;
  line?: number;
  column?: number;
  severity: 'critical' | 'high' | 'medium' | 'low';
  fixable: boolean;
  fix?: string;
}

export interface ValidationWarning {
  type: 'warning';
  code: string;
  message: string;
  location?: string;
  line?: number;
  column?: number;
  severity: 'high' | 'medium' | 'low';
  fixable: boolean;
  fix?: string;
}

export interface ValidationSuggestion {
  type: 'suggestion';
  code: string;
  message: string;
  location?: string;
  impact: 'performance' | 'security' | 'maintainability' | 'best_practice';
  fixable: boolean;
  fix?: string;
}

export interface ValidationMetadata {
  totalChecks: number;
  passedChecks: number;
  failedChecks: number;
  validationTime: number;
  fileSize?: number;
  complexity?: number;
}

export interface DockerfileValidationContext {
  content: string;
  config: ProjectConfiguration;
  filePath?: string;
}

export interface ComposeValidationContext {
  content: string;
  config: ComposeConfiguration;
  filePath?: string;
}

export interface EnvironmentValidationContext {
  config: EnvironmentConfig;
  variables: Record<string, any>;
}

export class ValidationService {
  private validationRules: Map<string, ValidationRule> = new Map();
  private customValidators: Map<string, CustomValidator> = new Map();

  constructor() {
    this.initializeValidationRules();
  }

  /**
   * Initialize default validation rules
   */
  private initializeValidationRules(): void {
    // Dockerfile validation rules
    this.addValidationRule('dockerfile-required-instructions', {
      name: 'Required Dockerfile Instructions',
      description: 'Check for required Dockerfile instructions',
      validate: (context: DockerfileValidationContext): ValidationResult => {
        const errors: ValidationError[] = [];
        const warnings: ValidationWarning[] = [];
        const suggestions: ValidationSuggestion[] = [];

        const requiredInstructions = ['FROM'];
        const recommendedInstructions = ['WORKDIR', 'COPY', 'CMD'];

        for (const instruction of requiredInstructions) {
          if (!context.content.includes(instruction)) {
            errors.push({
              type: 'error',
              code: 'MISSING_REQUIRED_INSTRUCTION',
              message: `Missing required instruction: ${instruction}`,
              severity: 'critical',
              fixable: true,
              fix: `Add ${instruction} instruction to your Dockerfile`
            });
          }
        }

        for (const instruction of recommendedInstructions) {
          if (!context.content.includes(instruction)) {
            suggestions.push({
              type: 'suggestion',
              code: 'MISSING_RECOMMENDED_INSTRUCTION',
              message: `Consider adding instruction: ${instruction}`,
              impact: 'best_practice',
              fixable: true,
              fix: `Add ${instruction} instruction for better Dockerfile structure`
            });
          }
        }

        return {
          isValid: errors.length === 0,
          errors,
          warnings,
          suggestions,
          metadata: {
            totalChecks: requiredInstructions.length + recommendedInstructions.length,
            passedChecks: requiredInstructions.length + recommendedInstructions.length - errors.length - suggestions.length,
            failedChecks: errors.length + suggestions.length,
            validationTime: 0
          }
        };
      }
    });

    // Security validation rules
    this.addValidationRule('dockerfile-security', {
      name: 'Dockerfile Security',
      description: 'Check for security issues in Dockerfile',
      validate: (context: DockerfileValidationContext): ValidationResult => {
        const errors: ValidationError[] = [];
        const warnings: ValidationWarning[] = [];
        const suggestions: ValidationSuggestion[] = [];

        // Check for root user
        if (context.content.includes('USER root') || !context.content.includes('USER ')) {
          warnings.push({
            type: 'warning',
            code: 'ROOT_USER_DETECTED',
            message: 'Container runs as root user, consider using a non-root user',
            severity: 'high',
            fixable: true,
            fix: 'Add USER instruction to run as non-root user'
          });
        }

        // Check for sensitive data in layers
        const sensitivePatterns = [
          /password\s*=/i,
          /secret\s*=/i,
          /key\s*=/i,
          /token\s*=/i
        ];

        for (const pattern of sensitivePatterns) {
          if (pattern.test(context.content)) {
            warnings.push({
              type: 'warning',
              code: 'SENSITIVE_DATA_IN_LAYER',
              message: 'Sensitive data detected in Dockerfile layer',
              severity: 'high',
              fixable: true,
              fix: 'Use build args or environment variables for sensitive data'
            });
          }
        }

        return {
          isValid: errors.length === 0,
          errors,
          warnings,
          suggestions,
          metadata: {
            totalChecks: 1 + sensitivePatterns.length,
            passedChecks: 1 + sensitivePatterns.length - warnings.length,
            failedChecks: warnings.length,
            validationTime: 0
          }
        };
      }
    });

    // Performance validation rules
    this.addValidationRule('dockerfile-performance', {
      name: 'Dockerfile Performance',
      description: 'Check for performance optimizations in Dockerfile',
      validate: (context: DockerfileValidationContext): ValidationResult => {
        const errors: ValidationError[] = [];
        const warnings: ValidationWarning[] = [];
        const suggestions: ValidationSuggestion[] = [];

        // Check for proper layer ordering
        const lines = context.content.split('\n');
        let copyIndex = -1;
        let runIndex = -1;

        for (let i = 0; i < lines.length; i++) {
          if (lines[i].trim().startsWith('COPY')) {
            copyIndex = i;
          }
          if (lines[i].trim().startsWith('RUN')) {
            runIndex = i;
          }
        }

        if (copyIndex > runIndex && copyIndex !== -1 && runIndex !== -1) {
          suggestions.push({
            type: 'suggestion',
            code: 'IMPROPER_LAYER_ORDERING',
            message: 'Consider copying files after installing dependencies for better caching',
            impact: 'performance',
            fixable: true,
            fix: 'Move COPY instructions after RUN instructions for better layer caching'
          });
        }

        // Check for multi-stage builds
        if (!context.content.includes('FROM') || context.content.split('FROM').length === 2) {
          suggestions.push({
            type: 'suggestion',
            code: 'SINGLE_STAGE_BUILD',
            message: 'Consider using multi-stage builds to reduce final image size',
            impact: 'performance',
            fixable: true,
            fix: 'Use multi-stage builds with separate builder and runtime stages'
          });
        }

        return {
          isValid: errors.length === 0,
          errors,
          warnings,
          suggestions,
          metadata: {
            totalChecks: 2,
            passedChecks: 2 - suggestions.length,
            failedChecks: suggestions.length,
            validationTime: 0
          }
        };
      }
    });

    // Docker Compose validation rules
    this.addValidationRule('compose-structure', {
      name: 'Docker Compose Structure',
      description: 'Validate docker-compose.yml structure and syntax',
      validate: (context: ComposeValidationContext): ValidationResult => {
        const errors: ValidationError[] = [];
        const warnings: ValidationWarning[] = [];
        const suggestions: ValidationSuggestion[] = [];

        // Check for required fields
        if (!context.config.services || context.config.services.length === 0) {
          errors.push({
            type: 'error',
            code: 'NO_SERVICES_DEFINED',
            message: 'No services defined in docker-compose.yml',
            severity: 'critical',
            fixable: true,
            fix: 'Add at least one service to your docker-compose.yml'
          });
        }

        // Check for service names
        for (const service of context.config.services) {
          if (!service.name || service.name.trim() === '') {
            errors.push({
              type: 'error',
              code: 'INVALID_SERVICE_NAME',
              message: 'Service name is required',
              severity: 'critical',
              fixable: true,
              fix: 'Provide a valid service name'
            });
          }
        }

        // Check for port conflicts
        const usedPorts = new Set<string>();
        for (const service of context.config.services) {
          if (service.ports) {
            for (const port of service.ports) {
              if (usedPorts.has(port)) {
                warnings.push({
                  type: 'warning',
                  code: 'PORT_CONFLICT',
                  message: `Port conflict detected: ${port}`,
                  severity: 'medium',
                  fixable: true,
                  fix: 'Use different ports for each service'
                });
              }
              usedPorts.add(port);
            }
          }
        }

        return {
          isValid: errors.length === 0,
          errors,
          warnings,
          suggestions,
          metadata: {
            totalChecks: 1 + context.config.services.length + usedPorts.size,
            passedChecks: 1 + context.config.services.length + usedPorts.size - errors.length - warnings.length,
            failedChecks: errors.length + warnings.length,
            validationTime: 0
          }
        };
      }
    });

    // Environment validation rules
    this.addValidationRule('environment-consistency', {
      name: 'Environment Consistency',
      description: 'Validate environment variable consistency across services',
      validate: (context: EnvironmentValidationContext): ValidationResult => {
        const errors: ValidationError[] = [];
        const warnings: ValidationWarning[] = [];
        const suggestions: ValidationSuggestion[] = [];

        // Check for required variables
        for (const [name, variable] of Object.entries(context.config.variables)) {
          if (variable.required && !variable.value) {
            errors.push({
              type: 'error',
              code: 'MISSING_REQUIRED_VARIABLE',
              message: `Required environment variable missing: ${name}`,
              severity: 'critical',
              fixable: true,
              fix: `Set value for required variable: ${name}`
            });
          }
        }

        // Check for sensitive variables without encryption
        for (const [name, variable] of Object.entries(context.config.variables)) {
          if (variable.sensitive && !variable.encrypted) {
            warnings.push({
              type: 'warning',
              code: 'SENSITIVE_VARIABLE_NOT_ENCRYPTED',
              message: `Sensitive variable not encrypted: ${name}`,
              severity: 'high',
              fixable: true,
              fix: `Enable encryption for sensitive variable: ${name}`
            });
          }
        }

        return {
          isValid: errors.length === 0,
          errors,
          warnings,
          suggestions,
          metadata: {
            totalChecks: Object.keys(context.config.variables).length * 2,
            passedChecks: Object.keys(context.config.variables).length * 2 - errors.length - warnings.length,
            failedChecks: errors.length + warnings.length,
            validationTime: 0
          }
        };
      }
    });

    logger.info('Validation service initialized with default rules');
  }

  /**
   * Add a custom validation rule
   */
  addValidationRule(ruleId: string, rule: ValidationRule): void {
    this.validationRules.set(ruleId, rule);
    logger.info(`Added validation rule: ${ruleId}`);
  }

  /**
   * Add a custom validator
   */
  addCustomValidator(validatorId: string, validator: CustomValidator): void {
    this.customValidators.set(validatorId, validator);
    logger.info(`Added custom validator: ${validatorId}`);
  }

  /**
   * Validate Dockerfile
   */
  validateDockerfile(context: DockerfileValidationContext): ValidationResult {
    const startTime = Date.now();
    const allErrors: ValidationError[] = [];
    const allWarnings: ValidationWarning[] = [];
    const allSuggestions: ValidationSuggestion[] = [];
    let totalChecks = 0;
    let passedChecks = 0;

    // Run all Dockerfile-specific validation rules
    const dockerfileRules = ['dockerfile-required-instructions', 'dockerfile-security', 'dockerfile-performance'];
    
    for (const ruleId of dockerfileRules) {
      const rule = this.validationRules.get(ruleId);
      if (rule) {
        const result = rule.validate(context);
        allErrors.push(...result.errors);
        allWarnings.push(...result.warnings);
        allSuggestions.push(...result.suggestions);
        totalChecks += result.metadata.totalChecks;
        passedChecks += result.metadata.passedChecks;
      }
    }

    // Run custom validators
    for (const [validatorId, validator] of this.customValidators.entries()) {
      if (validator.type === 'dockerfile') {
        const result = validator.validate(context);
        allErrors.push(...result.errors);
        allWarnings.push(...result.warnings);
        allSuggestions.push(...result.suggestions);
        totalChecks += result.metadata.totalChecks;
        passedChecks += result.metadata.passedChecks;
      }
    }

    const validationTime = Date.now() - startTime;

    return {
      isValid: allErrors.length === 0,
      errors: allErrors,
      warnings: allWarnings,
      suggestions: allSuggestions,
      metadata: {
        totalChecks,
        passedChecks,
        failedChecks: allErrors.length + allWarnings.length + allSuggestions.length,
        validationTime,
        fileSize: context.content.length,
        complexity: this.calculateComplexity(context.content)
      }
    };
  }

  /**
   * Validate Docker Compose file
   */
  validateCompose(context: ComposeValidationContext): ValidationResult {
    const startTime = Date.now();
    const allErrors: ValidationError[] = [];
    const allWarnings: ValidationWarning[] = [];
    const allSuggestions: ValidationSuggestion[] = [];
    let totalChecks = 0;
    let passedChecks = 0;

    // Run compose-specific validation rules
    const composeRules = ['compose-structure'];
    
    for (const ruleId of composeRules) {
      const rule = this.validationRules.get(ruleId);
      if (rule) {
        const result = rule.validate(context);
        allErrors.push(...result.errors);
        allWarnings.push(...result.warnings);
        allSuggestions.push(...result.suggestions);
        totalChecks += result.metadata.totalChecks;
        passedChecks += result.metadata.passedChecks;
      }
    }

    // Run custom validators
    for (const [validatorId, validator] of this.customValidators.entries()) {
      if (validator.type === 'compose') {
        const result = validator.validate(context);
        allErrors.push(...result.errors);
        allWarnings.push(...result.warnings);
        allSuggestions.push(...result.suggestions);
        totalChecks += result.metadata.totalChecks;
        passedChecks += result.metadata.passedChecks;
      }
    }

    const validationTime = Date.now() - startTime;

    return {
      isValid: allErrors.length === 0,
      errors: allErrors,
      warnings: allWarnings,
      suggestions: allSuggestions,
      metadata: {
        totalChecks,
        passedChecks,
        failedChecks: allErrors.length + allWarnings.length + allSuggestions.length,
        validationTime,
        fileSize: context.content.length,
        complexity: this.calculateComplexity(context.content)
      }
    };
  }

  /**
   * Validate environment configuration
   */
  validateEnvironment(context: EnvironmentValidationContext): ValidationResult {
    const startTime = Date.now();
    const allErrors: ValidationError[] = [];
    const allWarnings: ValidationWarning[] = [];
    const allSuggestions: ValidationSuggestion[] = [];
    let totalChecks = 0;
    let passedChecks = 0;

    // Run environment-specific validation rules
    const environmentRules = ['environment-consistency'];
    
    for (const ruleId of environmentRules) {
      const rule = this.validationRules.get(ruleId);
      if (rule) {
        const result = rule.validate(context);
        allErrors.push(...result.errors);
        allWarnings.push(...result.warnings);
        allSuggestions.push(...result.suggestions);
        totalChecks += result.metadata.totalChecks;
        passedChecks += result.metadata.passedChecks;
      }
    }

    // Run custom validators
    for (const [validatorId, validator] of this.customValidators.entries()) {
      if (validator.type === 'environment') {
        const result = validator.validate(context);
        allErrors.push(...result.errors);
        allWarnings.push(...result.warnings);
        allSuggestions.push(...result.suggestions);
        totalChecks += result.metadata.totalChecks;
        passedChecks += result.metadata.passedChecks;
      }
    }

    const validationTime = Date.now() - startTime;

    return {
      isValid: allErrors.length === 0,
      errors: allErrors,
      warnings: allWarnings,
      suggestions: allSuggestions,
      metadata: {
        totalChecks,
        passedChecks,
        failedChecks: allErrors.length + allWarnings.length + allSuggestions.length,
        validationTime
      }
    };
  }

  /**
   * Validate complete project configuration
   */
  validateProject(
    dockerfileContext: DockerfileValidationContext,
    composeContext: ComposeValidationContext,
    environmentContext: EnvironmentValidationContext
  ): ValidationResult {
    const startTime = Date.now();
    
    // Run individual validations
    const dockerfileResult = this.validateDockerfile(dockerfileContext);
    const composeResult = this.validateCompose(composeContext);
    const environmentResult = this.validateEnvironment(environmentContext);

    // Combine results
    const allErrors = [
      ...dockerfileResult.errors,
      ...composeResult.errors,
      ...environmentResult.errors
    ];

    const allWarnings = [
      ...dockerfileResult.warnings,
      ...composeResult.warnings,
      ...environmentResult.warnings
    ];

    const allSuggestions = [
      ...dockerfileResult.suggestions,
      ...composeResult.suggestions,
      ...environmentResult.suggestions
    ];

    // Cross-validation checks
    const crossValidationResult = this.performCrossValidation(
      dockerfileContext,
      composeContext,
      environmentContext
    );

    allErrors.push(...crossValidationResult.errors);
    allWarnings.push(...crossValidationResult.warnings);
    allSuggestions.push(...crossValidationResult.suggestions);

    const validationTime = Date.now() - startTime;

    return {
      isValid: allErrors.length === 0,
      errors: allErrors,
      warnings: allWarnings,
      suggestions: allSuggestions,
      metadata: {
        totalChecks: dockerfileResult.metadata.totalChecks + 
                    composeResult.metadata.totalChecks + 
                    environmentResult.metadata.totalChecks + 
                    crossValidationResult.metadata.totalChecks,
        passedChecks: dockerfileResult.metadata.passedChecks + 
                     composeResult.metadata.passedChecks + 
                     environmentResult.metadata.passedChecks + 
                     crossValidationResult.metadata.passedChecks,
        failedChecks: allErrors.length + allWarnings.length + allSuggestions.length,
        validationTime
      }
    };
  }

  /**
   * Perform cross-validation between different configuration types
   */
  private performCrossValidation(
    dockerfileContext: DockerfileValidationContext,
    composeContext: ComposeValidationContext,
    environmentContext: EnvironmentValidationContext
  ): ValidationResult {
    const errors: ValidationError[] = [];
    const warnings: ValidationWarning[] = [];
    const suggestions: ValidationSuggestion[] = [];

    // Check for environment variable consistency
    const dockerfileEnvVars = this.extractEnvironmentVariables(dockerfileContext.content);
    const composeEnvVars = this.extractEnvironmentVariables(composeContext.content);
    const configEnvVars = Object.keys(environmentContext.config.variables);

    // Check if environment variables are consistent across all configurations
    const allEnvVars = new Set([...dockerfileEnvVars, ...composeEnvVars, ...configEnvVars]);
    
    for (const envVar of allEnvVars) {
      const inDockerfile = dockerfileEnvVars.includes(envVar);
      const inCompose = composeEnvVars.includes(envVar);
      const inConfig = configEnvVars.includes(envVar);

      if (!inConfig && (inDockerfile || inCompose)) {
        warnings.push({
          type: 'warning',
          code: 'ENV_VAR_NOT_IN_CONFIG',
          message: `Environment variable ${envVar} used but not defined in configuration`,
          severity: 'medium',
          fixable: true,
          fix: `Add ${envVar} to environment configuration`
        });
      }
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      suggestions,
      metadata: {
        totalChecks: allEnvVars.size,
        passedChecks: allEnvVars.size - warnings.length,
        failedChecks: warnings.length,
        validationTime: 0
      }
    };
  }

  /**
   * Extract environment variables from content
   */
  private extractEnvironmentVariables(content: string): string[] {
    const envVars: string[] = [];
    const lines = content.split('\n');
    
    for (const line of lines) {
      // Match ENV instructions in Dockerfile
      const envMatch = line.match(/^ENV\s+(\w+)=/);
      if (envMatch) {
        envVars.push(envMatch[1]);
      }
      
      // Match environment variables in docker-compose
      const composeEnvMatch = line.match(/^\s*(\w+):\s*\$\{?\w+\}?/);
      if (composeEnvMatch) {
        envVars.push(composeEnvMatch[1]);
      }
    }
    
    return envVars;
  }

  /**
   * Calculate complexity score for content
   */
  private calculateComplexity(content: string): number {
    const lines = content.split('\n');
    let complexity = 0;
    
    // Base complexity
    complexity += lines.length;
    
    // Add complexity for different instruction types
    const instructionWeights = {
      'FROM': 1,
      'RUN': 3,
      'COPY': 2,
      'ADD': 2,
      'ENV': 1,
      'EXPOSE': 1,
      'VOLUME': 1,
      'USER': 1,
      'WORKDIR': 1,
      'ARG': 1,
      'ONBUILD': 2,
      'STOPSIGNAL': 1,
      'HEALTHCHECK': 3,
      'SHELL': 1,
      'ENTRYPOINT': 2,
      'CMD': 2
    };
    
    for (const line of lines) {
      const trimmed = line.trim();
      for (const [instruction, weight] of Object.entries(instructionWeights)) {
        if (trimmed.startsWith(instruction)) {
          complexity += weight;
          break;
        }
      }
    }
    
    return complexity;
  }

  /**
   * Get validation statistics
   */
  getValidationStats(): {
    totalRules: number;
    totalValidators: number;
    ruleTypes: string[];
  } {
    const ruleTypes = new Set<string>();
    
    for (const rule of this.validationRules.values()) {
      ruleTypes.add(rule.name);
    }
    
    return {
      totalRules: this.validationRules.size,
      totalValidators: this.customValidators.size,
      ruleTypes: Array.from(ruleTypes)
    };
  }
}

// Interfaces for validation rules and custom validators
export interface ValidationRule {
  name: string;
  description: string;
  validate: (context: any) => ValidationResult;
}

export interface CustomValidator {
  type: 'dockerfile' | 'compose' | 'environment';
  validate: (context: any) => ValidationResult;
}

// Export singleton instance
export const validationService = new ValidationService(); 