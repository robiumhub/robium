import { schemaValidator, ValidationResult } from './schema-validator';

export interface ConfigValidationOptions {
  validateMetadata?: boolean;
  validateRos2Packages?: boolean;
  validateEnvironment?: boolean;
  validateSimulation?: boolean;
  validateComplete?: boolean;
}

export interface ConfigValidationResult {
  valid: boolean;
  errors: ValidationResult[];
  warnings: string[];
}

export class ConfigValidator {
  /**
   * Validate a complete project configuration
   */
  validateProjectConfiguration(config: any, options: ConfigValidationOptions = {}): ConfigValidationResult {
    const {
      validateMetadata = true,
      validateRos2Packages = true,
      validateEnvironment = true,
      validateSimulation = true,
      validateComplete = true
    } = options;

    const errors: ValidationResult[] = [];
    const warnings: string[] = [];

    // Validate metadata
    if (validateMetadata && config.metadata) {
      const result = schemaValidator.validateProjectMetadata(config.metadata);
      if (!result.valid) {
        errors.push(result);
      }
    }

    // Validate ROS2 packages
    if (validateRos2Packages && config.ros2Packages) {
      if (Array.isArray(config.ros2Packages)) {
        config.ros2Packages.forEach((pkg: any, index: number) => {
          const result = schemaValidator.validateRos2Package(pkg);
          if (!result.valid) {
            // Add package index to error context
            result.errors.forEach(error => {
              error.dataPath = `ros2Packages[${index}]${error.dataPath}`;
            });
            errors.push(result);
          }
        });
      } else {
        warnings.push('ros2Packages should be an array');
      }
    }

    // Validate environment configuration
    if (validateEnvironment && config.environment) {
      const result = schemaValidator.validateEnvironmentConfig(config.environment);
      if (!result.valid) {
        errors.push(result);
      }
    }

    // Validate simulation configuration
    if (validateSimulation && config.simulation) {
      const result = schemaValidator.validateSimulationConfig(config.simulation);
      if (!result.valid) {
        errors.push(result);
      }
    }

    // Validate complete configuration
    if (validateComplete) {
      const result = schemaValidator.validateProjectConfig(config);
      if (!result.valid) {
        errors.push(result);
      }
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings
    };
  }

  /**
   * Validate configuration with detailed error reporting
   */
  validateWithDetails(config: any, options: ConfigValidationOptions = {}): {
    valid: boolean;
    details: {
      metadata: ValidationResult;
      ros2Packages: ValidationResult[];
      environment: ValidationResult;
      simulation: ValidationResult;
      complete: ValidationResult;
    };
  } {
    const details = {
      metadata: schemaValidator.validateProjectMetadata(config.metadata || {}),
      ros2Packages: Array.isArray(config.ros2Packages) 
        ? config.ros2Packages.map((pkg: any) => schemaValidator.validateRos2Package(pkg))
        : [],
      environment: schemaValidator.validateEnvironmentConfig(config.environment || {}),
      simulation: schemaValidator.validateSimulationConfig(config.simulation || {}),
      complete: schemaValidator.validateProjectConfig(config)
    };

    const valid = details.metadata.valid &&
      details.ros2Packages.every((result: ValidationResult) => result.valid) &&
      details.environment.valid &&
      details.simulation.valid &&
      details.complete.valid;

    return { valid, details };
  }

  /**
   * Get all validation errors as a flat list
   */
  getAllErrors(result: ConfigValidationResult): string[] {
    const allErrors: string[] = [];
    
    result.errors.forEach(validationResult => {
      validationResult.errors.forEach(error => {
        const path = error.dataPath || 'root';
        allErrors.push(`${path}: ${error.message}`);
      });
    });

    return allErrors;
  }

  /**
   * Validate and throw if invalid
   */
  validateOrThrow(config: any, options: ConfigValidationOptions = {}, context?: string): void {
    const result = this.validateProjectConfiguration(config, options);
    if (!result.valid) {
      const errors = this.getAllErrors(result);
      const contextMsg = context ? ` in ${context}` : '';
      throw new Error(`Configuration validation failed${contextMsg}:\n${errors.join('\n')}`);
    }
  }
}

// Export singleton instance
export const configValidator = new ConfigValidator(); 