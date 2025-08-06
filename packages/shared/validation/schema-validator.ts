import Ajv from 'ajv';
import addFormats from 'ajv-formats';
import { projectMetadataSchema, ros2PackageSchema, environmentConfigSchema, simulationConfigSchema, projectConfigSchema } from '../index';

export interface ValidationError {
  keyword: string;
  dataPath: string;
  schemaPath: string;
  params: any;
  message: string;
  data?: any;
}

export interface ValidationResult {
  valid: boolean;
  errors: ValidationError[];
}

export class SchemaValidator {
  private ajv: Ajv;

  constructor() {
    this.ajv = new Ajv({
      allErrors: true,
      verbose: true
    });
    
    // Add format validators
    addFormats(this.ajv);
    
    // Add custom formats if needed
    this.ajv.addFormat('uuid', {
      type: 'string',
      validate: (str: string) => {
        const uuidRegex = /^[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i;
        return uuidRegex.test(str);
      }
    });

    // Add all schemas to the AJV instance
    this.ajv.addSchema(this.stripSchemaKeyword(projectMetadataSchema), 'project-metadata.schema.json');
    this.ajv.addSchema(this.stripSchemaKeyword(ros2PackageSchema), 'ros2-package.schema.json');
    this.ajv.addSchema(this.stripSchemaKeyword(environmentConfigSchema), 'environment-config.schema.json');
    this.ajv.addSchema(this.stripSchemaKeyword(simulationConfigSchema), 'simulation-config.schema.json');
    this.ajv.addSchema(this.stripSchemaKeyword(projectConfigSchema), 'project-config.schema.json');
  }

  private stripSchemaKeyword(schema: any): any {
    const { $schema, ...rest } = schema;
    return rest;
  }

  /**
   * Validate project metadata against schema
   */
  validateProjectMetadata(data: any): ValidationResult {
    return this.validate(data, projectMetadataSchema);
  }

  /**
   * Validate ROS2 package configuration against schema
   */
  validateRos2Package(data: any): ValidationResult {
    return this.validate(data, ros2PackageSchema);
  }

  /**
   * Validate environment configuration against schema
   */
  validateEnvironmentConfig(data: any): ValidationResult {
    return this.validate(data, environmentConfigSchema);
  }

  /**
   * Validate simulation configuration against schema
   */
  validateSimulationConfig(data: any): ValidationResult {
    return this.validate(data, simulationConfigSchema);
  }

  /**
   * Validate complete project configuration against schema
   */
  validateProjectConfig(data: any): ValidationResult {
    return this.validate(data, projectConfigSchema);
  }

  /**
   * Generic validation method
   */
  private validate(data: any, schema: any): ValidationResult {
    try {
      const validate = this.ajv.compile(this.stripSchemaKeyword(schema));
      const valid = validate(data);
      
      return {
        valid,
        errors: valid ? [] : (validate.errors || []).map((error: any) => ({
          keyword: error.keyword,
          dataPath: error.instancePath,
          schemaPath: error.schemaPath,
          params: error.params,
          message: error.message || 'Validation error',
          data: error.data
        }))
      };
    } catch (error) {
      return {
        valid: false,
        errors: [{
          keyword: 'exception',
          dataPath: '',
          schemaPath: '',
          params: {},
          message: error instanceof Error ? error.message : 'Unknown validation error',
          data
        }]
      };
    }
  }

  /**
   * Get human-readable error messages
   */
  getErrorMessages(result: ValidationResult): string[] {
    return result.errors.map(error => {
      const path = error.dataPath || 'root';
      return `${path}: ${error.message}`;
    });
  }

  /**
   * Validate and throw if invalid
   */
  validateOrThrow(data: any, schema: any, context?: string): void {
    const result = this.validate(data, schema);
    if (!result.valid) {
      const messages = this.getErrorMessages(result);
      const contextMsg = context ? ` in ${context}` : '';
      throw new Error(`Validation failed${contextMsg}:\n${messages.join('\n')}`);
    }
  }
}

// Export singleton instance
export const schemaValidator = new SchemaValidator(); 