import { logger } from '../utils/logger';
import fs from 'fs';
import path from 'path';
import { promises as fsPromises } from 'fs';
import crypto from 'crypto';

export interface EnvironmentVariable {
  name: string;
  value: string | number | boolean | object;
  type: 'string' | 'number' | 'boolean' | 'json';
  description?: string;
  required?: boolean;
  sensitive?: boolean;
  validation?: {
    pattern?: string;
    minLength?: number;
    maxLength?: number;
    min?: number;
    max?: number;
    options?: (string | number | boolean)[];
  };
  environment?: 'development' | 'production' | 'staging' | 'all';
  defaultValue?: string | number | boolean | object;
  encrypted?: boolean;
  source?: 'user' | 'system' | 'secret' | 'inherited';
  inheritedFrom?: string;
  lastModified?: string;
}

export interface EnvironmentConfig {
  id: string;
  projectId: string;
  userId: string;
  name: string;
  description?: string;
  environment: 'development' | 'production' | 'staging';
  variables: Record<string, EnvironmentVariable>;
  secrets: Record<string, EnvironmentVariable>;
  inheritedConfigs?: string[];
  createdAt: string;
  updatedAt: string;
  version: string;
}

export interface EnvironmentValidation {
  isValid: boolean;
  errors: string[];
  warnings: string[];
  missingRequired: string[];
  invalidValues: string[];
  securityIssues: string[];
}

export interface EnvironmentInjection {
  variables: Record<string, string>;
  secrets: Record<string, string>;
  envFiles: string[];
  dockerSecrets: string[];
  validation: EnvironmentValidation;
}

export interface SecretInfo {
  id: string;
  name: string;
  encrypted: boolean;
  lastRotated?: string;
  expiresAt?: string;
  accessCount: number;
}

export class EnvironmentVariableService {
  private configs: Map<string, EnvironmentConfig> = new Map();
  private secrets: Map<string, string> = new Map();
  private secretMetadata: Map<string, SecretInfo> = new Map();
  private encryptionKey: string;
  private configDir: string;
  private secretsDir: string;

  constructor() {
    this.encryptionKey = process.env.ENV_ENCRYPTION_KEY || this.generateEncryptionKey();
    this.configDir = path.join(process.cwd(), 'generated', 'environments');
    this.secretsDir = path.join(process.cwd(), 'generated', 'secrets');
    this.initializeDirectories();
  }

  /**
   * Initialize required directories
   */
  private async initializeDirectories(): Promise<void> {
    try {
      await fsPromises.mkdir(this.configDir, { recursive: true });
      await fsPromises.mkdir(this.secretsDir, { recursive: true });
      logger.info('Environment variable service directories initialized');
    } catch (error) {
      logger.error('Failed to initialize environment directories:', error as Record<string, unknown>);
    }
  }

  /**
   * Generate encryption key for sensitive data
   */
  private generateEncryptionKey(): string {
    return crypto.randomBytes(32).toString('hex');
  }

  /**
   * Create environment configuration
   */
  async createEnvironmentConfig(
    projectId: string,
    userId: string,
    name: string,
    environment: 'development' | 'production' | 'staging',
    variables?: Record<string, EnvironmentVariable>,
    secrets?: Record<string, EnvironmentVariable>
  ): Promise<EnvironmentConfig> {
    const configId = `env_${userId}_${projectId}_${environment}_${Date.now()}`;
    
    const config: EnvironmentConfig = {
      id: configId,
      projectId,
      userId,
      name,
      environment,
      variables: variables || {},
      secrets: secrets || {},
      createdAt: new Date().toISOString(),
      updatedAt: new Date().toISOString(),
      version: '1.0.0'
    };

    // Validate configuration
    const validation = this.validateEnvironmentConfig(config);
    if (!validation.isValid) {
      throw new Error(`Invalid environment configuration: ${validation.errors.join(', ')}`);
    }

    this.configs.set(configId, config);
    await this.saveEnvironmentConfig(config);
    
    logger.info(`Environment configuration created: ${configId}`);
    return config;
  }

  /**
   * Get environment configuration
   */
  getEnvironmentConfig(configId: string): EnvironmentConfig | undefined {
    return this.configs.get(configId);
  }

  /**
   * Get environment configurations for a project
   */
  getProjectEnvironmentConfigs(projectId: string, userId?: string): EnvironmentConfig[] {
    return Array.from(this.configs.values()).filter(config => {
      if (config.projectId !== projectId) return false;
      if (userId && config.userId !== userId) return false;
      return true;
    });
  }

  /**
   * Update environment configuration
   */
  async updateEnvironmentConfig(
    configId: string,
    updates: Partial<EnvironmentConfig>
  ): Promise<EnvironmentConfig> {
    const config = this.configs.get(configId);
    if (!config) {
      throw new Error(`Environment configuration not found: ${configId}`);
    }

    const updatedConfig: EnvironmentConfig = {
      ...config,
      ...updates,
      updatedAt: new Date().toISOString()
    };

    // Validate updated configuration
    const validation = this.validateEnvironmentConfig(updatedConfig);
    if (!validation.isValid) {
      throw new Error(`Invalid environment configuration: ${validation.errors.join(', ')}`);
    }

    this.configs.set(configId, updatedConfig);
    await this.saveEnvironmentConfig(updatedConfig);
    
    logger.info(`Environment configuration updated: ${configId}`);
    return updatedConfig;
  }

  /**
   * Add or update environment variable
   */
  async setEnvironmentVariable(
    configId: string,
    name: string,
    variable: EnvironmentVariable
  ): Promise<void> {
    const config = this.configs.get(configId);
    if (!config) {
      throw new Error(`Environment configuration not found: ${configId}`);
    }

    // Validate variable
    const validation = this.validateEnvironmentVariable(name, variable);
    if (!validation.isValid) {
      throw new Error(`Invalid environment variable: ${validation.errors.join(', ')}`);
    }

    // Handle sensitive variables
    if (variable.sensitive) {
      await this.encryptSensitiveVariable(configId, name, variable);
    }

    config.variables[name] = {
      ...variable,
      lastModified: new Date().toISOString()
    };

    config.updatedAt = new Date().toISOString();
    this.configs.set(configId, config);
    await this.saveEnvironmentConfig(config);
    
    logger.info(`Environment variable set: ${configId}.${name}`);
  }

  /**
   * Get environment variable
   */
  getEnvironmentVariable(configId: string, name: string): EnvironmentVariable | undefined {
    const config = this.configs.get(configId);
    if (!config) return undefined;

    const variable = config.variables[name];
    if (!variable) return undefined;

    // Decrypt if necessary
    if (variable.sensitive && variable.encrypted) {
      return this.decryptSensitiveVariable(configId, name, variable);
    }

    return variable;
  }

  /**
   * Remove environment variable
   */
  async removeEnvironmentVariable(configId: string, name: string): Promise<void> {
    const config = this.configs.get(configId);
    if (!config) {
      throw new Error(`Environment configuration not found: ${configId}`);
    }

    if (config.variables[name]?.sensitive) {
      await this.removeSensitiveVariable(configId, name);
    }

    delete config.variables[name];
    config.updatedAt = new Date().toISOString();
    this.configs.set(configId, config);
    await this.saveEnvironmentConfig(config);
    
    logger.info(`Environment variable removed: ${configId}.${name}`);
  }

  /**
   * Create secret
   */
  async createSecret(
    configId: string,
    name: string,
    value: string,
    metadata?: Partial<SecretInfo>
  ): Promise<void> {
    const config = this.configs.get(configId);
    if (!config) {
      throw new Error(`Environment configuration not found: ${configId}`);
    }

    const secretId = `secret_${configId}_${name}`;
    const encryptedValue = this.encryptValue(value);
    
    this.secrets.set(secretId, encryptedValue);
    
    const secretInfo: SecretInfo = {
      id: secretId,
      name,
      encrypted: true,
      accessCount: 0,
      ...metadata
    };
    
    this.secretMetadata.set(secretId, secretInfo);
    await this.saveSecret(secretId, encryptedValue, secretInfo);
    
    logger.info(`Secret created: ${secretId}`);
  }

  /**
   * Get secret value
   */
  getSecret(configId: string, name: string): string | undefined {
    const secretId = `secret_${configId}_${name}`;
    const encryptedValue = this.secrets.get(secretId);
    if (!encryptedValue) return undefined;

    const metadata = this.secretMetadata.get(secretId);
    if (metadata) {
      metadata.accessCount++;
      metadata.lastRotated = new Date().toISOString();
    }

    return this.decryptValue(encryptedValue);
  }

  /**
   * Validate environment configuration
   */
  validateEnvironmentConfig(config: EnvironmentConfig): EnvironmentValidation {
    const errors: string[] = [];
    const warnings: string[] = [];
    const missingRequired: string[] = [];
    const invalidValues: string[] = [];
    const securityIssues: string[] = [];

    // Validate required fields
    if (!config.projectId) errors.push('Project ID is required');
    if (!config.userId) errors.push('User ID is required');
    if (!config.name) errors.push('Name is required');
    if (!config.environment) errors.push('Environment is required');

    // Validate variables
    for (const [name, variable] of Object.entries(config.variables)) {
      const validation = this.validateEnvironmentVariable(name, variable);
      if (!validation.isValid) {
        errors.push(...validation.errors);
      }
      if (validation.warnings.length > 0) {
        warnings.push(...validation.warnings);
      }
      if (variable.required && !variable.value) {
        missingRequired.push(name);
      }
    }

    // Check for security issues
    for (const [name, variable] of Object.entries(config.variables)) {
      if (variable.sensitive && !variable.encrypted) {
        securityIssues.push(`Sensitive variable ${name} is not encrypted`);
      }
      if (variable.value && typeof variable.value === 'string') {
        if (variable.value.includes('password') || variable.value.includes('secret')) {
          warnings.push(`Variable ${name} may contain sensitive data`);
        }
      }
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      missingRequired,
      invalidValues,
      securityIssues
    };
  }

  /**
   * Validate environment variable
   */
  validateEnvironmentVariable(name: string, variable: EnvironmentVariable): EnvironmentValidation {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Validate name
    if (!name || name.length === 0) {
      errors.push('Variable name is required');
    } else if (!/^[A-Z_][A-Z0-9_]*$/.test(name)) {
      errors.push('Variable name must be uppercase letters, numbers, and underscores only');
    }

    // Validate type
    if (!variable.type) {
      errors.push('Variable type is required');
    } else if (!['string', 'number', 'boolean', 'json'].includes(variable.type)) {
      errors.push('Invalid variable type');
    }

    // Validate value based on type
    if (variable.value !== undefined) {
      switch (variable.type) {
        case 'string':
          if (typeof variable.value !== 'string') {
            errors.push(`Variable ${name} must be a string`);
          } else if (variable.validation?.minLength && variable.value.length < variable.validation.minLength) {
            errors.push(`Variable ${name} is too short (min: ${variable.validation.minLength})`);
          } else if (variable.validation?.maxLength && variable.value.length > variable.validation.maxLength) {
            errors.push(`Variable ${name} is too long (max: ${variable.validation.maxLength})`);
          }
          break;
        case 'number':
          if (typeof variable.value !== 'number') {
            errors.push(`Variable ${name} must be a number`);
          } else if (variable.validation?.min !== undefined && variable.value < variable.validation.min) {
            errors.push(`Variable ${name} is too small (min: ${variable.validation.min})`);
          } else if (variable.validation?.max !== undefined && variable.value > variable.validation.max) {
            errors.push(`Variable ${name} is too large (max: ${variable.validation.max})`);
          }
          break;
        case 'boolean':
          if (typeof variable.value !== 'boolean') {
            errors.push(`Variable ${name} must be a boolean`);
          }
          break;
        case 'json':
          try {
            JSON.stringify(variable.value);
          } catch {
            errors.push(`Variable ${name} must be valid JSON`);
          }
          break;
      }

      // Validate pattern if specified
      if (variable.validation?.pattern && typeof variable.value === 'string') {
        const regex = new RegExp(variable.validation.pattern);
        if (!regex.test(variable.value)) {
          errors.push(`Variable ${name} does not match required pattern`);
        }
      }

      // Validate options if specified
      if (variable.validation?.options && !variable.validation.options.includes(variable.value as string | number | boolean)) {
        errors.push(`Variable ${name} must be one of: ${variable.validation.options.join(', ')}`);
      }
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      missingRequired: [],
      invalidValues: [],
      securityIssues: []
    };
  }

  /**
   * Generate environment injection for containers
   */
  generateEnvironmentInjection(configId: string, environment?: string): EnvironmentInjection {
    const config = this.configs.get(configId);
    if (!config) {
      throw new Error(`Environment configuration not found: ${configId}`);
    }

    const variables: Record<string, string> = {};
    const secrets: Record<string, string> = {};
    const envFiles: string[] = [];
    const dockerSecrets: string[] = [];

    // Process variables
    for (const [name, variable] of Object.entries(config.variables)) {
      // Filter by environment if specified
      if (environment && variable.environment && variable.environment !== 'all' && variable.environment !== environment) {
        continue;
      }

      let value: string;
      if (variable.sensitive && variable.encrypted) {
        const decrypted = this.decryptSensitiveVariable(configId, name, variable);
        value = String(decrypted.value);
        secrets[name] = value;
      } else {
        value = String(variable.value);
        variables[name] = value;
      }
    }

    // Generate .env file
    const envFilePath = path.join(this.configDir, `${configId}.env`);
    const envContent = Object.entries(variables)
      .map(([name, value]) => `${name}=${value}`)
      .join('\n');
    
    fs.writeFileSync(envFilePath, envContent);
    envFiles.push(envFilePath);

    // Generate Docker secrets
    for (const [name, value] of Object.entries(secrets)) {
      const secretPath = path.join(this.secretsDir, `${configId}_${name}`);
      fs.writeFileSync(secretPath, value);
      dockerSecrets.push(secretPath);
    }

    const validation = this.validateEnvironmentConfig(config);

    return {
      variables,
      secrets,
      envFiles,
      dockerSecrets,
      validation
    };
  }

  /**
   * Encrypt sensitive variable
   */
  private async encryptSensitiveVariable(
    configId: string,
    name: string,
    variable: EnvironmentVariable
  ): Promise<void> {
    if (!variable.sensitive) return;

    const encryptedValue = this.encryptValue(String(variable.value));
    variable.value = encryptedValue;
    variable.encrypted = true;
  }

  /**
   * Decrypt sensitive variable
   */
  private decryptSensitiveVariable(
    configId: string,
    name: string,
    variable: EnvironmentVariable
  ): EnvironmentVariable {
    if (!variable.sensitive || !variable.encrypted) return variable;

    const decryptedValue = this.decryptValue(String(variable.value));
    return {
      ...variable,
      value: decryptedValue,
      encrypted: false
    };
  }

  /**
   * Remove sensitive variable
   */
  private async removeSensitiveVariable(configId: string, name: string): Promise<void> {
    const secretId = `secret_${configId}_${name}`;
    this.secrets.delete(secretId);
    this.secretMetadata.delete(secretId);
    
    const secretPath = path.join(this.secretsDir, `${configId}_${name}`);
    try {
      await fsPromises.unlink(secretPath);
    } catch (error) {
      // File might not exist
    }
  }

  /**
   * Encrypt value
   */
  private encryptValue(value: string): string {
    const iv = crypto.randomBytes(16);
    const key = crypto.scryptSync(this.encryptionKey, 'salt', 32);
    const cipher = crypto.createCipheriv('aes-256-cbc', key, iv);
    let encrypted = cipher.update(value, 'utf8', 'hex');
    encrypted += cipher.final('hex');
    return `${iv.toString('hex')}:${encrypted}`;
  }

  /**
   * Decrypt value
   */
  private decryptValue(encryptedValue: string): string {
    const [ivHex, encrypted] = encryptedValue.split(':');
    const iv = Buffer.from(ivHex, 'hex');
    const key = crypto.scryptSync(this.encryptionKey, 'salt', 32);
    const decipher = crypto.createDecipheriv('aes-256-cbc', key, iv);
    let decrypted = decipher.update(encrypted, 'hex', 'utf8');
    decrypted += decipher.final('utf8');
    return decrypted;
  }

  /**
   * Save environment configuration to file
   */
  private async saveEnvironmentConfig(config: EnvironmentConfig): Promise<void> {
    const filePath = path.join(this.configDir, `${config.id}.json`);
    await fsPromises.writeFile(filePath, JSON.stringify(config, null, 2));
  }

  /**
   * Save secret to file
   */
  private async saveSecret(secretId: string, encryptedValue: string, metadata: SecretInfo): Promise<void> {
    const secretPath = path.join(this.secretsDir, secretId);
    await fsPromises.writeFile(secretPath, encryptedValue);
    
    const metadataPath = path.join(this.secretsDir, `${secretId}.meta.json`);
    await fsPromises.writeFile(metadataPath, JSON.stringify(metadata, null, 2));
  }

  /**
   * Get service statistics
   */
  getServiceStats(): {
    totalConfigs: number;
    totalVariables: number;
    totalSecrets: number;
    encryptedVariables: number;
    validationErrors: number;
  } {
    let totalVariables = 0;
    let totalSecrets = 0;
    let encryptedVariables = 0;
    let validationErrors = 0;

    for (const config of this.configs.values()) {
      totalVariables += Object.keys(config.variables).length;
      totalSecrets += Object.keys(config.secrets).length;
      
      for (const variable of Object.values(config.variables)) {
        if (variable.encrypted) encryptedVariables++;
      }

      const validation = this.validateEnvironmentConfig(config);
      validationErrors += validation.errors.length;
    }

    return {
      totalConfigs: this.configs.size,
      totalVariables,
      totalSecrets,
      encryptedVariables,
      validationErrors
    };
  }

  /**
   * Clean up old configurations
   */
  async cleanupOldConfigs(maxAgeDays: number = 30): Promise<void> {
    const cutoffDate = new Date();
    cutoffDate.setDate(cutoffDate.getDate() - maxAgeDays);

    for (const [configId, config] of this.configs.entries()) {
      const configDate = new Date(config.updatedAt);
      if (configDate < cutoffDate) {
        this.configs.delete(configId);
        
        // Remove associated files
        try {
          const configPath = path.join(this.configDir, `${configId}.json`);
          await fsPromises.unlink(configPath);
          
          const envPath = path.join(this.configDir, `${configId}.env`);
          await fsPromises.unlink(envPath);
        } catch (error) {
          // Files might not exist
        }
      }
    }

    logger.info(`Cleaned up old environment configurations (older than ${maxAgeDays} days)`);
  }
}

// Export singleton instance
export const environmentVariableService = new EnvironmentVariableService(); 