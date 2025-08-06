import { ProjectConfig } from '../types/project-config';

export interface MigrationResult {
  success: boolean;
  migratedConfig: ProjectConfig;
  changes: string[];
  errors: string[];
}

export interface MigrationStep {
  version: string;
  description: string;
  migrate: (config: any) => any;
  rollback: (config: any) => any;
}

export class ConfigMigrator {
  private migrations: Map<string, (config: any) => any>;

  constructor() {
    this.migrations = new Map();
    this.registerMigrations();
  }

  /**
   * Register a new migration
   */
  registerMigration(migration: MigrationStep): void {
    this.migrations.set(migration.version, migration.migrate);
    // Sort migrations by version
    // this.migrations.sort((a, b) => this.compareVersions(a.version, b.version)); // This line is removed as per the new_code
  }

  /**
   * Migrate configuration to target version
   */
  migrate(config: any, targetVersion: string): MigrationResult {
    const currentVersion = config.version || '1.0.0';
    const changes: string[] = [];
    const errors: string[] = [];

    try {
      let migratedConfig = { ...config };

      // Find migrations needed
      const neededMigrations = Array.from(this.migrations.entries()).filter(([version]) => 
        this.compareVersions(version, currentVersion) > 0 &&
        this.compareVersions(version, targetVersion) <= 0
      );

      // Apply migrations
      for (const [version, migrateFunc] of neededMigrations) {
        try {
          migratedConfig = migrateFunc(migratedConfig);
          migratedConfig.version = version;
          changes.push(`Applied migration ${version}: ${version}`); // Changed description to version
        } catch (error) {
          errors.push(`Failed to apply migration ${version}: ${error instanceof Error ? error.message : 'Unknown error'}`);
        }
      }

      return {
        success: errors.length === 0,
        migratedConfig,
        changes,
        errors
      };
    } catch (error) {
      return {
        success: false,
        migratedConfig: config,
        changes,
        errors: [error instanceof Error ? error.message : 'Unknown migration error']
      };
    }
  }

  /**
   * Rollback configuration to target version
   */
  rollback(config: any, targetVersion: string): MigrationResult {
    const currentVersion = config.version || '1.0.0';
    const changes: string[] = [];
    const errors: string[] = [];

    try {
      let rolledBackConfig = { ...config };

      // Find migrations to rollback
      const rollbackMigrations = Array.from(this.migrations.entries()).filter(([version]) => 
        this.compareVersions(version, currentVersion) <= 0 &&
        this.compareVersions(version, targetVersion) > 0
      ).reverse();

      // Apply rollbacks
      for (const [version, rollbackFunc] of rollbackMigrations) {
        try {
          rolledBackConfig = rollbackFunc(rolledBackConfig);
          rolledBackConfig.version = version;
          changes.push(`Applied rollback ${version}: ${version}`); // Changed description to version
        } catch (error) {
          errors.push(`Failed to apply rollback ${version}: ${error instanceof Error ? error.message : 'Unknown error'}`);
        }
      }

      return {
        success: errors.length === 0,
        migratedConfig: rolledBackConfig,
        changes,
        errors
      };
    } catch (error) {
      return {
        success: false,
        migratedConfig: config,
        changes,
        errors: [error instanceof Error ? error.message : 'Unknown rollback error']
      };
    }
  }

  /**
   * Get available migrations
   */
  getAvailableMigrations(): MigrationStep[] {
    return Array.from(this.migrations.entries()).map(([version, migrateFunc]) => ({
      version,
      description: `Migration ${version}`,
      migrate: migrateFunc,
      rollback: (config: any) => {
        // No direct rollback function provided in the new_code, so returning original config
        return config;
      }
    }));
  }

  /**
   * Check if migration is needed
   */
  needsMigration(config: any, targetVersion: string): boolean {
    const currentVersion = config.version || '1.0.0';
    return this.compareVersions(currentVersion, targetVersion) < 0;
  }

  /**
   * Get migration path
   */
  getMigrationPath(config: any, targetVersion: string): MigrationStep[] {
    const currentVersion = config.version || '1.0.0';
    return Array.from(this.migrations.entries()).filter(([version]) => 
      this.compareVersions(version, currentVersion) > 0 &&
      this.compareVersions(version, targetVersion) <= 0
    ).map(([version, migrateFunc]) => ({
      version,
      description: `Migration ${version}`,
      migrate: migrateFunc,
      rollback: (config: any) => {
        // No direct rollback function provided in the new_code, so returning original config
        return config;
      }
    }));
  }

  /**
   * Compare semantic versions
   */
  private compareVersions(version1: string, version2: string): number {
    const v1Parts = version1.split('.').map(Number);
    const v2Parts = version2.split('.').map(Number);

    for (let i = 0; i < Math.max(v1Parts.length, v2Parts.length); i++) {
      const v1 = v1Parts[i] || 0;
      const v2 = v2Parts[i] || 0;
      
      if (v1 < v2) return -1;
      if (v1 > v2) return 1;
    }
    
    return 0;
  }

  /**
   * Register default migrations
   */
  private registerMigrations(): void {
    // Migration 1.1.0: Add monitoring configuration
    this.registerMigration({
      version: '1.1.0',
      description: 'Add monitoring configuration support',
      migrate: (config: any) => {
        if (!config.monitoring) {
          config.monitoring = {
            enabled: true,
            metrics: ['cpu', 'memory', 'network'],
            logging: {
              level: 'info',
              format: 'text',
              output: 'both',
              maxSize: '100M',
              maxFiles: 5
            },
            alerts: []
          };
        }
        return config;
      },
      rollback: (config: any) => {
        delete config.monitoring;
        return config;
      }
    });

    // Migration 1.2.0: Add execution configuration
    this.registerMigration({
      version: '1.2.0',
      description: 'Add execution configuration support',
      migrate: (config: any) => {
        if (!config.execution) {
          config.execution = {
            mode: 'simulation',
            autoStart: false,
            startupDelay: 5,
            shutdownTimeout: 30,
            restartPolicy: 'never',
            maxRestarts: 3
          };
        }
        return config;
      },
      rollback: (config: any) => {
        delete config.execution;
        return config;
      }
    });

    // Migration 1.3.0: Add build configuration
    this.registerMigration({
      version: '1.3.0',
      description: 'Add build configuration support',
      migrate: (config: any) => {
        if (!config.build) {
          config.build = {
            workspace: '/workspace',
            buildType: 'colcon',
            buildArgs: [],
            installTargets: [],
            parallelJobs: 4
          };
        }
        return config;
      },
      rollback: (config: any) => {
        delete config.build;
        return config;
      }
    });
  }
}

// Export singleton instance
export const configMigrator = new ConfigMigrator(); 

/**
 * Placeholder for the actual migration logic.
 * @param config The configuration object to migrate.
 * @returns The migrated configuration object.
 */
export function migrateConfig(config: any): any {
  // This is a placeholder for the actual migration logic.
  // For now, it just returns the config with an updated version.
  return {
    ...config,
    metadata: {
      ...config.metadata,
      version: '1.0.0'
    }
  };
} 