import { Database } from './database';
import fs from 'fs';
import path from 'path';
import { PoolClient } from 'pg';

export interface Migration {
  id: string;
  name: string;
  up: string;
  down: string;
  executedAt?: Date;
}

export class MigrationManager {
  private migrationsPath: string;

  constructor() {
    this.migrationsPath = path.join(__dirname, '../migrations');
  }

  // Create migrations table if it doesn't exist
  async createMigrationsTable(): Promise<void> {
    const query = `
      CREATE TABLE IF NOT EXISTS migrations (
        id VARCHAR(255) PRIMARY KEY,
        name VARCHAR(255) NOT NULL,
        executed_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
      );
    `;
    await Database.query(query);
    console.log('✅ Migrations table created');
  }

  // Get executed migrations from database
  async getExecutedMigrations(): Promise<string[]> {
    try {
      const result = (await Database.query(
        'SELECT id FROM migrations ORDER BY executed_at'
      )) as { rows: { id: string }[] };
      return result.rows.map((row) => row.id);
    } catch (error) {
      console.log('📋 No migrations table found, will create it');
      return [];
    }
  }

  // Get pending migrations from files
  async getPendingMigrations(): Promise<Migration[]> {
    const executedMigrations = await this.getExecutedMigrations();
    const migrationFiles = fs
      .readdirSync(this.migrationsPath)
      .filter((file) => file.endsWith('.sql'))
      .sort();

    const pendingMigrations: Migration[] = [];

    for (const file of migrationFiles) {
      const migrationId = file.replace('.sql', '');

      if (!executedMigrations.includes(migrationId)) {
        const filePath = path.join(this.migrationsPath, file);
        const content = fs.readFileSync(filePath, 'utf8');

        // Parse migration file (expects -- UP and -- DOWN sections)
        const sections = content.split('-- DOWN');
        const upSection = sections[0].replace('-- UP', '').trim();
        const downSection = sections[1] ? sections[1].trim() : '';

        pendingMigrations.push({
          id: migrationId,
          name: file,
          up: upSection,
          down: downSection,
        });
      }
    }

    return pendingMigrations;
  }

  // Run a single migration
  async runMigration(migration: Migration): Promise<void> {
    try {
      await Database.transaction(async (client: PoolClient) => {
        // Execute the UP migration
        await client.query(migration.up);

        // Record the migration as executed
        await client.query(
          'INSERT INTO migrations (id, name) VALUES ($1, $2)',
          [migration.id, migration.name]
        );
      });

      console.log(`✅ Migration executed: ${migration.name}`);
    } catch (error) {
      console.error(`❌ Migration failed: ${migration.name}`, error);
      throw error;
    }
  }

  // Run all pending migrations
  async runPendingMigrations(): Promise<void> {
    await this.createMigrationsTable();
    const pendingMigrations = await this.getPendingMigrations();

    if (pendingMigrations.length === 0) {
      console.log('✅ No pending migrations');
      return;
    }

    console.log(`📋 Running ${pendingMigrations.length} pending migrations...`);

    for (const migration of pendingMigrations) {
      await this.runMigration(migration);
    }

    console.log('✅ All migrations completed');
  }

  // Rollback the last migration
  async rollbackLastMigration(): Promise<void> {
    const result = (await Database.query(
      'SELECT id, name FROM migrations ORDER BY executed_at DESC LIMIT 1'
    )) as { rows: { id: string; name: string }[] };

    if (result.rows.length === 0) {
      console.log('📋 No migrations to rollback');
      return;
    }

    const lastMigration = result.rows[0];
    const migrationFile = path.join(
      this.migrationsPath,
      `${lastMigration.id}.sql`
    );

    if (!fs.existsSync(migrationFile)) {
      throw new Error(`Migration file not found: ${migrationFile}`);
    }

    const content = fs.readFileSync(migrationFile, 'utf8');
    const sections = content.split('-- DOWN');
    const downSection = sections[1] ? sections[1].trim() : '';

    if (!downSection) {
      throw new Error(
        `No DOWN section found in migration: ${lastMigration.name}`
      );
    }

    try {
      await Database.transaction(async (client: PoolClient) => {
        // Execute the DOWN migration
        await client.query(downSection);

        // Remove the migration record
        await client.query('DELETE FROM migrations WHERE id = $1', [
          lastMigration.id,
        ]);
      });

      console.log(`✅ Migration rolled back: ${lastMigration.name}`);
    } catch (error) {
      console.error(`❌ Rollback failed: ${lastMigration.name}`, error);
      throw error;
    }
  }

  // Get migration status
  async getMigrationStatus(): Promise<void> {
    const executedMigrations = await this.getExecutedMigrations();
    const pendingMigrations = await this.getPendingMigrations();

    console.log('\n📋 Migration Status:');
    console.log(`✅ Executed: ${executedMigrations.length}`);
    console.log(`⏳ Pending: ${pendingMigrations.length}`);

    if (pendingMigrations.length > 0) {
      console.log('\n⏳ Pending migrations:');
      pendingMigrations.forEach((m) => console.log(`  - ${m.name}`));
    }
  }
}
