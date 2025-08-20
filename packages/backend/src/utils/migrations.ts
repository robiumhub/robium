import { Database } from './database';
import fs from 'fs';
import path from 'path';
import { PoolClient } from 'pg';

const DB_CLIENT = (process.env.DB_CLIENT || 'postgres').toLowerCase();

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
    if (DB_CLIENT === 'sqlite') {
      await Database.query(
        `CREATE TABLE IF NOT EXISTS migrations (
          id TEXT PRIMARY KEY,
          name TEXT NOT NULL,
          executed_at TEXT DEFAULT CURRENT_TIMESTAMP
        )`
      );
    } else {
      const query = `
        CREATE TABLE IF NOT EXISTS migrations (
          id VARCHAR(255) PRIMARY KEY,
          name VARCHAR(255) NOT NULL,
          executed_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        );
      `;
      await Database.query(query);
    }
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
      await Database.transaction(async (client: unknown) => {
        const poolClient = client as PoolClient;
        // Execute the UP migration
        await poolClient.query(migration.up);

        // Record the migration as executed
        await poolClient.query(
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
    if (DB_CLIENT === 'sqlite') {
      await this.initializeSqliteSchema();
      console.log('✅ SQLite baseline schema ensured');
      return;
    }

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

  private async initializeSqliteSchema(): Promise<void> {
    // Minimal schema to satisfy current code paths
    const statements: string[] = [
      // users
      `CREATE TABLE IF NOT EXISTS users (
        id TEXT PRIMARY KEY,
        email TEXT UNIQUE NOT NULL,
        username TEXT UNIQUE NOT NULL,
        password_hash TEXT NOT NULL,
        is_active INTEGER DEFAULT 1,
        role TEXT NOT NULL DEFAULT 'user',
        created_at TEXT DEFAULT CURRENT_TIMESTAMP,
        updated_at TEXT DEFAULT CURRENT_TIMESTAMP
      )`,
      `CREATE INDEX IF NOT EXISTS idx_users_email ON users(email)`,
      `CREATE INDEX IF NOT EXISTS idx_users_username ON users(username)`,
      // projects (minimal fields used by dashboard stats)
      `CREATE TABLE IF NOT EXISTS projects (
        id TEXT PRIMARY KEY,
        name TEXT NOT NULL,
        description TEXT,
        owner_id TEXT,
        is_active INTEGER DEFAULT 1,
        is_template INTEGER DEFAULT 0,
        created_at TEXT DEFAULT CURRENT_TIMESTAMP,
        updated_at TEXT DEFAULT CURRENT_TIMESTAMP
      )`,
      `CREATE INDEX IF NOT EXISTS idx_projects_is_active ON projects(is_active)`,
      `CREATE INDEX IF NOT EXISTS idx_projects_created_at ON projects(created_at)`,
      // ros_packages (minimal for stats)
      `CREATE TABLE IF NOT EXISTS ros_packages (
        id TEXT PRIMARY KEY,
        name TEXT NOT NULL UNIQUE,
        version TEXT DEFAULT '0.0.0',
        description TEXT,
        category TEXT,
        type TEXT DEFAULT 'mock',
        is_active INTEGER DEFAULT 1,
        created_at TEXT DEFAULT CURRENT_TIMESTAMP,
        updated_at TEXT DEFAULT CURRENT_TIMESTAMP
      )`,
      `CREATE INDEX IF NOT EXISTS idx_ros_packages_is_active ON ros_packages(is_active)`
    ];

    for (const sql of statements) {
      await Database.query(sql);
    }
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
      await Database.transaction(async (client: unknown) => {
        const poolClient = client as PoolClient;
        // Execute the DOWN migration
        await poolClient.query(downSection);

        // Remove the migration record
        await poolClient.query('DELETE FROM migrations WHERE id = $1', [
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
