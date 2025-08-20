import { Database } from './database';
import fs from 'fs';
import path from 'path';

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
    await Database.query(
      `CREATE TABLE IF NOT EXISTS migrations (
        id TEXT PRIMARY KEY,
        name TEXT NOT NULL,
        executed_at TEXT DEFAULT CURRENT_TIMESTAMP
      )`
    );
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

      // Skip the initial schema migration since we handle it in initializeSqliteSchema
      if (
        file === '001_initial_schema.sql' ||
        file === '001_initial_schema_sqlite.sql'
      ) {
        continue;
      }

      // Skip non-SQLite migrations
      if (file.includes('_sqlite')) {
        continue;
      }

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
      await Database.transaction(async (client) => {
        // Execute the UP migration
        await client.query(migration.up);

        // Record the migration as executed
        await client.query('INSERT INTO migrations (id, name) VALUES (?, ?)', [
          migration.id,
          migration.name,
        ]);
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
    await this.initializeSqliteSchema();
    console.log('✅ SQLite baseline schema ensured');

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
      // projects (extended for endpoints)
      `CREATE TABLE IF NOT EXISTS projects (
        id TEXT PRIMARY KEY,
        name TEXT NOT NULL,
        description TEXT,
        owner_id TEXT,
        category TEXT,
        type TEXT,
        is_active INTEGER DEFAULT 1,
        is_template INTEGER DEFAULT 0,
        author TEXT,
        license TEXT,
        config TEXT DEFAULT '{}',
        metadata TEXT DEFAULT '{}',
        github_repo_owner TEXT,
        github_repo_name TEXT,
        github_repo_url TEXT,
        github_repo_id TEXT,
        tags TEXT,
        created_at TEXT DEFAULT CURRENT_TIMESTAMP,
        updated_at TEXT DEFAULT CURRENT_TIMESTAMP
      )`,
      `CREATE INDEX IF NOT EXISTS idx_projects_is_active ON projects(is_active)`,
      `CREATE INDEX IF NOT EXISTS idx_projects_created_at ON projects(created_at)`,
      `CREATE INDEX IF NOT EXISTS idx_projects_owner_id ON projects(owner_id)`,
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
      `CREATE INDEX IF NOT EXISTS idx_ros_packages_is_active ON ros_packages(is_active)`,
      // ensure optional columns for tags & supported_robots (handle gracefully for SQLite)
      // Note: These will fail if columns already exist, but that's okay
      // modules (minimal)
      `CREATE TABLE IF NOT EXISTS modules (
        id TEXT PRIMARY KEY,
        name TEXT NOT NULL UNIQUE,
        version TEXT,
        description TEXT,
        category TEXT,
        type TEXT,
        supported_robots TEXT,
        is_active INTEGER DEFAULT 1,
        is_default INTEGER DEFAULT 0,
        created_at TEXT DEFAULT CURRENT_TIMESTAMP,
        updated_at TEXT DEFAULT CURRENT_TIMESTAMP
      )`,
      `CREATE INDEX IF NOT EXISTS idx_modules_name ON modules(name)`,
      // project relations used by endpoints
      `CREATE TABLE IF NOT EXISTS project_module_dependencies (
        project_id TEXT NOT NULL,
        module_id TEXT NOT NULL,
        dependency_type TEXT,
        version_constraint TEXT,
        order_index INTEGER DEFAULT 0
      )`,
      `CREATE INDEX IF NOT EXISTS idx_pmd_project_id ON project_module_dependencies(project_id)`,
      `CREATE TABLE IF NOT EXISTS project_packages (
        project_id TEXT NOT NULL,
        package_id TEXT NOT NULL,
        is_required INTEGER DEFAULT 1,
        order_index INTEGER DEFAULT 0
      )`,
      `CREATE INDEX IF NOT EXISTS idx_pp_project_id ON project_packages(project_id)`,
      `CREATE TABLE IF NOT EXISTS project_files (
        project_id TEXT NOT NULL,
        file_path TEXT NOT NULL,
        file_type TEXT,
        content TEXT,
        content_hash TEXT,
        is_generated INTEGER DEFAULT 0
      )`,
      `CREATE INDEX IF NOT EXISTS idx_pf_project_id ON project_files(project_id)`,
      // datasets (minimal)
      `CREATE TABLE IF NOT EXISTS datasets (
        id TEXT PRIMARY KEY,
        name TEXT NOT NULL,
        description TEXT,
        owner_id TEXT,
        is_public INTEGER DEFAULT 1,
        created_at TEXT DEFAULT CURRENT_TIMESTAMP,
        updated_at TEXT DEFAULT CURRENT_TIMESTAMP
      )`,
      `CREATE INDEX IF NOT EXISTS idx_datasets_owner_id ON datasets(owner_id)`,
      // project_configurations (minimal)
      `CREATE TABLE IF NOT EXISTS project_configurations (
        project_id TEXT PRIMARY KEY,
        type TEXT,
        base_image TEXT,
        workdir TEXT,
        environment_variables TEXT,
        ports TEXT,
        created_at TEXT DEFAULT CURRENT_TIMESTAMP,
        updated_at TEXT DEFAULT CURRENT_TIMESTAMP
      )`,
      `CREATE INDEX IF NOT EXISTS idx_project_configurations_project_id ON project_configurations(project_id)`,
    ];

    for (const sql of statements) {
      try {
        await Database.query(sql);
      } catch (error) {
        // Ignore errors for ALTER TABLE statements that might fail if columns already exist
        if (
          sql.includes('ALTER TABLE') &&
          error instanceof Error &&
          error.message.includes('duplicate column')
        ) {
          console.log(
            `⚠️  Column already exists, skipping: ${sql.substring(0, 50)}...`
          );
        } else {
          throw error;
        }
      }
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
      await Database.transaction(async (client) => {
        // Execute the DOWN migration
        await client.query(downSection);

        // Remove the migration record
        await client.query('DELETE FROM migrations WHERE id = ?', [
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
