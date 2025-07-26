"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.MigrationManager = void 0;
const database_1 = require("./database");
const fs_1 = __importDefault(require("fs"));
const path_1 = __importDefault(require("path"));
class MigrationManager {
    constructor() {
        this.migrationsPath = path_1.default.join(__dirname, '../migrations');
    }
    // Create migrations table if it doesn't exist
    async createMigrationsTable() {
        const query = `
      CREATE TABLE IF NOT EXISTS migrations (
        id VARCHAR(255) PRIMARY KEY,
        name VARCHAR(255) NOT NULL,
        executed_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
      );
    `;
        await database_1.Database.query(query);
        console.log('✅ Migrations table created');
    }
    // Get executed migrations from database
    async getExecutedMigrations() {
        try {
            const result = (await database_1.Database.query('SELECT id FROM migrations ORDER BY executed_at'));
            return result.rows.map((row) => row.id);
        }
        catch (error) {
            console.log('📋 No migrations table found, will create it');
            return [];
        }
    }
    // Get pending migrations from files
    async getPendingMigrations() {
        const executedMigrations = await this.getExecutedMigrations();
        const migrationFiles = fs_1.default
            .readdirSync(this.migrationsPath)
            .filter((file) => file.endsWith('.sql'))
            .sort();
        const pendingMigrations = [];
        for (const file of migrationFiles) {
            const migrationId = file.replace('.sql', '');
            if (!executedMigrations.includes(migrationId)) {
                const filePath = path_1.default.join(this.migrationsPath, file);
                const content = fs_1.default.readFileSync(filePath, 'utf8');
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
    async runMigration(migration) {
        try {
            await database_1.Database.transaction(async (client) => {
                const poolClient = client;
                // Execute the UP migration
                await poolClient.query(migration.up);
                // Record the migration as executed
                await poolClient.query('INSERT INTO migrations (id, name) VALUES ($1, $2)', [migration.id, migration.name]);
            });
            console.log(`✅ Migration executed: ${migration.name}`);
        }
        catch (error) {
            console.error(`❌ Migration failed: ${migration.name}`, error);
            throw error;
        }
    }
    // Run all pending migrations
    async runPendingMigrations() {
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
    async rollbackLastMigration() {
        const result = (await database_1.Database.query('SELECT id, name FROM migrations ORDER BY executed_at DESC LIMIT 1'));
        if (result.rows.length === 0) {
            console.log('📋 No migrations to rollback');
            return;
        }
        const lastMigration = result.rows[0];
        const migrationFile = path_1.default.join(this.migrationsPath, `${lastMigration.id}.sql`);
        if (!fs_1.default.existsSync(migrationFile)) {
            throw new Error(`Migration file not found: ${migrationFile}`);
        }
        const content = fs_1.default.readFileSync(migrationFile, 'utf8');
        const sections = content.split('-- DOWN');
        const downSection = sections[1] ? sections[1].trim() : '';
        if (!downSection) {
            throw new Error(`No DOWN section found in migration: ${lastMigration.name}`);
        }
        try {
            await database_1.Database.transaction(async (client) => {
                const poolClient = client;
                // Execute the DOWN migration
                await poolClient.query(downSection);
                // Remove the migration record
                await poolClient.query('DELETE FROM migrations WHERE id = $1', [
                    lastMigration.id,
                ]);
            });
            console.log(`✅ Migration rolled back: ${lastMigration.name}`);
        }
        catch (error) {
            console.error(`❌ Rollback failed: ${lastMigration.name}`, error);
            throw error;
        }
    }
    // Get migration status
    async getMigrationStatus() {
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
exports.MigrationManager = MigrationManager;
