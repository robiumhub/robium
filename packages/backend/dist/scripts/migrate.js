#!/usr/bin/env ts-node
"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
const migrations_1 = require("../utils/migrations");
const database_1 = require("../utils/database");
const dotenv_1 = __importDefault(require("dotenv"));
// Load environment variables
dotenv_1.default.config();
const migrationManager = new migrations_1.MigrationManager();
async function main() {
    const command = process.argv[2];
    try {
        // Connect to database
        await database_1.Database.connect();
        switch (command) {
            case 'up':
            case 'migrate':
                console.log('🚀 Running migrations...');
                await migrationManager.runPendingMigrations();
                break;
            case 'down':
            case 'rollback':
                console.log('🔄 Rolling back last migration...');
                await migrationManager.rollbackLastMigration();
                break;
            case 'status':
                console.log('📋 Checking migration status...');
                await migrationManager.getMigrationStatus();
                break;
            case 'reset': {
                console.log('⚠️  Resetting database (rolling back all migrations)...');
                // Rollback all migrations
                let hasMore = true;
                while (hasMore) {
                    try {
                        await migrationManager.rollbackLastMigration();
                    }
                    catch (error) {
                        hasMore = false;
                    }
                }
                console.log('✅ Database reset complete');
                break;
            }
            default:
                console.log(`
🗃️  Robium Database Migration Tool

Usage: npm run migrate <command>

Commands:
  up, migrate    Run all pending migrations
  down, rollback Roll back the last migration
  status         Show migration status
  reset          Roll back all migrations (⚠️  destructive)

Examples:
  npm run migrate up
  npm run migrate status
  npm run migrate rollback
        `);
                break;
        }
    }
    catch (error) {
        console.error('❌ Migration failed:', error);
        process.exit(1);
    }
    finally {
        // Close database connection
        await database_1.Database.disconnect();
    }
}
// Run if called directly
if (require.main === module) {
    main();
}
