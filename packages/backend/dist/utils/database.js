"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.Database = exports.pool = void 0;
const pg_1 = require("pg");
// Database configuration
const dbConfig = {
    host: process.env.DB_HOST || 'localhost',
    port: parseInt(process.env.DB_PORT || '5432'),
    database: process.env.DB_NAME || 'robiumdb',
    user: process.env.DB_USER || 'user',
    password: process.env.DB_PASSWORD || 'password',
    // Connection pool settings
    min: 2,
    max: 20,
    idleTimeoutMillis: 30000,
    connectionTimeoutMillis: 2000, // Return error after 2 seconds if connection cannot be established
};
// Create connection pool
exports.pool = new pg_1.Pool(dbConfig);
// Database connection helper
class Database {
    static async connect() {
        try {
            const client = await exports.pool.connect();
            console.log('✅ Database connected successfully');
            client.release();
        }
        catch (error) {
            console.error('❌ Database connection failed:', error);
            throw error;
        }
    }
    static async disconnect() {
        try {
            await exports.pool.end();
            console.log('✅ Database disconnected successfully');
        }
        catch (error) {
            console.error('❌ Database disconnection failed:', error);
            throw error;
        }
    }
    static async query(text, params) {
        const start = Date.now();
        try {
            const result = await exports.pool.query(text, params);
            const duration = Date.now() - start;
            console.log('📊 Query executed', {
                text,
                duration,
                rows: result.rowCount,
            });
            return result;
        }
        catch (error) {
            console.error('❌ Query failed:', { text, params, error });
            throw error;
        }
    }
    static async transaction(callback) {
        const client = await exports.pool.connect();
        try {
            await client.query('BEGIN');
            const result = await callback(client);
            await client.query('COMMIT');
            return result;
        }
        catch (error) {
            await client.query('ROLLBACK');
            throw error;
        }
        finally {
            client.release();
        }
    }
    // Health check for database
    static async healthCheck() {
        try {
            const result = await exports.pool.query('SELECT NOW()');
            return !!result.rows[0];
        }
        catch (error) {
            console.error('Database health check failed:', error);
            return false;
        }
    }
}
exports.Database = Database;
// Handle graceful shutdown
process.on('SIGINT', async () => {
    console.log('🔄 Shutting down database connections...');
    await Database.disconnect();
    process.exit(0);
});
process.on('SIGTERM', async () => {
    console.log('🔄 Shutting down database connections...');
    await Database.disconnect();
    process.exit(0);
});
