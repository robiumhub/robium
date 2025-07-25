import { Pool, PoolConfig } from 'pg';

// Database configuration
const dbConfig: PoolConfig = {
  host: process.env.DB_HOST || 'localhost',
  port: parseInt(process.env.DB_PORT || '5432'),
  database: process.env.DB_NAME || 'robiumdb',
  user: process.env.DB_USER || 'user',
  password: process.env.DB_PASSWORD || 'password',
  // Connection pool settings
  min: 2, // Minimum number of connections
  max: 20, // Maximum number of connections
  idleTimeoutMillis: 30000, // Close idle connections after 30 seconds
  connectionTimeoutMillis: 2000, // Return error after 2 seconds if connection cannot be established
};

// Create connection pool
export const pool = new Pool(dbConfig);

// Database connection helper
export class Database {
  static async connect(): Promise<void> {
    try {
      const client = await pool.connect();
      console.log('✅ Database connected successfully');
      client.release();
    } catch (error) {
      console.error('❌ Database connection failed:', error);
      throw error;
    }
  }

  static async disconnect(): Promise<void> {
    try {
      await pool.end();
      console.log('✅ Database disconnected successfully');
    } catch (error) {
      console.error('❌ Database disconnection failed:', error);
      throw error;
    }
  }

  static async query(text: string, params?: unknown[]): Promise<unknown> {
    const start = Date.now();
    try {
      const result = await pool.query(text, params);
      const duration = Date.now() - start;
      console.log('📊 Query executed', {
        text,
        duration,
        rows: result.rowCount,
      });
      return result;
    } catch (error) {
      console.error('❌ Query failed:', { text, params, error });
      throw error;
    }
  }

  static async transaction<T>(
    callback: (client: unknown) => Promise<T>
  ): Promise<T> {
    const client = await pool.connect();
    try {
      await client.query('BEGIN');
      const result = await callback(client);
      await client.query('COMMIT');
      return result;
    } catch (error) {
      await client.query('ROLLBACK');
      throw error;
    } finally {
      client.release();
    }
  }

  // Health check for database
  static async healthCheck(): Promise<boolean> {
    try {
      const result = await pool.query('SELECT NOW()');
      return !!result.rows[0];
    } catch (error) {
      console.error('Database health check failed:', error);
      return false;
    }
  }
}

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
