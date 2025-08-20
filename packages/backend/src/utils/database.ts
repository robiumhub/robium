import type { PoolConfig } from 'pg';
import { Pool } from 'pg';
import DatabaseSqlite from 'better-sqlite3';
import path from 'path';
import fs from 'fs';

type QueryResult<Row = any> = { rows: Row[]; rowCount: number };

const DB_CLIENT = (process.env.DB_CLIENT || 'postgres').toLowerCase();

// Postgres configuration
const pgConfig: PoolConfig = {
  host: process.env.DB_HOST || 'localhost',
  port: parseInt(process.env.DB_PORT || '5432'),
  database: process.env.DB_NAME || 'robiumdb',
  user: process.env.DB_USER || 'user',
  password: process.env.DB_PASSWORD || 'password',
  min: 2,
  max: 20,
  idleTimeoutMillis: 30000,
  connectionTimeoutMillis: 2000,
};

// Singletons for clients
let pgPool: Pool | null = null;
let sqliteDb: DatabaseSqlite.Database | null = null;

function ensurePg(): Pool {
  if (!pgPool) {
    pgPool = new Pool(pgConfig);
  }
  return pgPool;
}

function ensureSqlite(): DatabaseSqlite.Database {
  if (!sqliteDb) {
    const sqliteDir = process.env.SQLITE_DIR || path.join(__dirname, '../../generated');
    const sqliteFile = process.env.SQLITE_PATH || path.join(sqliteDir, 'robium.db');
    if (!fs.existsSync(sqliteDir)) {
      fs.mkdirSync(sqliteDir, { recursive: true });
    }
    sqliteDb = new DatabaseSqlite(sqliteFile);
    // Enable foreign keys
    sqliteDb.pragma('foreign_keys = ON');
  }
  return sqliteDb;
}

function normalizeSqlForSqlite(sql: string): string {
  let out = sql;
  // Replace Postgres parameter style $1,$2,... with SQLite '?'
  out = out.replace(/\$\d+/g, '?');
  // Remove Postgres casts like ::int, ::text
  out = out.replace(/::\w+/g, '');
  // Replace booleans comparisons with 1/0 literals when used as equality
  out = out.replace(/=\s*true/gi, '= 1');
  out = out.replace(/=\s*false/gi, '= 0');
  return out;
}

export class Database {
  static async connect(): Promise<void> {
    try {
      if (DB_CLIENT === 'sqlite') {
        ensureSqlite();
        console.log('✅ SQLite connected successfully');
      } else {
        const client = await ensurePg().connect();
        client.release();
        console.log('✅ Postgres connected successfully');
      }
    } catch (error) {
      console.error('❌ Database connection failed:', error);
      throw error;
    }
  }

  static async disconnect(): Promise<void> {
    try {
      if (DB_CLIENT === 'sqlite') {
        if (sqliteDb) {
          sqliteDb.close();
          sqliteDb = null;
        }
        console.log('✅ SQLite disconnected successfully');
      } else {
        if (pgPool) {
          await pgPool.end();
          pgPool = null as any;
        }
        console.log('✅ Postgres disconnected successfully');
      }
    } catch (error) {
      console.error('❌ Database disconnection failed:', error);
      throw error;
    }
  }

  static async query<Row = any>(text: string, params: unknown[] = []): Promise<QueryResult<Row>> {
    const start = Date.now();
    try {
      if (DB_CLIENT === 'sqlite') {
        const db = ensureSqlite();
        const sql = normalizeSqlForSqlite(text);
        const hasReturning = /\bRETURNING\b/i.test(sql);
        const isSelect = /^\s*select\b/i.test(sql);
        const stmt = db.prepare(sql);
        let rows: Row[] = [];
        let rowCount = 0;
        if (isSelect || hasReturning) {
          rows = stmt.all(...params) as Row[];
          rowCount = rows.length;
        } else {
          const info = stmt.run(...params);
          rowCount = info.changes || 0;
          rows = [] as Row[];
        }
        const duration = Date.now() - start;
        console.log('📊 Query executed', { text: sql, duration, rows: rowCount });
        return { rows, rowCount };
      }

      const result = await ensurePg().query(text, params as any[]);
      const duration = Date.now() - start;
      console.log('📊 Query executed', {
        text,
        duration,
        rows: result.rowCount,
      });
      return { rows: result.rows as Row[], rowCount: result.rowCount };
    } catch (error) {
      console.error('❌ Query failed:', { text, params, error });
      throw error;
    }
  }

  static async transaction<T>(callback: (client: { query: typeof Database.query }) => Promise<T>): Promise<T> {
    if (DB_CLIENT === 'sqlite') {
      const db = ensureSqlite();
      try {
        db.exec('BEGIN');
        const result = await callback({
          query: async (text: string, params?: unknown[]) => Database.query(text, params as any[]),
        });
        db.exec('COMMIT');
        return result;
      } catch (error) {
        try { db.exec('ROLLBACK'); } catch {}
        throw error;
      }
    }

    const client = await ensurePg().connect();
    try {
      await client.query('BEGIN');
      const result = await callback({
        query: async (text: string, params?: unknown[]) => {
          const r = await client.query(text, params as any[]);
          return { rows: r.rows, rowCount: r.rowCount };
        },
      });
      await client.query('COMMIT');
      return result;
    } catch (error) {
      await client.query('ROLLBACK');
      throw error;
    } finally {
      client.release();
    }
  }

  static async healthCheck(): Promise<boolean> {
    try {
      if (DB_CLIENT === 'sqlite') {
        const db = ensureSqlite();
        const row = db.prepare('SELECT 1 as ok').get() as { ok: number } | undefined;
        return !!row && row.ok === 1;
      }
      const result = await ensurePg().query('SELECT NOW()');
      return !!(result.rows as any[])[0];
    } catch (error) {
      console.error('Database health check failed:', error);
      return false;
    }
  }
}

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
