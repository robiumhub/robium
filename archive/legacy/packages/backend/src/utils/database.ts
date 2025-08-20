// eslint-disable-next-line @typescript-eslint/ban-ts-comment
// @ts-ignore - types are optional; we use runtime API only
import DatabaseSqlite from 'better-sqlite3';
import path from 'path';
import fs from 'fs';

type QueryResult<Row = any> = { rows: Row[]; rowCount: number | null };

// Singleton for SQLite database
let sqliteDb: DatabaseSqlite.Database | null = null;

function ensureSqlite(): DatabaseSqlite.Database {
  if (!sqliteDb) {
    const sqliteDir =
      process.env.SQLITE_DIR || path.join(__dirname, '../../generated');
    const sqliteFile =
      process.env.SQLITE_PATH || path.join(sqliteDir, 'robium.db');
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
      ensureSqlite();
      console.log('✅ SQLite connected successfully');
    } catch (error) {
      console.error('❌ Database connection failed:', error);
      throw error;
    }
  }

  static async disconnect(): Promise<void> {
    try {
      if (sqliteDb) {
        sqliteDb.close();
        sqliteDb = null;
      }
      console.log('✅ SQLite disconnected successfully');
    } catch (error) {
      console.error('❌ Database disconnection failed:', error);
      throw error;
    }
  }

  static async query<Row = any>(
    text: string,
    params: unknown[] = []
  ): Promise<QueryResult<Row>> {
    const start = Date.now();
    try {
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
      console.log('📊 Query executed', {
        text: sql,
        duration,
        rows: rowCount,
      });
      return { rows, rowCount };
    } catch (error) {
      console.error('❌ Query failed:', { text, params, error });
      throw error;
    }
  }

  static async transaction<T>(
    callback: (client: { query: typeof Database.query }) => Promise<T>
  ): Promise<T> {
    const db = ensureSqlite();
    try {
      db.exec('BEGIN');
      const result = await callback({
        query: async (text: string, params?: unknown[]) =>
          Database.query(text, params as any[]),
      });
      db.exec('COMMIT');
      return result;
    } catch (error) {
      try {
        db.exec('ROLLBACK');
      } catch {}
      throw error;
    }
  }

  static async healthCheck(): Promise<boolean> {
    try {
      const db = ensureSqlite();
      const row = db.prepare('SELECT 1 as ok').get() as
        | { ok: number }
        | undefined;
      return !!row && row.ok === 1;
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
