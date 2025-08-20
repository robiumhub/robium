import sqlite3 from 'sqlite3';
import path from 'path';
import { logger } from './logger';

class DatabaseManager {
  private db: sqlite3.Database | null = null;
  private dbPath: string;

  constructor() {
    this.dbPath = path.join(__dirname, '../generated/robium.db');
  }

  async connect(): Promise<void> {
    try {
      // Ensure the generated directory exists
      const fs = require('fs');
      const dir = path.dirname(this.dbPath);
      if (!fs.existsSync(dir)) {
        fs.mkdirSync(dir, { recursive: true });
      }

      return new Promise((resolve, reject) => {
        this.db = new sqlite3.Database(this.dbPath, (err) => {
          if (err) {
            logger.error('Failed to connect to database', { error: err.message });
            reject(err);
          } else {
            // Enable foreign keys
            this.db!.run('PRAGMA foreign_keys = ON');

            // Enable WAL mode for better concurrency
            this.db!.run('PRAGMA journal_mode = WAL');

            logger.info('Database connected successfully', { path: this.dbPath });
            resolve();
          }
        });
      });
    } catch (error) {
      logger.error('Failed to connect to database', {
        error: error instanceof Error ? error.message : 'Unknown error',
      });
      throw error;
    }
  }

  getDatabase(): sqlite3.Database {
    if (!this.db) {
      throw new Error('Database not connected. Call connect() first.');
    }
    return this.db;
  }

  async healthCheck(): Promise<boolean> {
    try {
      if (!this.db) return false;
      return new Promise((resolve) => {
        this.db!.get('SELECT 1', (err) => {
          resolve(!err);
        });
      });
    } catch (error) {
      logger.error('Database health check failed', {
        error: error instanceof Error ? error.message : 'Unknown error',
      });
      return false;
    }
  }

  close(): void {
    if (this.db) {
      this.db.close();
      this.db = null;
      logger.info('Database connection closed');
    }
  }
}

export const Database = new DatabaseManager();
