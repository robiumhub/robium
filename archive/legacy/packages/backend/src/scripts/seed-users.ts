#!/usr/bin/env ts-node

import dotenv from 'dotenv';
import bcrypt from 'bcryptjs';
import { v4 as uuidv4 } from 'uuid';
import { Database } from '../utils/database';
import { MigrationManager } from '../utils/migrations';

dotenv.config();

async function seedUsers() {
  const adminEmail = process.env.SEED_ADMIN_EMAIL || 'admin@robium.dev';
  const adminUsername = process.env.SEED_ADMIN_USERNAME || 'admin';
  const testEmail = process.env.SEED_TEST_EMAIL || 'test@robium.dev';
  const testUsername = process.env.SEED_TEST_USERNAME || 'test';
  const plainPassword = process.env.SEED_PASSWORD || 'bn193431';

  const passwordHash = await bcrypt.hash(plainPassword, 12);

  // Ensure DB is ready and migrations are applied
  const migrationManager = new MigrationManager();
  await migrationManager.runPendingMigrations();

  // Upsert admin (SQLite compatible)
  await Database.query(
    `INSERT OR REPLACE INTO users (id, email, username, password_hash, role, updated_at)
     VALUES (?, ?, ?, ?, 'admin', CURRENT_TIMESTAMP)`,
    [uuidv4(), adminEmail, adminUsername, passwordHash]
  );

  // Upsert test user (SQLite compatible)
  await Database.query(
    `INSERT OR REPLACE INTO users (id, email, username, password_hash, role, updated_at)
     VALUES (?, ?, ?, ?, 'user', CURRENT_TIMESTAMP)`,
    [uuidv4(), testEmail, testUsername, passwordHash]
  );

  // Ensure existing rows have ids (SQLite may allow nulls earlier)
  await Database.query(
    `UPDATE users SET id = ? WHERE email = ? AND (id IS NULL OR id = '')`,
    [uuidv4(), adminEmail]
  );
  await Database.query(
    `UPDATE users SET id = ? WHERE email = ? AND (id IS NULL OR id = '')`,
    [uuidv4(), testEmail]
  );

  console.log('✅ Seeded users:');
  console.log(` - ${adminEmail} (admin)`);
  console.log(` - ${testEmail} (user)`);
}

async function main() {
  try {
    await Database.connect();
    await seedUsers();
  } catch (error) {
    console.error('❌ Failed to seed users:', error);
    process.exit(1);
  } finally {
    await Database.disconnect();
  }
}

if (require.main === module) {
  main();
}
