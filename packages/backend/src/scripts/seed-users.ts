#!/usr/bin/env ts-node

import dotenv from 'dotenv';
import bcrypt from 'bcryptjs';
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

  // Upsert admin
  await Database.query(
    `INSERT INTO users (email, username, password_hash, role)
     VALUES ($1, $2, $3, 'admin')
     ON CONFLICT (email) DO UPDATE SET password_hash = EXCLUDED.password_hash, role = 'admin', updated_at = NOW()`,
    [adminEmail, adminUsername, passwordHash]
  );

  // Upsert test user
  await Database.query(
    `INSERT INTO users (email, username, password_hash, role)
     VALUES ($1, $2, $3, 'user')
     ON CONFLICT (email) DO UPDATE SET password_hash = EXCLUDED.password_hash, role = 'user', updated_at = NOW()`,
    [testEmail, testUsername, passwordHash]
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
