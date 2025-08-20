import { Database } from '../utils/database';
import bcrypt from 'bcryptjs';

async function main() {
  try {
    await Database.connect();
    const db = Database.getDatabase();

    // Hash password
    const passwordHash = await bcrypt.hash('password123', 12);

    // Insert sample users
    const insertUser = db.prepare(`
      INSERT OR REPLACE INTO users (id, email, username, password_hash, role, is_active)
      VALUES (?, ?, ?, ?, ?, ?)
    `);

    insertUser.run('1', 'admin@robium.com', 'admin', passwordHash, 'admin', 1);
    insertUser.run('2', 'user@robium.com', 'user', passwordHash, 'user', 1);

    console.log('Sample users seeded successfully');
  } catch (error) {
    console.error('Failed to seed users:', error);
    process.exit(1);
  } finally {
    Database.close();
  }
}

main();
