import path from 'path';
import dotenv from 'dotenv';
import { Database } from '../utils/database';

// Load backend .env explicitly
dotenv.config({ path: path.resolve(__dirname, '../../.env') });

async function main() {
  try {
    await Database.connect();
    const result = await Database.query(
      `SELECT id, email, username, role FROM users WHERE role = 'ADMIN' LIMIT 1`
    );

    if ((result as any).rows.length === 0) {
      console.log('No admin user found');
    } else {
      console.log('Admin user:', (result as any).rows[0]);
    }

    process.exit(0);
  } catch (err) {
    console.error('Failed to check admin user:', err);
    process.exit(1);
  }
}

main();
