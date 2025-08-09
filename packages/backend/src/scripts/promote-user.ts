import path from 'path';
import dotenv from 'dotenv';
import { Database } from '../utils/database';

// Load backend .env explicitly
dotenv.config({ path: path.resolve(__dirname, '../../.env') });

async function main() {
  const userId = process.argv[2];
  if (!userId) {
    console.error('Usage: ts-node promote-user.ts <USER_ID>');
    process.exit(1);
  }

  try {
    await Database.connect();
    const result = await Database.query(
      `UPDATE users SET role = 'admin', updated_at = NOW() WHERE id = $1 RETURNING id, email, role`,
      [userId]
    );

    if ((result as any).rows.length === 0) {
      console.error('User not found:', userId);
      process.exit(1);
    }

    console.log('Promoted user to ADMIN:', (result as any).rows[0]);
    process.exit(0);
  } catch (err) {
    console.error('Failed to promote user:', err);
    process.exit(1);
  }
}

main();
