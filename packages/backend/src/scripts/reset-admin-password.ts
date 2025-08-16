import path from 'path';
import dotenv from 'dotenv';
import bcrypt from 'bcryptjs';
import { Database } from '../utils/database';

// Load backend .env explicitly
dotenv.config({ path: path.resolve(__dirname, '../../.env') });

async function main() {
  const newPassword = 'bn193431';
  const email = 'admin@robium.dev';

  try {
    await Database.connect();

    // Hash the new password
    const passwordHash = await bcrypt.hash(newPassword, 12);

    // Update the user's password
    const result = await Database.query(
      `UPDATE users SET password_hash = $1, updated_at = NOW() WHERE email = $2 RETURNING id, email, username, role`,
      [passwordHash, email]
    );

    if ((result as any).rows.length === 0) {
      console.error('User not found:', email);
      process.exit(1);
    }

    console.log('✅ Password reset successful for:', (result as any).rows[0]);
    console.log('🔑 New password:', newPassword);

    process.exit(0);
  } catch (err) {
    console.error('❌ Failed to reset password:', err);
    process.exit(1);
  }
}

main();
