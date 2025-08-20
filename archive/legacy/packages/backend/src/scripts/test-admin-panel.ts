import path from 'path';
import dotenv from 'dotenv';
import { Database } from '../utils/database';
import { AuthService } from '../services/AuthService';

// Load backend .env explicitly
dotenv.config({ path: path.resolve(__dirname, '../../.env') });

async function main() {
  try {
    await Database.connect();

    // Test login with admin user
    console.log('🔐 Testing admin login...');
    const loginResult = await AuthService.login({
      email: 'mahmutdemir@gmail.com',
      password: 'admin123',
    });

    console.log('✅ Login successful:', {
      token: loginResult.token ? 'Present' : 'Missing',
      user: loginResult.user,
    });

    // Test admin endpoint with token
    if (loginResult.token) {
      console.log('🔍 Testing admin robots overview endpoint...');
      const response = await fetch(
        'http://localhost:8000/admin/robots/overview',
        {
          headers: {
            Authorization: `Bearer ${loginResult.token}`,
            'Content-Type': 'application/json',
          },
        }
      );

      if (response.ok) {
        const data = await response.json();
        console.log('✅ Admin panel working:', data);
      } else {
        const error = await response.text();
        console.log('❌ Admin panel error:', response.status, error);
      }
    }

    process.exit(0);
  } catch (err) {
    console.error('❌ Failed to test admin panel:', err);
    process.exit(1);
  }
}

main();
