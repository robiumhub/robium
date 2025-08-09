import path from 'path';
import dotenv from 'dotenv';
import { Database } from '../utils/database';

// Load backend .env explicitly
dotenv.config({ path: path.resolve(__dirname, '../../.env') });

async function main() {
  try {
    console.log('🔍 Testing database connection...');
    
    // Test database connection
    await Database.connect();
    console.log('✅ Database connected successfully');
    
    // Test health check
    const health = await Database.healthCheck();
    console.log('✅ Database health check:', health);
    
    // Test user lookup
    console.log('🔍 Testing user lookup...');
    const result = await Database.query(
      'SELECT id, email, username, role FROM users WHERE email = $1',
      ['mahmutdemir@gmail.com']
    );
    
    console.log('✅ User lookup result:', (result as any).rows);
    
    // Test password hash lookup
    console.log('🔍 Testing password hash lookup...');
    const passwordResult = await Database.query(
      'SELECT id, email, username, password_hash, role FROM users WHERE email = $1',
      ['mahmutdemir@gmail.com']
    );
    
    console.log('✅ Password hash lookup result:', {
      id: (passwordResult as any).rows[0]?.id,
      email: (passwordResult as any).rows[0]?.email,
      hasPasswordHash: !!(passwordResult as any).rows[0]?.password_hash,
      role: (passwordResult as any).rows[0]?.role
    });
    
    process.exit(0);
  } catch (error) {
    console.error('❌ Database test failed:', error);
    process.exit(1);
  }
}

main();
