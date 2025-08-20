import path from 'path';
import dotenv from 'dotenv';
import { AuthService } from '../services/AuthService';

// Load backend .env explicitly
dotenv.config({ path: path.resolve(__dirname, '../../.env') });

async function main() {
  try {
    console.log('🔍 Testing complete login flow...');
    
    // Test login with admin credentials
    console.log('🔐 Attempting login...');
    const loginResult = await AuthService.login({
      email: 'mahmutdemir@gmail.com',
      password: 'admin123'
    });
    
    console.log('✅ Login successful!');
    console.log('User:', {
      id: loginResult.user.id,
      email: loginResult.user.email,
      username: loginResult.user.username,
      role: loginResult.user.role
    });
    console.log('Token present:', !!loginResult.token);
    console.log('Expires in:', loginResult.expiresIn);
    
    // Test token verification
    console.log('🔍 Testing token verification...');
    const verifiedUser = await AuthService.verifyToken(loginResult.token);
    console.log('✅ Token verification successful:', {
      id: verifiedUser.id,
      email: verifiedUser.email,
      role: verifiedUser.role
    });
    
    process.exit(0);
  } catch (error) {
    console.error('❌ Login test failed:', error);
    console.error('Error details:', {
      name: error instanceof Error ? error.name : 'Unknown',
      message: error instanceof Error ? error.message : 'Unknown error',
      stack: error instanceof Error ? error.stack : undefined
    });
    process.exit(1);
  }
}

main();
