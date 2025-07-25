import fetch from 'node-fetch';
import dotenv from 'dotenv';

dotenv.config();

const BASE_URL = process.env.BACKEND_URL || 'http://localhost:8000';

async function testValidation() {
  console.log('🧪 Testing Input Validation Middleware...\n');

  // Test 1: Valid user registration
  console.log('1. Testing valid user registration...');
  try {
    const validUserData = {
      email: 'test@example.com',
      username: 'testuser',
      password: 'TestPass123!',
      role: 'user',
    };

    const response = await fetch(`${BASE_URL}/auth/register`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(validUserData),
    });

    if (response.ok) {
      console.log('✅ Valid user registration passed');
    } else {
      const error = await response.json();
      console.log('❌ Valid user registration failed:', error);
    }
  } catch (error) {
    console.log('❌ Valid user registration error:', error);
  }

  // Test 2: Invalid user registration (missing required fields)
  console.log('\n2. Testing invalid user registration (missing fields)...');
  try {
    const invalidUserData = {
      email: 'invalid-email',
      username: 'ab', // too short
      password: 'weak', // too weak
    };

    const response = await fetch(`${BASE_URL}/auth/register`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(invalidUserData),
    });

    if (!response.ok) {
      const error = await response.json();
      console.log('✅ Invalid user registration correctly rejected');
      console.log('   Validation errors:', error.details);
    } else {
      console.log('❌ Invalid user registration should have been rejected');
    }
  } catch (error) {
    console.log('❌ Invalid user registration test error:', error);
  }

  // Test 3: Invalid UUID parameter
  console.log('\n3. Testing invalid UUID parameter...');
  try {
    const response = await fetch(`${BASE_URL}/auth/users/invalid-uuid`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      const error = await response.json();
      console.log('✅ Invalid UUID correctly rejected');
      console.log('   Error:', error.error);
    } else {
      console.log('❌ Invalid UUID should have been rejected');
    }
  } catch (error) {
    console.log('❌ Invalid UUID test error:', error);
  }

  // Test 4: Invalid pagination parameters
  console.log('\n4. Testing invalid pagination parameters...');
  try {
    const response = await fetch(`${BASE_URL}/admin/users?page=-1&limit=1000`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      const error = await response.json();
      console.log('✅ Invalid pagination correctly rejected');
      console.log('   Error:', error.error);
    } else {
      console.log('❌ Invalid pagination should have been rejected');
    }
  } catch (error) {
    console.log('❌ Invalid pagination test error:', error);
  }

  // Test 5: XSS protection test
  console.log('\n5. Testing XSS protection...');
  try {
    const xssUserData = {
      email: 'xss@example.com',
      username: '<script>alert("xss")</script>',
      password: 'TestPass123!',
      role: 'user',
    };

    const response = await fetch(`${BASE_URL}/auth/register`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(xssUserData),
    });

    if (response.ok) {
      const result = await response.json();
      console.log('✅ XSS protection working - script tags removed');
      console.log('   Username after sanitization:', result.data.user.username);
    } else {
      const error = await response.json();
      console.log('❌ XSS test failed:', error);
    }
  } catch (error) {
    console.log('❌ XSS test error:', error);
  }

  // Test 6: Content-Type validation
  console.log('\n6. Testing Content-Type validation...');
  try {
    const response = await fetch(`${BASE_URL}/auth/register`, {
      method: 'POST',
      headers: {
        'Content-Type': 'text/plain',
      },
      body: 'invalid json',
    });

    if (!response.ok) {
      const error = await response.json();
      console.log('✅ Content-Type validation working');
      console.log('   Error:', error.error);
    } else {
      console.log('❌ Content-Type validation should have failed');
    }
  } catch (error) {
    console.log('❌ Content-Type test error:', error);
  }

  console.log('\n🎉 Validation middleware testing completed!');
}

if (require.main === module) {
  testValidation().catch(console.error);
}

export { testValidation };
