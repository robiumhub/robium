#!/usr/bin/env ts-node

import fetch from 'node-fetch';
import dotenv from 'dotenv';

// Load environment variables
dotenv.config();

const BASE_URL = 'http://localhost:8000';

interface ApiResponse<T = unknown> {
  success: boolean;
  message?: string;
  data?: T;
  error?: string;
}

async function testAuthEndpoints() {
  console.log('🧪 Testing Authentication Endpoints...\n');

  const timestamp = Date.now();
  const testUser = {
    email: `testauth${timestamp}@example.com`,
    username: `testauth${timestamp}`,
    password: 'TestPassword123!'
  };

  let authToken: string = '';

  try {
    // Test 1: User Registration
    console.log('1️⃣ Testing user registration (POST /auth/signup)...');
    const signupResponse = await fetch(`${BASE_URL}/auth/signup`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(testUser),
    });

    const signupData: ApiResponse = await signupResponse.json();
    
    if (signupResponse.ok && signupData.success) {
      console.log('✅ Registration successful:', {
        status: signupResponse.status,
        user: signupData.data?.user?.email,
        hasToken: !!signupData.data?.token
      });
      authToken = signupData.data?.token || '';
    } else {
      console.log('❌ Registration failed:', signupData);
      return;
    }
    console.log('');

    // Test 2: User Login
    console.log('2️⃣ Testing user login (POST /auth/login)...');
    const loginResponse = await fetch(`${BASE_URL}/auth/login`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email: testUser.email,
        password: testUser.password
      }),
    });

    const loginData: ApiResponse = await loginResponse.json();
    
    if (loginResponse.ok && loginData.success) {
      console.log('✅ Login successful:', {
        status: loginResponse.status,
        user: loginData.data?.user?.email,
        hasToken: !!loginData.data?.token,
        expiresIn: loginData.data?.expiresIn
      });
      authToken = loginData.data?.token || authToken; // Use login token
    } else {
      console.log('❌ Login failed:', loginData);
    }
    console.log('');

    // Test 3: Get Current User Profile
    console.log('3️⃣ Testing get current user (GET /auth/me)...');
    const meResponse = await fetch(`${BASE_URL}/auth/me`, {
      method: 'GET',
      headers: {
        'Authorization': `Bearer ${authToken}`,
        'Content-Type': 'application/json',
      },
    });

    const meData: ApiResponse = await meResponse.json();
    
    if (meResponse.ok && meData.success) {
      console.log('✅ Get profile successful:', {
        status: meResponse.status,
        user: {
          id: meData.data?.id,
          email: meData.data?.email,
          username: meData.data?.username,
          role: meData.data?.role
        }
      });
    } else {
      console.log('❌ Get profile failed:', meData);
    }
    console.log('');

    // Test 4: Token Verification
    console.log('4️⃣ Testing token verification (GET /auth/verify)...');
    const verifyResponse = await fetch(`${BASE_URL}/auth/verify`, {
      method: 'GET',
      headers: {
        'Authorization': `Bearer ${authToken}`,
        'Content-Type': 'application/json',
      },
    });

    const verifyData: ApiResponse = await verifyResponse.json();
    
    if (verifyResponse.ok && verifyData.success) {
      console.log('✅ Token verification successful:', {
        status: verifyResponse.status,
        valid: verifyData.data?.valid,
        userId: verifyData.data?.user?.userId
      });
    } else {
      console.log('❌ Token verification failed:', verifyData);
    }
    console.log('');

    // Test 5: Invalid Token Test
    console.log('5️⃣ Testing invalid token handling...');
    const invalidTokenResponse = await fetch(`${BASE_URL}/auth/me`, {
      method: 'GET',
      headers: {
        'Authorization': 'Bearer invalid_token_here',
        'Content-Type': 'application/json',
      },
    });

    const invalidTokenData: ApiResponse = await invalidTokenResponse.json();
    
    if (invalidTokenResponse.status === 401) {
      console.log('✅ Invalid token properly rejected:', {
        status: invalidTokenResponse.status,
        error: invalidTokenData.error
      });
    } else {
      console.log('❌ Invalid token not properly handled:', invalidTokenData);
    }
    console.log('');

    // Test 6: Missing Token Test
    console.log('6️⃣ Testing missing token handling...');
    const noTokenResponse = await fetch(`${BASE_URL}/auth/me`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    const noTokenData: ApiResponse = await noTokenResponse.json();
    
    if (noTokenResponse.status === 401) {
      console.log('✅ Missing token properly rejected:', {
        status: noTokenResponse.status,
        error: noTokenData.error
      });
    } else {
      console.log('❌ Missing token not properly handled:', noTokenData);
    }
    console.log('');

    // Test 7: User Logout
    console.log('7️⃣ Testing user logout (POST /auth/logout)...');
    const logoutResponse = await fetch(`${BASE_URL}/auth/logout`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${authToken}`,
        'Content-Type': 'application/json',
      },
    });

    const logoutData: ApiResponse = await logoutResponse.json();
    
    if (logoutResponse.ok && logoutData.success) {
      console.log('✅ Logout successful:', {
        status: logoutResponse.status,
        message: logoutData.message
      });
    } else {
      console.log('❌ Logout failed:', logoutData);
    }
    console.log('');

    // Test 8: Duplicate Registration Test
    console.log('8️⃣ Testing duplicate registration prevention...');
    const duplicateResponse = await fetch(`${BASE_URL}/auth/signup`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(testUser),
    });

    const duplicateData: ApiResponse = await duplicateResponse.json();
    
    if (duplicateResponse.status === 409) {
      console.log('✅ Duplicate registration properly prevented:', {
        status: duplicateResponse.status,
        error: duplicateData.error
      });
    } else {
      console.log('❌ Duplicate registration not properly handled:', duplicateData);
    }
    console.log('');

    // Test 9: Invalid Login Test
    console.log('9️⃣ Testing invalid login credentials...');
    const invalidLoginResponse = await fetch(`${BASE_URL}/auth/login`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email: testUser.email,
        password: 'WrongPassword123!'
      }),
    });

    const invalidLoginData: ApiResponse = await invalidLoginResponse.json();
    
    if (invalidLoginResponse.status === 401) {
      console.log('✅ Invalid login properly rejected:', {
        status: invalidLoginResponse.status,
        error: invalidLoginData.error
      });
    } else {
      console.log('❌ Invalid login not properly handled:', invalidLoginData);
    }
    console.log('');

    console.log('🎉 All Authentication Endpoint tests completed!');

  } catch (error) {
    console.error('❌ Test failed:', error);
    
    if (error instanceof Error) {
      console.error('Error message:', error.message);
    }
  }
}

// Run tests if called directly
if (require.main === module) {
  testAuthEndpoints().catch(console.error);
}

export { testAuthEndpoints }; 