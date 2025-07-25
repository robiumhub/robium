#!/usr/bin/env ts-node

import { Database } from '../utils/database';
import { UserModel } from '../models/User';
import { UserRole } from '../types';
import dotenv from 'dotenv';

// Load environment variables
dotenv.config();

async function testUserModel() {
  console.log('🧪 Testing User Model Implementation...\n');

  try {
    // Connect to database
    console.log('📡 Connecting to database...');
    await Database.connect();
    console.log('✅ Database connected successfully\n');

    // Test 1: Create a new user
    console.log('1️⃣ Testing user creation...');
    const timestamp = Date.now();
    const newUser = await UserModel.create({
      email: `test${timestamp}@example.com`,
      username: `testuser${timestamp}`,
      password: 'TestPassword123!',
      role: UserRole.USER
    });
    console.log('✅ User created:', {
      id: newUser.id,
      email: newUser.email,
      username: newUser.username,
      role: newUser.role
    });
    console.log('');

    // Test 2: Find user by ID
    console.log('2️⃣ Testing find user by ID...');
    const foundUser = await UserModel.findById(newUser.id);
    console.log('✅ User found by ID:', foundUser ? 'Success' : 'Failed');
    console.log('');

    // Test 3: Find user by email
    console.log('3️⃣ Testing find user by email...');
    const foundByEmail = await UserModel.findByEmail(`test${timestamp}@example.com`);
    console.log('✅ User found by email:', foundByEmail ? 'Success' : 'Failed');
    console.log('');

    // Test 4: Update user
    console.log('4️⃣ Testing user update...');
    const updatedUser = await UserModel.update(newUser.id, {
      username: `updateduser${timestamp}`
    });
    console.log('✅ User updated:', {
      id: updatedUser.id,
      username: updatedUser.username
    });
    console.log('');

    // Test 5: Test password validation
    console.log('5️⃣ Testing password validation...');
    const userWithPassword = await UserModel.findByEmailWithPassword(`test${timestamp}@example.com`);
    if (userWithPassword) {
      const isValid = await UserModel.validatePassword('TestPassword123!', userWithPassword.password_hash);
      console.log('✅ Password validation:', isValid ? 'Success' : 'Failed');
    }
    console.log('');

    // Test 6: Test pagination
    console.log('6️⃣ Testing user listing with pagination...');
    const userList = await UserModel.findAll(1, 5);
    console.log('✅ User list retrieved:', {
      count: userList.users.length,
      total: userList.pagination.total,
      page: userList.pagination.page
    });
    console.log('');

    // Test 7: Test existence checks
    console.log('7️⃣ Testing existence checks...');
    const emailExists = await UserModel.existsByEmail(`test${timestamp}@example.com`);
    const usernameExists = await UserModel.existsByUsername(`updateduser${timestamp}`);
    console.log('✅ Existence checks:', {
      emailExists,
      usernameExists
    });
    console.log('');

    // Test 8: Test role statistics
    console.log('8️⃣ Testing role statistics...');
    const roleStats = await UserModel.getCountByRole();
    console.log('✅ Role statistics:', roleStats);
    console.log('');

    // Test 9: Clean up - delete test user
    console.log('9️⃣ Cleaning up test user...');
    await UserModel.delete(newUser.id);
    console.log('✅ Test user deleted successfully');
    console.log('');

    console.log('🎉 All User Model tests passed successfully!');

  } catch (error) {
    console.error('❌ Test failed:', error);
    
    if (error instanceof Error) {
      console.error('Error message:', error.message);
      console.error('Error stack:', error.stack);
    }
  } finally {
    // Close database connection
    try {
      await Database.disconnect();
      console.log('📡 Database disconnected');
    } catch (error) {
      console.error('Error disconnecting from database:', error);
    }
  }
}

// Run tests if called directly
if (require.main === module) {
  testUserModel().catch(console.error);
}

export { testUserModel }; 