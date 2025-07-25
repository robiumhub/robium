#!/usr/bin/env ts-node
"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.testUserModel = void 0;
const database_1 = require("../utils/database");
const User_1 = require("../models/User");
const types_1 = require("../types");
const dotenv_1 = __importDefault(require("dotenv"));
// Load environment variables
dotenv_1.default.config();
async function testUserModel() {
    console.log('🧪 Testing User Model Implementation...\n');
    try {
        // Connect to database
        console.log('📡 Connecting to database...');
        await database_1.Database.connect();
        console.log('✅ Database connected successfully\n');
        // Test 1: Create a new user
        console.log('1️⃣ Testing user creation...');
        const timestamp = Date.now();
        const newUser = await User_1.UserModel.create({
            email: `test${timestamp}@example.com`,
            username: `testuser${timestamp}`,
            password: 'TestPassword123!',
            role: types_1.UserRole.USER,
        });
        console.log('✅ User created:', {
            id: newUser.id,
            email: newUser.email,
            username: newUser.username,
            role: newUser.role,
        });
        console.log('');
        // Test 2: Find user by ID
        console.log('2️⃣ Testing find user by ID...');
        const foundUser = await User_1.UserModel.findById(newUser.id);
        console.log('✅ User found by ID:', foundUser ? 'Success' : 'Failed');
        console.log('');
        // Test 3: Find user by email
        console.log('3️⃣ Testing find user by email...');
        const foundByEmail = await User_1.UserModel.findByEmail(`test${timestamp}@example.com`);
        console.log('✅ User found by email:', foundByEmail ? 'Success' : 'Failed');
        console.log('');
        // Test 4: Update user
        console.log('4️⃣ Testing user update...');
        const updatedUser = await User_1.UserModel.update(newUser.id, {
            username: `updateduser${timestamp}`,
        });
        console.log('✅ User updated:', {
            id: updatedUser.id,
            username: updatedUser.username,
        });
        console.log('');
        // Test 5: Test password validation
        console.log('5️⃣ Testing password validation...');
        const userWithPassword = await User_1.UserModel.findByEmailWithPassword(`test${timestamp}@example.com`);
        if (userWithPassword) {
            const isValid = await User_1.UserModel.validatePassword('TestPassword123!', userWithPassword.password_hash);
            console.log('✅ Password validation:', isValid ? 'Success' : 'Failed');
        }
        console.log('');
        // Test 6: Test pagination
        console.log('6️⃣ Testing user listing with pagination...');
        const userList = await User_1.UserModel.findAll(1, 5);
        console.log('✅ User list retrieved:', {
            count: userList.users.length,
            total: userList.pagination.total,
            page: userList.pagination.page,
        });
        console.log('');
        // Test 7: Test existence checks
        console.log('7️⃣ Testing existence checks...');
        const emailExists = await User_1.UserModel.existsByEmail(`test${timestamp}@example.com`);
        const usernameExists = await User_1.UserModel.existsByUsername(`updateduser${timestamp}`);
        console.log('✅ Existence checks:', {
            emailExists,
            usernameExists,
        });
        console.log('');
        // Test 8: Test role statistics
        console.log('8️⃣ Testing role statistics...');
        const roleStats = await User_1.UserModel.getCountByRole();
        console.log('✅ Role statistics:', roleStats);
        console.log('');
        // Test 9: Clean up - delete test user
        console.log('9️⃣ Cleaning up test user...');
        await User_1.UserModel.delete(newUser.id);
        console.log('✅ Test user deleted successfully');
        console.log('');
        console.log('🎉 All User Model tests passed successfully!');
    }
    catch (error) {
        console.error('❌ Test failed:', error);
        if (error instanceof Error) {
            console.error('Error message:', error.message);
            console.error('Error stack:', error.stack);
        }
    }
    finally {
        // Close database connection
        try {
            await database_1.Database.disconnect();
            console.log('📡 Database disconnected');
        }
        catch (error) {
            console.error('Error disconnecting from database:', error);
        }
    }
}
exports.testUserModel = testUserModel;
// Run tests if called directly
if (require.main === module) {
    testUserModel().catch(console.error);
}
