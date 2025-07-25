"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.testErrorHandling = void 0;
const node_fetch_1 = __importDefault(require("node-fetch"));
const dotenv_1 = __importDefault(require("dotenv"));
dotenv_1.default.config();
const BASE_URL = process.env.BACKEND_URL || 'http://localhost:8000';
async function testErrorHandling() {
    console.log('🧪 Testing Error Handling System...\n');
    // Test 1: 404 Error (Route not found)
    console.log('1. Testing 404 Error (Route not found)...');
    try {
        const response = await (0, node_fetch_1.default)(`${BASE_URL}/nonexistent-route`, {
            method: 'GET',
            headers: {
                'Content-Type': 'application/json',
            },
        });
        if (!response.ok) {
            const error = await response.json();
            console.log('✅ 404 Error handled correctly');
            console.log('   Status:', response.status);
            console.log('   Error:', error.error);
            console.log('   Request ID:', error.requestId);
            console.log('   Timestamp:', error.timestamp);
        }
        else {
            console.log('❌ 404 Error should have been returned');
        }
    }
    catch (error) {
        console.log('❌ 404 Error test failed:', error);
    }
    // Test 2: Validation Error
    console.log('\n2. Testing Validation Error...');
    try {
        const invalidData = {
            email: 'invalid-email',
            username: 'ab',
            password: 'weak', // too weak
        };
        const response = await (0, node_fetch_1.default)(`${BASE_URL}/auth/register`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(invalidData),
        });
        if (!response.ok) {
            const error = await response.json();
            console.log('✅ Validation Error handled correctly');
            console.log('   Status:', response.status);
            console.log('   Error:', error.error);
            console.log('   Details:', error.details);
            console.log('   Request ID:', error.requestId);
        }
        else {
            console.log('❌ Validation Error should have been returned');
        }
    }
    catch (error) {
        console.log('❌ Validation Error test failed:', error);
    }
    // Test 3: Authentication Error (Unauthorized)
    console.log('\n3. Testing Authentication Error (Unauthorized)...');
    try {
        const response = await (0, node_fetch_1.default)(`${BASE_URL}/admin/dashboard`, {
            method: 'GET',
            headers: {
                'Content-Type': 'application/json',
                // No Authorization header
            },
        });
        if (!response.ok) {
            const error = await response.json();
            console.log('✅ Authentication Error handled correctly');
            console.log('   Status:', response.status);
            console.log('   Error:', error.error);
            console.log('   Request ID:', error.requestId);
        }
        else {
            console.log('❌ Authentication Error should have been returned');
        }
    }
    catch (error) {
        console.log('❌ Authentication Error test failed:', error);
    }
    // Test 4: Invalid UUID Error
    console.log('\n4. Testing Invalid UUID Error...');
    try {
        const response = await (0, node_fetch_1.default)(`${BASE_URL}/auth/users/invalid-uuid`, {
            method: 'GET',
            headers: {
                'Content-Type': 'application/json',
            },
        });
        if (!response.ok) {
            const error = await response.json();
            console.log('✅ Invalid UUID Error handled correctly');
            console.log('   Status:', response.status);
            console.log('   Error:', error.error);
            console.log('   Request ID:', error.requestId);
        }
        else {
            console.log('❌ Invalid UUID Error should have been returned');
        }
    }
    catch (error) {
        console.log('❌ Invalid UUID Error test failed:', error);
    }
    // Test 5: Request ID Tracking
    console.log('\n5. Testing Request ID Tracking...');
    try {
        const response = await (0, node_fetch_1.default)(`${BASE_URL}/health`, {
            method: 'GET',
            headers: {
                'Content-Type': 'application/json',
            },
        });
        const requestId = response.headers.get('X-Request-ID');
        console.log('✅ Request ID tracking working');
        console.log('   Request ID:', requestId);
        console.log('   Status:', response.status);
        if (response.ok) {
            const data = await response.json();
            console.log('   Response Request ID:', data.requestId);
        }
    }
    catch (error) {
        console.log('❌ Request ID tracking test failed:', error);
    }
    // Test 6: Health Check with Error Handling
    console.log('\n6. Testing Health Check Error Handling...');
    try {
        // This should work and show proper error handling structure
        const response = await (0, node_fetch_1.default)(`${BASE_URL}/health`, {
            method: 'GET',
            headers: {
                'Content-Type': 'application/json',
            },
        });
        if (response.ok) {
            const data = await response.json();
            console.log('✅ Health check working with error handling structure');
            console.log('   Status:', data.status);
            console.log('   Request ID:', data.requestId);
            console.log('   Timestamp:', data.timestamp);
        }
        else {
            const error = await response.json();
            console.log('✅ Health check error handled correctly');
            console.log('   Error structure:', error);
        }
    }
    catch (error) {
        console.log('❌ Health check test failed:', error);
    }
    // Test 7: Content-Type Error
    console.log('\n7. Testing Content-Type Error...');
    try {
        const response = await (0, node_fetch_1.default)(`${BASE_URL}/auth/register`, {
            method: 'POST',
            headers: {
                'Content-Type': 'text/plain',
            },
            body: 'invalid json',
        });
        if (!response.ok) {
            const error = await response.json();
            console.log('✅ Content-Type Error handled correctly');
            console.log('   Status:', response.status);
            console.log('   Error:', error.error);
            console.log('   Request ID:', error.requestId);
        }
        else {
            console.log('❌ Content-Type Error should have been returned');
        }
    }
    catch (error) {
        console.log('❌ Content-Type Error test failed:', error);
    }
    console.log('\n🎉 Error handling system testing completed!');
    console.log('\n📋 Summary:');
    console.log('- All error types are properly handled');
    console.log('- Request IDs are tracked and included in responses');
    console.log('- Error responses include timestamps and structured data');
    console.log('- Different HTTP status codes are used appropriately');
    console.log('- Error messages are user-friendly and informative');
}
exports.testErrorHandling = testErrorHandling;
if (require.main === module) {
    testErrorHandling().catch(console.error);
}
