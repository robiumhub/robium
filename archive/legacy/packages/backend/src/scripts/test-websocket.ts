#!/usr/bin/env ts-node

import WebSocket from 'ws';
import fetch from 'node-fetch';
import dotenv from 'dotenv';

// Load environment variables
dotenv.config();

const BASE_URL = 'http://localhost:8000';
const WS_URL = 'ws://localhost:8000/ws';

interface ApiResponse<T = unknown> {
  success: boolean;
  message?: string;
  data?: T;
  error?: string;
}

async function testWebSocket() {
  console.log('🧪 Testing WebSocket Server...\n');

  let authToken: string = '';

  try {
    // Step 1: Create a test user and get authentication token
    console.log('1️⃣ Creating test user for WebSocket authentication...');
    const signupResponse = await fetch(`${BASE_URL}/auth/signup`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email: `wsuser${Date.now()}@example.com`,
        username: `wsuser${Date.now()}`,
        password: 'WebSocketTest123!',
        role: 'user',
      }),
    });

    const signupData: ApiResponse = await signupResponse.json();

    if (signupResponse.ok && signupData.success) {
      console.log('✅ Test user created successfully');
      authToken = signupData.data?.token || '';
    } else {
      console.log('❌ User creation failed:', signupData);
      return;
    }
    console.log('');

    // Step 2: Test WebSocket connection without authentication
    console.log('2️⃣ Testing WebSocket connection without authentication...');
    const wsUnauthenticated = new WebSocket(WS_URL);

    await new Promise<void>((resolve, reject) => {
      const timeout = setTimeout(() => {
        reject(new Error('Connection timeout'));
      }, 5000);

      wsUnauthenticated.on('open', () => {
        console.log('✅ WebSocket connection established');
        clearTimeout(timeout);
        resolve();
      });

      wsUnauthenticated.on('error', (error) => {
        console.log('❌ WebSocket connection error:', error.message);
        clearTimeout(timeout);
        reject(error);
      });
    });

    // Test sending message without authentication
    wsUnauthenticated.send(
      JSON.stringify({
        type: 'room_message',
        data: 'Hello from unauthenticated user',
        timestamp: Date.now(),
      })
    );

    // Wait for error response
    await new Promise<void>((resolve) => {
      wsUnauthenticated.on('message', (data) => {
        const message = JSON.parse(data.toString());
        if (message.type === 'error') {
          console.log(
            '✅ Correctly received authentication error:',
            message.data?.error
          );
          resolve();
        }
      });
    });

    wsUnauthenticated.close();
    console.log('');

    // Step 3: Test WebSocket connection with authentication
    console.log('3️⃣ Testing WebSocket connection with authentication...');
    const wsAuthenticated = new WebSocket(`${WS_URL}?token=${authToken}`);

    await new Promise<void>((resolve, reject) => {
      const timeout = setTimeout(() => {
        reject(new Error('Connection timeout'));
      }, 5000);

      wsAuthenticated.on('open', () => {
        console.log('✅ Authenticated WebSocket connection established');
        clearTimeout(timeout);
        resolve();
      });

      wsAuthenticated.on('error', (error) => {
        console.log(
          '❌ Authenticated WebSocket connection error:',
          error.message
        );
        clearTimeout(timeout);
        reject(error);
      });
    });

    // Wait for authentication success message
    await new Promise<void>((resolve) => {
      wsAuthenticated.on('message', (data) => {
        const message = JSON.parse(data.toString());
        if (message.type === 'authentication_success') {
          console.log(
            '✅ Authentication successful:',
            message.data?.user?.email
          );
          resolve();
        }
      });
    });

    console.log('');

    // Step 4: Test room joining and messaging
    console.log('4️⃣ Testing room functionality...');

    // Join a test room
    wsAuthenticated.send(
      JSON.stringify({
        type: 'join_room',
        data: 'test-room-1',
        timestamp: Date.now(),
      })
    );

    // Wait for room join confirmation
    await new Promise<void>((resolve) => {
      wsAuthenticated.on('message', (data) => {
        const message = JSON.parse(data.toString());
        if (
          message.type === 'room_user_joined' &&
          message.data?.roomId === 'test-room-1'
        ) {
          console.log('✅ Successfully joined room:', message.data?.roomId);
          resolve();
        }
      });
    });

    // Send a message to the room
    wsAuthenticated.send(
      JSON.stringify({
        type: 'room_message',
        roomId: 'test-room-1',
        data: 'Hello from authenticated user!',
        timestamp: Date.now(),
      })
    );

    // Wait for room message
    await new Promise<void>((resolve) => {
      wsAuthenticated.on('message', (data) => {
        const message = JSON.parse(data.toString());
        if (
          message.type === 'room_message' &&
          message.data?.roomId === 'test-room-1'
        ) {
          console.log('✅ Room message received:', message.data?.message);
          console.log('   - Sender:', message.data?.senderName);
          console.log('   - Room:', message.data?.roomId);
          resolve();
        }
      });
    });

    console.log('');

    // Step 5: Test user status updates
    console.log('5️⃣ Testing user status updates...');

    wsAuthenticated.send(
      JSON.stringify({
        type: 'user_status_update',
        data: 'away',
        timestamp: Date.now(),
      })
    );

    // Wait for status update broadcast
    await new Promise<void>((resolve) => {
      wsAuthenticated.on('message', (data) => {
        const message = JSON.parse(data.toString());
        if (
          message.type === 'user_status_update' &&
          message.data?.status === 'away'
        ) {
          console.log('✅ User status updated to:', message.data?.status);
          resolve();
        }
      });
    });

    console.log('');

    // Step 6: Test heartbeat functionality
    console.log('6️⃣ Testing heartbeat functionality...');

    wsAuthenticated.send(
      JSON.stringify({
        type: 'heartbeat',
        timestamp: Date.now(),
      })
    );

    // Wait for heartbeat response
    await new Promise<void>((resolve) => {
      wsAuthenticated.on('message', (data) => {
        const message = JSON.parse(data.toString());
        if (message.type === 'heartbeat') {
          console.log('✅ Heartbeat response received');
          resolve();
        }
      });
    });

    console.log('');

    // Step 7: Test room leaving
    console.log('7️⃣ Testing room leaving...');

    wsAuthenticated.send(
      JSON.stringify({
        type: 'leave_room',
        data: 'test-room-1',
        timestamp: Date.now(),
      })
    );

    // Wait for room leave confirmation
    await new Promise<void>((resolve) => {
      wsAuthenticated.on('message', (data) => {
        const message = JSON.parse(data.toString());
        if (
          message.type === 'room_user_left' &&
          message.data?.roomId === 'test-room-1'
        ) {
          console.log('✅ Successfully left room:', message.data?.roomId);
          resolve();
        }
      });
    });

    console.log('');

    // Step 8: Test connection closure
    console.log('8️⃣ Testing connection closure...');

    wsAuthenticated.close();

    // Wait a moment for cleanup
    await new Promise((resolve) => setTimeout(resolve, 1000));

    console.log('✅ WebSocket connection closed successfully');
    console.log('');

    console.log('🎉 All WebSocket tests completed successfully!');
    console.log('');
    console.log('📊 WebSocket Test Summary:');
    console.log('✅ WebSocket server is running and accepting connections');
    console.log('✅ Authentication is working correctly');
    console.log('✅ Room functionality (join/leave/message) is working');
    console.log('✅ User status updates are working');
    console.log('✅ Heartbeat mechanism is working');
    console.log('✅ Connection management is working properly');
    console.log('✅ Error handling for unauthenticated users is working');
  } catch (error) {
    console.error('❌ WebSocket test failed:', error);

    if (error instanceof Error) {
      console.error('Error message:', error.message);
    }
  }
}

// Run tests if called directly
if (require.main === module) {
  testWebSocket().catch(console.error);
}

export { testWebSocket };
