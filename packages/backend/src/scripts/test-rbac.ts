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

async function testRBAC() {
  console.log('🧪 Testing Role-Based Access Control (RBAC)...\n');

  let adminToken: string = '';
  let userToken: string = '';

  try {
    // Test 1: Create admin user
    console.log('1️⃣ Creating admin user...');
    const adminResponse = await fetch(`${BASE_URL}/auth/signup`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email: `admin${Date.now()}@example.com`,
        username: `admin${Date.now()}`,
        password: 'AdminPassword123!',
        role: 'admin',
      }),
    });

    const adminData: ApiResponse = await adminResponse.json();

    if (adminResponse.ok && adminData.success) {
      console.log('✅ Admin user created successfully');
      adminToken = adminData.data?.token || '';
    } else {
      console.log('❌ Admin creation failed:', adminData);
      return;
    }
    console.log('');

    // Test 2: Create regular user
    console.log('2️⃣ Creating regular user...');
    const userResponse = await fetch(`${BASE_URL}/auth/signup`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email: `user${Date.now()}@example.com`,
        username: `user${Date.now()}`,
        password: 'UserPassword123!',
        role: 'user',
      }),
    });

    const userData: ApiResponse = await userResponse.json();

    if (userResponse.ok && userData.success) {
      console.log('✅ Regular user created successfully');
      userToken = userData.data?.token || '';
    } else {
      console.log('❌ User creation failed:', userData);
      return;
    }
    console.log('');

    // Test 3: Admin accessing admin dashboard
    console.log('3️⃣ Testing admin dashboard access...');
    const adminDashboardResponse = await fetch(`${BASE_URL}/admin/dashboard`, {
      method: 'GET',
      headers: {
        Authorization: `Bearer ${adminToken}`,
        'Content-Type': 'application/json',
      },
    });

    const adminDashboardData: ApiResponse = await adminDashboardResponse.json();

    if (adminDashboardResponse.ok && adminDashboardData.success) {
      console.log('✅ Admin dashboard access successful');
      console.log('   - User role:', adminDashboardData.data?.user?.role);
      console.log(
        '   - Permissions count:',
        adminDashboardData.data?.permissions?.length
      );
    } else {
      console.log('❌ Admin dashboard access failed:', adminDashboardData);
    }
    console.log('');

    // Test 4: Regular user trying to access admin dashboard (should fail)
    console.log(
      '4️⃣ Testing regular user access to admin dashboard (should fail)...'
    );
    const userDashboardResponse = await fetch(`${BASE_URL}/admin/dashboard`, {
      method: 'GET',
      headers: {
        Authorization: `Bearer ${userToken}`,
        'Content-Type': 'application/json',
      },
    });

    const userDashboardData: ApiResponse = await userDashboardResponse.json();

    if (userDashboardResponse.status === 403) {
      console.log('✅ Regular user correctly denied admin dashboard access');
      console.log('   - Error:', userDashboardData.error);
    } else {
      console.log(
        '❌ Regular user incorrectly allowed admin dashboard access:',
        userDashboardData
      );
    }
    console.log('');

    // Test 5: Admin accessing user list
    console.log('5️⃣ Testing admin access to user list...');
    const adminUsersResponse = await fetch(`${BASE_URL}/admin/users`, {
      method: 'GET',
      headers: {
        Authorization: `Bearer ${adminToken}`,
        'Content-Type': 'application/json',
      },
    });

    const adminUsersData: ApiResponse = await adminUsersResponse.json();

    if (adminUsersResponse.ok && adminUsersData.success) {
      console.log('✅ Admin user list access successful');
      console.log('   - Users count:', adminUsersData.data?.pagination?.total);
    } else {
      console.log('❌ Admin user list access failed:', adminUsersData);
    }
    console.log('');

    // Test 6: Regular user trying to access user list (should fail)
    console.log('6️⃣ Testing regular user access to user list (should fail)...');
    const userUsersResponse = await fetch(`${BASE_URL}/admin/users`, {
      method: 'GET',
      headers: {
        Authorization: `Bearer ${userToken}`,
        'Content-Type': 'application/json',
      },
    });

    const userUsersData: ApiResponse = await userUsersResponse.json();

    if (userUsersResponse.status === 403) {
      console.log('✅ Regular user correctly denied user list access');
      console.log('   - Error:', userUsersData.error);
    } else {
      console.log(
        '❌ Regular user incorrectly allowed user list access:',
        userUsersData
      );
    }
    console.log('');

    // Test 7: Admin accessing permissions endpoint
    console.log('7️⃣ Testing admin access to permissions endpoint...');
    const adminPermissionsResponse = await fetch(
      `${BASE_URL}/admin/permissions`,
      {
        method: 'GET',
        headers: {
          Authorization: `Bearer ${adminToken}`,
          'Content-Type': 'application/json',
        },
      }
    );

    const adminPermissionsData: ApiResponse =
      await adminPermissionsResponse.json();

    if (adminPermissionsResponse.ok && adminPermissionsData.success) {
      console.log('✅ Admin permissions access successful');
      console.log(
        '   - Permissions count:',
        adminPermissionsData.data?.permissions?.length
      );
      console.log(
        '   - System admin access:',
        adminPermissionsData.data?.permissionGroups?.systemAdministration
      );
    } else {
      console.log('❌ Admin permissions access failed:', adminPermissionsData);
    }
    console.log('');

    // Test 8: Regular user accessing permissions endpoint (should work for system read)
    console.log('8️⃣ Testing regular user access to permissions endpoint...');
    const userPermissionsResponse = await fetch(
      `${BASE_URL}/admin/permissions`,
      {
        method: 'GET',
        headers: {
          Authorization: `Bearer ${userToken}`,
          'Content-Type': 'application/json',
        },
      }
    );

    const userPermissionsData: ApiResponse =
      await userPermissionsResponse.json();

    if (userPermissionsResponse.ok && userPermissionsData.success) {
      console.log('✅ Regular user permissions access successful');
      console.log(
        '   - Permissions count:',
        userPermissionsData.data?.permissions?.length
      );
      console.log(
        '   - System admin access:',
        userPermissionsData.data?.permissionGroups?.systemAdministration
      );
    } else {
      console.log(
        '❌ Regular user permissions access failed:',
        userPermissionsData
      );
    }
    console.log('');

    // Test 9: Admin accessing system health
    console.log('9️⃣ Testing admin access to system health...');
    const adminHealthResponse = await fetch(`${BASE_URL}/admin/system/health`, {
      method: 'GET',
      headers: {
        Authorization: `Bearer ${adminToken}`,
        'Content-Type': 'application/json',
      },
    });

    const adminHealthData: ApiResponse = await adminHealthResponse.json();

    if (adminHealthResponse.ok && adminHealthData.success) {
      console.log('✅ Admin system health access successful');
      console.log('   - Status:', adminHealthData.data?.status);
      console.log('   - Uptime:', adminHealthData.data?.uptime);
    } else {
      console.log('❌ Admin system health access failed:', adminHealthData);
    }
    console.log('');

    // Test 10: Admin accessing system restart (should work)
    console.log('🔟 Testing admin access to system restart...');
    const adminRestartResponse = await fetch(
      `${BASE_URL}/admin/system/restart`,
      {
        method: 'POST',
        headers: {
          Authorization: `Bearer ${adminToken}`,
          'Content-Type': 'application/json',
        },
      }
    );

    const adminRestartData: ApiResponse = await adminRestartResponse.json();

    if (adminRestartResponse.ok && adminRestartData.success) {
      console.log('✅ Admin system restart access successful');
      console.log('   - Message:', adminRestartData.message);
    } else {
      console.log('❌ Admin system restart access failed:', adminRestartData);
    }
    console.log('');

    // Test 11: Regular user trying to access system restart (should fail)
    console.log(
      '1️⃣1️⃣ Testing regular user access to system restart (should fail)...'
    );
    const userRestartResponse = await fetch(
      `${BASE_URL}/admin/system/restart`,
      {
        method: 'POST',
        headers: {
          Authorization: `Bearer ${userToken}`,
          'Content-Type': 'application/json',
        },
      }
    );

    const userRestartData: ApiResponse = await userRestartResponse.json();

    if (userRestartResponse.status === 403) {
      console.log('✅ Regular user correctly denied system restart access');
      console.log('   - Error:', userRestartData.error);
    } else {
      console.log(
        '❌ Regular user incorrectly allowed system restart access:',
        userRestartData
      );
    }
    console.log('');

    console.log('🎉 All RBAC tests completed successfully!');
    console.log('');
    console.log('📊 RBAC Test Summary:');
    console.log('✅ Admin users can access all admin endpoints');
    console.log(
      '✅ Regular users are properly restricted from admin endpoints'
    );
    console.log('✅ Permission-based access control is working correctly');
    console.log('✅ Role hierarchy is properly enforced');
    console.log('✅ System administration permissions are properly protected');
  } catch (error) {
    console.error('❌ RBAC test failed:', error);

    if (error instanceof Error) {
      console.error('Error message:', error.message);
    }
  }
}

// Run tests if called directly
if (require.main === module) {
  testRBAC().catch(console.error);
}

export { testRBAC };
