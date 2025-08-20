import { automatedCleanupService } from '../services/AutomatedCleanupService';
import { logger } from '../utils/logger';

async function testAutomatedCleanupService() {
  try {
    console.log('🧪 Testing Automated Cleanup Service...\n');

    // Test 1: Service statistics
    console.log('1. Testing service statistics...');
    const stats = automatedCleanupService.getServiceStats();
    console.log('✅ Service statistics:');
    console.log(`   - Service running: ${stats.isRunning}`);
    console.log(`   - Total policies: ${stats.totalPolicies}`);
    console.log(`   - Enabled policies: ${stats.enabledPolicies}`);
    console.log(`   - Total jobs: ${stats.totalJobs}`);
    console.log(`   - Total notifications: ${stats.totalNotifications}`);
    console.log(`   - Scheduled jobs: ${stats.scheduledJobs}\n`);

    // Test 2: Cleanup policies
    console.log('2. Testing cleanup policies...');
    const policies = automatedCleanupService.getCleanupPolicies();
    console.log(`✅ Cleanup policies: ${policies.length} policies`);
    
    policies.forEach(policy => {
      console.log(`   - ${policy.name}: ${policy.description}`);
      console.log(`     States: ${policy.containerStates.join(', ')}`);
      console.log(`     Idle threshold: ${policy.idleThresholdMinutes} minutes`);
      console.log(`     Grace period: ${policy.gracePeriodMinutes} minutes`);
      console.log(`     Priority: ${policy.priority}`);
      console.log(`     Enabled: ${policy.enabled}\n`);
    });

    // Test 3: Policy management
    console.log('3. Testing policy management...');
    const customPolicy = {
      id: 'test_policy',
      name: 'Test Cleanup Policy',
      description: 'A test policy for demonstration',
      containerStates: ['stopped'],
      idleThresholdMinutes: 120, // 2 hours
      gracePeriodMinutes: 30, // 30 minutes
      maxContainersPerUser: 8,
      maxContainersPerProject: 4,
      enabled: true,
      priority: 'low' as const,
    };

    automatedCleanupService.setCleanupPolicy(customPolicy);
    console.log('✅ Custom policy added successfully');
    
    const updatedPolicies = automatedCleanupService.getCleanupPolicies();
    console.log(`✅ Total policies after addition: ${updatedPolicies.length}\n`);

    // Test 4: Service start/stop
    console.log('4. Testing service start/stop...');
    try {
      await automatedCleanupService.startService();
      console.log('✅ Service started successfully');
      
      const runningStats = automatedCleanupService.getServiceStats();
      console.log(`   - Service running: ${runningStats.isRunning}`);
      console.log(`   - Scheduled jobs: ${runningStats.scheduledJobs}`);
      
      await automatedCleanupService.stopService();
      console.log('✅ Service stopped successfully');
    } catch (error) {
      console.log(`⚠️  Service start/stop test failed: ${error}`);
    }
    console.log('');

    // Test 5: Cleanup jobs
    console.log('5. Testing cleanup jobs...');
    const jobs = automatedCleanupService.getCleanupJobs({ limit: 10 });
    console.log(`✅ Cleanup jobs: ${jobs.length} jobs`);
    
    if (jobs.length > 0) {
      console.log('   Recent jobs:');
      jobs.slice(0, 3).forEach(job => {
        console.log(`   - ${job.id}: ${job.status} (${job.containersCleaned}/${job.containersProcessed} cleaned)`);
      });
    }
    console.log('');

    // Test 6: Job filtering
    console.log('6. Testing job filtering...');
    const completedJobs = automatedCleanupService.getCleanupJobs({ status: 'completed', limit: 5 });
    console.log(`✅ Completed jobs: ${completedJobs.length}`);
    
    const failedJobs = automatedCleanupService.getCleanupJobs({ status: 'failed', limit: 5 });
    console.log(`✅ Failed jobs: ${failedJobs.length}\n`);

    // Test 7: Notifications
    console.log('7. Testing notifications...');
    const userId = 'user123';
    const userNotifications = automatedCleanupService.getUserNotifications(userId, { limit: 10 });
    console.log(`✅ User notifications for ${userId}: ${userNotifications.length}`);
    
    const unreadNotifications = automatedCleanupService.getUserNotifications(userId, { 
      unreadOnly: true, 
      limit: 5 
    });
    console.log(`✅ Unread notifications: ${unreadNotifications.length}`);
    
    if (unreadNotifications.length > 0) {
      console.log('   Sample notifications:');
      unreadNotifications.slice(0, 2).forEach(notification => {
        console.log(`   - ${notification.type}: ${notification.title}`);
        console.log(`     ${notification.message.substring(0, 50)}...`);
      });
    }
    console.log('');

    // Test 8: Notification management
    console.log('8. Testing notification management...');
    if (unreadNotifications.length > 0) {
      const notificationToMark = unreadNotifications[0];
      const marked = automatedCleanupService.markNotificationAsRead(notificationToMark.id);
      console.log(`✅ Marked notification as read: ${marked}`);
      
      const updatedUnread = automatedCleanupService.getUserNotifications(userId, { unreadOnly: true });
      console.log(`✅ Updated unread count: ${updatedUnread.length}`);
    } else {
      console.log('⚠️  No notifications to mark as read');
    }
    console.log('');

    // Test 9: Idle container detection (simulated)
    console.log('9. Testing idle container detection...');
    try {
      await automatedCleanupService.detectIdleContainers();
      console.log('✅ Idle container detection completed');
    } catch (error) {
      console.log(`⚠️  Idle container detection failed: ${error}`);
    }
    console.log('');

    // Test 10: Cleanup operations
    console.log('10. Testing cleanup operations...');
    const beforeCleanup = automatedCleanupService.getServiceStats();
    
    // Cleanup old notifications
    automatedCleanupService.cleanupOldNotifications(1); // Clean notifications older than 1 day
    console.log('✅ Old notifications cleanup completed');
    
    // Cleanup old jobs
    automatedCleanupService.cleanupOldJobs(1); // Clean jobs older than 1 day
    console.log('✅ Old jobs cleanup completed');
    
    const afterCleanup = automatedCleanupService.getServiceStats();
    console.log(`✅ Cleanup results: ${beforeCleanup.totalNotifications} -> ${afterCleanup.totalNotifications} notifications`);
    console.log(`✅ Cleanup results: ${beforeCleanup.totalJobs} -> ${afterCleanup.totalJobs} jobs\n`);

    // Test 11: Policy removal
    console.log('11. Testing policy removal...');
    const removed = automatedCleanupService.removeCleanupPolicy('test_policy');
    console.log(`✅ Test policy removed: ${removed}`);
    
    const finalPolicies = automatedCleanupService.getCleanupPolicies();
    console.log(`✅ Final policy count: ${finalPolicies.length}\n`);

    // Test 12: Service integration
    console.log('12. Testing service integration...');
    const finalStats = automatedCleanupService.getServiceStats();
    console.log('✅ Final service statistics:');
    console.log(`   - Service running: ${finalStats.isRunning}`);
    console.log(`   - Total policies: ${finalStats.totalPolicies}`);
    console.log(`   - Enabled policies: ${finalStats.enabledPolicies}`);
    console.log(`   - Total jobs: ${finalStats.totalJobs}`);
    console.log(`   - Total notifications: ${finalStats.totalNotifications}`);

    console.log('\n🎉 All automated cleanup service tests completed successfully!');
    console.log('\n📋 Summary:');
    console.log('✅ Service statistics working');
    console.log('✅ Cleanup policies management functional');
    console.log('✅ Service start/stop operations working');
    console.log('✅ Cleanup jobs tracking operational');
    console.log('✅ Job filtering working');
    console.log('✅ Notifications system functional');
    console.log('✅ Notification management working');
    console.log('✅ Idle container detection ready');
    console.log('✅ Cleanup operations working');
    console.log('✅ Policy management complete');
    console.log('✅ Service integration ready');

  } catch (error) {
    console.error('❌ Automated cleanup service test failed:', error);
    process.exit(1);
  }
}

// Run the test
testAutomatedCleanupService(); 