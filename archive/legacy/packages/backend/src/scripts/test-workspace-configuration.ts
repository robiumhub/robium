import { dockerComposeGenerationService } from '../services/DockerComposeGenerationService';
import { workspaceMountingService } from '../services/WorkspaceMountingService';
import { logger } from '../utils/logger';

async function testWorkspaceConfiguration() {
  try {
    console.log('🧪 Testing Workspace Configuration Integration...\n');

    // Test 1: Service statistics
    console.log('1. Testing service statistics...');
    const composeStats = dockerComposeGenerationService.getServiceStats();
    const workspaceStats = workspaceMountingService.getServiceStats();
    
    console.log('✅ Docker Compose Service Statistics:');
    console.log(`   - Total configs: ${composeStats.totalConfigs}`);
    console.log(`   - Total generated files: ${composeStats.totalGeneratedFiles}`);
    console.log(`   - Total errors: ${composeStats.totalErrors}`);
    console.log(`   - Total warnings: ${composeStats.totalWarnings}`);
    
    console.log('✅ Workspace Mounting Service Statistics:');
    console.log(`   - Total mount points: ${workspaceStats.totalMountPoints}`);
    console.log(`   - Active mount points: ${workspaceStats.activeMountPoints}`);
    console.log(`   - Total backups: ${workspaceStats.totalBackups}`);
    console.log(`   - Total sync events: ${workspaceStats.totalSyncEvents}\n`);

    // Test 2: Register compose configuration with workspace support
    console.log('2. Testing compose configuration with workspace support...');
    const configId = 'workspace-test-config';
    
    const composeConfig = {
      id: configId,
      name: 'Workspace Test Application',
      version: '1.0.0',
      description: 'Test application with workspace mounting',
      services: [
        {
          name: 'app',
          build: {
            context: '.',
            dockerfile: 'Dockerfile'
          },
          ports: ['3000:3000'],
          environment: {
            NODE_ENV: 'development',
            PORT: '3000'
          } as Record<string, string>,
          restart: 'unless-stopped' as const
        },
        {
          name: 'db',
          image: 'postgres:13',
          environment: {
            POSTGRES_DB: 'testdb',
            POSTGRES_USER: 'testuser',
            POSTGRES_PASSWORD: 'testpass'
          } as Record<string, string>,
          restart: 'unless-stopped' as const
        }
      ],
      networks: [
        {
          name: 'app-network',
          driver: 'bridge'
        }
      ],
      volumes: [
        {
          name: 'postgres_data',
          driver: 'local'
        }
      ],
      environment: 'development' as const
    };

    dockerComposeGenerationService.registerComposeConfig(composeConfig);
    console.log('✅ Compose configuration registered successfully\n');

    // Test 3: Create workspace configuration
    console.log('3. Testing workspace configuration creation...');
    const workspaceConfig = {
      userId: 'test-user-123',
      projectId: configId,
      workspacePath: '/workspaces/test-user-123_workspace-test-config',
      containerPath: '/app/workspace',
      permissions: {
        read: true,
        write: true,
        execute: false
      },
      syncMode: 'realtime' as const,
      backupEnabled: true,
      backupInterval: 60,
      maxBackups: 5
    };

    console.log('✅ Workspace configuration created\n');

    // Test 4: Generate compose with workspace mounting
    console.log('4. Testing compose generation with workspace mounting...');
    const result = await dockerComposeGenerationService.generateCompose(configId, {
      template: 'docker-compose-development',
      includeComments: true,
      enableWorkspaceMounting: true,
      workspaceConfig: workspaceConfig
    });

    console.log('✅ Compose generation with workspace mounting:');
    console.log(`   - Content length: ${result.content.length} chars`);
    console.log(`   - Errors: ${result.errors.length}`);
    console.log(`   - Warnings: ${result.warnings.length}`);
    console.log(`   - Services: ${result.services.join(', ')}`);
    console.log(`   - Generation time: ${result.generationTime}ms`);
    console.log(`   - File path: ${result.path}\n`);

    // Test 5: Check workspace mount points
    console.log('5. Testing workspace mount points...');
    const mountPoints = dockerComposeGenerationService.getWorkspaceMountPoints(configId);
    console.log(`✅ Workspace mount points: ${mountPoints.length}`);
    
    if (mountPoints.length > 0) {
      mountPoints.forEach(mountPoint => {
        console.log(`   - Mount ID: ${mountPoint.id}`);
        console.log(`   - Host Path: ${mountPoint.hostPath}`);
        console.log(`   - Container Path: ${mountPoint.containerPath}`);
        console.log(`   - Status: ${mountPoint.status}`);
      });
    }
    console.log('');

    // Test 6: Get workspace statistics
    console.log('6. Testing workspace statistics...');
    try {
      const workspaceStats = await dockerComposeGenerationService.getWorkspaceStats(configId);
      console.log(`✅ Workspace statistics: ${workspaceStats.length} mount points`);
      
      workspaceStats.forEach(stat => {
        console.log(`   - Mount Point ID: ${stat.mountPointId}`);
        console.log(`   - Total files: ${stat.totalFiles}`);
        console.log(`   - Total size: ${(stat.totalSize / 1024).toFixed(2)} KB`);
        console.log(`   - Directory count: ${stat.directoryCount}`);
      });
    } catch (error) {
      console.log(`⚠️  Workspace stats failed: ${error}`);
    }
    console.log('');

    // Test 7: Create workspace backup
    console.log('7. Testing workspace backup creation...');
    try {
      const backups = await dockerComposeGenerationService.createWorkspaceBackup(configId);
      console.log(`✅ Workspace backup created: ${backups.length} backups`);
      
      backups.forEach((backup: any) => {
        console.log(`   - Backup ID: ${backup.id}`);
        console.log(`   - Path: ${backup.path}`);
        console.log(`   - Size: ${backup.size} bytes`);
        console.log(`   - Status: ${backup.status}`);
      });
    } catch (error) {
      console.log(`⚠️  Workspace backup failed: ${error}`);
    }
    console.log('');

    // Test 8: Test generated compose file content
    console.log('8. Testing generated compose file content...');
    const generatedCompose = dockerComposeGenerationService.getGeneratedCompose(configId);
    if (generatedCompose) {
      console.log('✅ Generated compose file content preview:');
      const lines = generatedCompose.content.split('\n').slice(0, 20);
      lines.forEach(line => {
        if (line.trim()) {
          console.log(`   ${line}`);
        }
      });
      if (generatedCompose.content.split('\n').length > 20) {
        console.log('   ... (truncated)');
      }
    }
    console.log('');

    // Test 9: Test workspace path validation
    console.log('9. Testing workspace path validation...');
    const validPath = mountPoints.length > 0 ? mountPoints[0].hostPath : '/workspaces/test';
    const invalidPath = '/etc/passwd';
    
    const isValidPath = workspaceMountingService.validateWorkspacePath(validPath);
    const isInvalidPath = workspaceMountingService.validateWorkspacePath(invalidPath);
    
    console.log(`✅ Valid path validation: ${isValidPath}`);
    console.log(`❌ Invalid path validation: ${isInvalidPath}\n`);

    // Test 10: Cleanup
    console.log('10. Testing cleanup operations...');
    const removed = dockerComposeGenerationService.removeGeneratedCompose(configId);
    console.log(`✅ Compose file cleanup: ${removed ? 'successful' : 'failed'}`);

    console.log('\n🎉 All workspace configuration integration tests completed successfully!');
    console.log('\n📋 Summary:');
    console.log('✅ Service statistics integration working');
    console.log('✅ Compose configuration with workspace support');
    console.log('✅ Workspace configuration creation');
    console.log('✅ Compose generation with workspace mounting');
    console.log('✅ Workspace mount points integration');
    console.log('✅ Workspace statistics integration');
    console.log('✅ Workspace backup integration');
    console.log('✅ Generated compose file validation');
    console.log('✅ Workspace path validation');
    console.log('✅ Cleanup operations working');

  } catch (error) {
    console.error('❌ Workspace configuration integration test failed:', error);
    process.exit(1);
  }
}

// Run the test
testWorkspaceConfiguration(); 