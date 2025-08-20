import { environmentVariableService } from '../services/EnvironmentVariableService';
import { dockerfileGenerationService } from '../services/DockerfileGenerationService';
import { dockerComposeGenerationService } from '../services/DockerComposeGenerationService';
import { logger } from '../utils/logger';

async function testEnvironmentVariableSupport() {
  console.log('🧪 Testing Environment Variable Support Integration\n');

  try {
    // Test 1: Create environment configuration
    console.log('1. Creating environment configuration...');
    const configId = await environmentVariableService.createEnvironmentConfig(
      'test-project-123',
      'test-user-456',
      'Test Environment',
      'development',
      {
        DATABASE_URL: {
          name: 'DATABASE_URL',
          value: 'postgresql://user:password@localhost:5432/testdb',
          type: 'string',
          description: 'Database connection URL',
          required: true,
          environment: 'development'
        },
        API_KEY: {
          name: 'API_KEY',
          value: 'secret-api-key-123',
          type: 'string',
          description: 'API authentication key',
          required: true,
          sensitive: true,
          environment: 'development'
        },
        DEBUG_MODE: {
          name: 'DEBUG_MODE',
          value: true,
          type: 'boolean',
          description: 'Enable debug mode',
          required: false,
          environment: 'development'
        },
        MAX_CONNECTIONS: {
          name: 'MAX_CONNECTIONS',
          value: 100,
          type: 'number',
          description: 'Maximum database connections',
          required: false,
          environment: 'development'
        }
      }
    );

    console.log(`✅ Environment configuration created: ${configId.id}`);

    // Test 2: Generate environment injection
    console.log('\n2. Generating environment injection...');
    const injection = environmentVariableService.generateEnvironmentInjection(configId.id, 'development');
    
    console.log(`✅ Environment injection generated:`);
    console.log(`   - Variables: ${Object.keys(injection.variables).length}`);
    console.log(`   - Secrets: ${Object.keys(injection.secrets).length}`);
    console.log(`   - Env files: ${injection.envFiles.length}`);
    console.log(`   - Docker secrets: ${injection.dockerSecrets.length}`);

    // Test 3: Create Dockerfile configuration with environment variables
    console.log('\n3. Creating Dockerfile configuration with environment variables...');
    const dockerfileConfig = {
      id: 'test-dockerfile-123',
      name: 'Test Python App',
      version: '1.0.0',
      description: 'Test application with environment variables',
      type: 'python' as const,
      baseImage: 'python:3.9-slim',
      workdir: '/app',
      systemDependencies: ['curl', 'wget'],
      pythonDependencies: ['flask', 'requests'],
      environmentVariables: injection.variables,
      ports: ['8080'],
      command: 'python app.py'
    };

    dockerfileGenerationService.registerProjectConfig(dockerfileConfig);
    console.log('✅ Dockerfile configuration registered');

    // Test 4: Generate Dockerfile with environment variables
    console.log('\n4. Generating Dockerfile with environment variables...');
    const dockerfileResult = await dockerfileGenerationService.generateDockerfile(
      'test-dockerfile-123',
      {
        template: 'python',
        outputPath: './test-output',
        validateOnly: false,
        optimize: true,
        includeComments: true
      }
    );

    console.log(`✅ Dockerfile generated: ${dockerfileResult.path}`);
    console.log(`   - Size: ${dockerfileResult.content.length} characters`);
    console.log(`   - Errors: ${dockerfileResult.errors.length}`);
    console.log(`   - Warnings: ${dockerfileResult.warnings.length}`);

    // Test 5: Create Docker Compose configuration with environment variables
    console.log('\n5. Creating Docker Compose configuration with environment variables...');
    const composeConfig = {
      id: 'test-compose-123',
      name: 'Test Compose App',
      version: '1.0.0',
      description: 'Test docker-compose with environment variables',
      environment: 'development' as const,
      variables: injection.variables,
      services: [
        {
          name: 'app',
          build: {
            context: '.',
            dockerfile: 'Dockerfile'
          },
          ports: ['8080:8080'],
          environment: injection.variables,
          env_file: injection.envFiles,
          volumes: ['./src:/app/src'],
          networks: ['app-network']
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
          name: 'app-data',
          driver: 'local'
        }
      ]
    };

    dockerComposeGenerationService.registerComposeConfig(composeConfig);
    console.log('✅ Docker Compose configuration registered');

    // Test 6: Generate Docker Compose with environment variables
    console.log('\n6. Generating Docker Compose with environment variables...');
    const composeResult = await dockerComposeGenerationService.generateCompose(
      'test-compose-123',
      {
        template: 'development',
        outputPath: './test-output',
        validateOnly: false,
        includeComments: true
      }
    );

    console.log(`✅ Docker Compose generated: ${composeResult.path}`);
    console.log(`   - Size: ${composeResult.content.length} characters`);
    console.log(`   - Errors: ${composeResult.errors.length}`);
    console.log(`   - Warnings: ${composeResult.warnings.length}`);

    // Test 7: Validate environment configuration
    console.log('\n7. Validating environment configuration...');
    const validation = environmentVariableService.validateEnvironmentConfig(configId);
    
    console.log(`✅ Environment validation:`);
    console.log(`   - Valid: ${validation.isValid}`);
    console.log(`   - Errors: ${validation.errors.length}`);
    console.log(`   - Warnings: ${validation.warnings.length}`);
    console.log(`   - Missing required: ${validation.missingRequired.length}`);
    console.log(`   - Security issues: ${validation.securityIssues.length}`);

    // Test 8: Get service statistics
    console.log('\n8. Getting service statistics...');
    const stats = environmentVariableService.getServiceStats();
    
    console.log(`✅ Service statistics:`);
    console.log(`   - Total configs: ${stats.totalConfigs}`);
    console.log(`   - Total variables: ${stats.totalVariables}`);
    console.log(`   - Total secrets: ${stats.totalSecrets}`);
    console.log(`   - Encrypted variables: ${stats.encryptedVariables}`);
    console.log(`   - Validation errors: ${stats.validationErrors}`);

    // Test 9: Test environment variable inheritance
    console.log('\n9. Testing environment variable inheritance...');
    
    // Create a base configuration
    const baseConfigId = await environmentVariableService.createEnvironmentConfig(
      'test-project-123',
      'test-user-456',
      'Base Environment',
      'development',
      {
        BASE_URL: {
          name: 'BASE_URL',
          value: 'https://api.example.com',
          type: 'string',
          description: 'Base API URL',
          required: true,
          environment: 'development'
        }
      }
    );

    // Create a derived configuration that inherits from base
    const derivedConfigId = await environmentVariableService.createEnvironmentConfig(
      'test-project-123',
      'test-user-456',
      'Derived Environment',
      'development',
      {
        API_VERSION: {
          name: 'API_VERSION',
          value: 'v1',
          type: 'string',
          description: 'API version',
          required: true,
          environment: 'development'
        }
      }
    );

    // Update derived config to inherit from base
    await environmentVariableService.updateEnvironmentConfig(derivedConfigId.id, {
      inheritedConfigs: [baseConfigId.id]
    });

    console.log(`✅ Inheritance test completed:`);
    console.log(`   - Base config: ${baseConfigId.id}`);
    console.log(`   - Derived config: ${derivedConfigId.id}`);

    // Test 10: Test secrets management
    console.log('\n10. Testing secrets management...');
    
    await environmentVariableService.createSecret(
      configId.id,
      'DATABASE_PASSWORD',
      'super-secret-password-123',
      {
        lastRotated: new Date().toISOString(),
        expiresAt: new Date(Date.now() + 30 * 24 * 60 * 60 * 1000).toISOString() // 30 days
      }
    );

    const secretValue = environmentVariableService.getSecret(configId.id, 'DATABASE_PASSWORD');
    console.log(`✅ Secret management test:`);
    console.log(`   - Secret created and retrieved: ${secretValue ? '✅' : '❌'}`);
    console.log(`   - Secret value: ${secretValue ? '***' : 'undefined'}`);

    console.log('\n🎉 All environment variable support tests completed successfully!');

  } catch (error) {
    console.error('❌ Test failed:', error);
    logger.error('Environment variable support test failed:', error as Record<string, unknown>);
  }
}

// Run the test
testEnvironmentVariableSupport().catch(console.error); 