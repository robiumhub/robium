import { dockerComposeGenerationService, ComposeConfiguration, ComposeService, ComposeNetwork, ComposeVolume } from '../services/DockerComposeGenerationService';
import { logger } from '../utils/logger';

/**
 * Test docker-compose generation service functionality
 */
async function testDockerComposeGenerationService() {
  logger.info('Starting Docker Compose Generation Service tests...');

  try {
    // Test 1: Service Statistics
    await testServiceStatistics();

    // Test 2: Configuration Management
    await testConfigurationManagement();

    // Test 3: Basic Compose Generation
    await testBasicComposeGeneration();

    // Test 4: Multi-Service Compose Generation
    await testMultiServiceComposeGeneration();

    // Test 5: Environment-Specific Generation
    await testEnvironmentSpecificGeneration();

    // Test 6: Validation-Only Mode
    await testValidationOnlyMode();

    // Test 7: Generated File Management
    await testGeneratedFileManagement();

    // Test 8: Configuration Retrieval
    await testConfigurationRetrieval();

    // Test 9: Error Handling
    await testErrorHandling();

    // Test 10: Cleanup Operations
    await testCleanupOperations();

    logger.info('✅ All Docker Compose Generation Service tests completed successfully!');

  } catch (error) {
    logger.error('❌ Docker Compose Generation Service tests failed:', undefined, error as Error);
    process.exit(1);
  }
}

/**
 * Test service statistics
 */
async function testServiceStatistics() {
  logger.info('Testing service statistics...');

  const stats = dockerComposeGenerationService.getServiceStats();
  
  logger.info(`Service Statistics:`, {
    totalConfigs: stats.totalConfigs,
    totalGeneratedFiles: stats.totalGeneratedFiles,
    totalErrors: stats.totalErrors,
    totalWarnings: stats.totalWarnings,
  });

  logger.info('✅ Service statistics test passed');
}

/**
 * Test configuration management
 */
async function testConfigurationManagement() {
  logger.info('Testing configuration management...');

  // Create test configuration
  const testConfig: ComposeConfiguration = {
    id: 'test-config-1',
    name: 'Test Application',
    version: '1.0.0',
    description: 'Test application for docker-compose generation',
    environment: 'development',
    services: [
      {
        name: 'app',
        build: {
          context: '.',
          dockerfile: 'Dockerfile',
        },
        ports: ['3000:3000'],
        volumes: ['./src:/app/src'],
        environment: {
          NODE_ENV: 'development',
          PORT: '3000',
        },
        depends_on: ['db'],
        networks: ['app-network'],
        restart: 'unless-stopped',
      },
      {
        name: 'db',
        image: 'postgres:13',
        environment: {
          POSTGRES_DB: 'testdb',
          POSTGRES_USER: 'testuser',
          POSTGRES_PASSWORD: 'testpass',
        },
        volumes: ['postgres_data:/var/lib/postgresql/data'],
        networks: ['app-network'],
        restart: 'unless-stopped',
      },
    ],
    networks: [
      {
        name: 'app-network',
        driver: 'bridge',
      },
    ],
    volumes: [
      {
        name: 'postgres_data',
        driver: 'local',
      },
    ],
  };

  // Register configuration
  dockerComposeGenerationService.registerComposeConfig(testConfig);

  logger.info('✅ Configuration management test passed');
}

/**
 * Test basic compose generation
 */
async function testBasicComposeGeneration() {
  logger.info('Testing basic compose generation...');

  const result = await dockerComposeGenerationService.generateCompose('test-config-1', {
    includeComments: true,
  });

  logger.info(`Generated compose file:`, {
    path: result.path,
    size: result.size,
    generationTime: result.generationTime,
    services: result.services,
    networks: result.networks,
    volumes: result.volumes,
    errors: result.errors,
    warnings: result.warnings,
  });

  if (result.errors.length > 0) {
    throw new Error(`Compose generation failed: ${result.errors.join(', ')}`);
  }

  logger.info('✅ Basic compose generation test passed');
}

/**
 * Test multi-service compose generation
 */
async function testMultiServiceComposeGeneration() {
  logger.info('Testing multi-service compose generation...');

  const multiServiceConfig: ComposeConfiguration = {
    id: 'test-config-2',
    name: 'Multi-Service Application',
    version: '1.0.0',
    description: 'Multi-service application with microservices',
    environment: 'production',
    services: [
      {
        name: 'api-gateway',
        image: 'nginx:alpine',
        ports: ['80:80', '443:443'],
        depends_on: ['auth-service', 'user-service'],
        networks: ['frontend-network', 'backend-network'],
        restart: 'always',
        deploy: {
          replicas: 2,
          resources: {
            limits: {
              cpus: '0.5',
              memory: '512M',
            },
          },
        },
      },
      {
        name: 'auth-service',
        build: {
          context: './auth-service',
          dockerfile: 'Dockerfile',
        },
        environment: {
          JWT_SECRET: 'secret',
          DB_HOST: 'auth-db',
        },
        depends_on: ['auth-db'],
        networks: ['backend-network'],
        restart: 'always',
        healthcheck: {
          test: ['CMD', 'curl', '-f', 'http://localhost:3000/health'],
          interval: '30s',
          timeout: '10s',
          retries: 3,
        },
      },
      {
        name: 'user-service',
        build: {
          context: './user-service',
          dockerfile: 'Dockerfile',
        },
        environment: {
          DB_HOST: 'user-db',
        },
        depends_on: ['user-db'],
        networks: ['backend-network'],
        restart: 'always',
      },
      {
        name: 'auth-db',
        image: 'postgres:13',
        environment: {
          POSTGRES_DB: 'auth_db',
          POSTGRES_USER: 'auth_user',
          POSTGRES_PASSWORD: 'auth_pass',
        },
        volumes: ['auth_data:/var/lib/postgresql/data'],
        networks: ['backend-network'],
        restart: 'always',
      },
      {
        name: 'user-db',
        image: 'postgres:13',
        environment: {
          POSTGRES_DB: 'user_db',
          POSTGRES_USER: 'user_user',
          POSTGRES_PASSWORD: 'user_pass',
        },
        volumes: ['user_data:/var/lib/postgresql/data'],
        networks: ['backend-network'],
        restart: 'always',
      },
    ],
    networks: [
      {
        name: 'frontend-network',
        driver: 'bridge',
      },
      {
        name: 'backend-network',
        driver: 'bridge',
      },
    ],
    volumes: [
      {
        name: 'auth_data',
        driver: 'local',
      },
      {
        name: 'user_data',
        driver: 'local',
      },
    ],
  };

  // Register configuration
  dockerComposeGenerationService.registerComposeConfig(multiServiceConfig);

  // Generate compose file
  const result = await dockerComposeGenerationService.generateCompose('test-config-2', {
    template: 'docker-compose-production',
    includeComments: true,
  });

  logger.info(`Generated multi-service compose file:`, {
    path: result.path,
    size: result.size,
    generationTime: result.generationTime,
    services: result.services,
    networks: result.networks,
    volumes: result.volumes,
    errors: result.errors,
    warnings: result.warnings,
  });

  if (result.errors.length > 0) {
    throw new Error(`Multi-service compose generation failed: ${result.errors.join(', ')}`);
  }

  logger.info('✅ Multi-service compose generation test passed');
}

/**
 * Test environment-specific generation
 */
async function testEnvironmentSpecificGeneration() {
  logger.info('Testing environment-specific generation...');

  const devConfig: ComposeConfiguration = {
    id: 'test-config-3',
    name: 'Development App',
    version: '1.0.0',
    environment: 'development',
    services: [
      {
        name: 'dev-app',
        build: {
          context: '.',
          dockerfile: 'Dockerfile.dev',
        },
        ports: ['3000:3000'],
        volumes: ['./src:/app/src', './node_modules:/app/node_modules'],
        environment: {
          NODE_ENV: 'development',
          DEBUG: 'true',
        },
        restart: 'unless-stopped',
      },
    ],
  };

  // Register configuration
  dockerComposeGenerationService.registerComposeConfig(devConfig);

  // Generate development compose
  const devResult = await dockerComposeGenerationService.generateCompose('test-config-3', {
    template: 'docker-compose-development',
  });

  logger.info(`Generated development compose:`, {
    path: devResult.path,
    services: devResult.services,
    errors: devResult.errors,
  });

  if (devResult.errors.length > 0) {
    throw new Error(`Development compose generation failed: ${devResult.errors.join(', ')}`);
  }

  logger.info('✅ Environment-specific generation test passed');
}

/**
 * Test validation-only mode
 */
async function testValidationOnlyMode() {
  logger.info('Testing validation-only mode...');

  const result = await dockerComposeGenerationService.generateCompose('test-config-1', {
    validateOnly: true,
  });

  logger.info(`Validation result:`, {
    isValid: result.errors.length === 0,
    errors: result.errors,
    warnings: result.warnings,
    validationIssues: result.validationIssues,
  });

  logger.info('✅ Validation-only mode test passed');
}

/**
 * Test generated file management
 */
async function testGeneratedFileManagement() {
  logger.info('Testing generated file management...');

  // List generated files
  const generatedFiles = dockerComposeGenerationService.listGeneratedComposeFiles();
  logger.info(`Generated files: ${generatedFiles.join(', ')}`);

  // Get specific generated file
  const result = dockerComposeGenerationService.getGeneratedCompose('test-config-1');
  if (result) {
    logger.info(`Retrieved generated file: ${result.path}`);
  }

  logger.info('✅ Generated file management test passed');
}

/**
 * Test configuration retrieval
 */
async function testConfigurationRetrieval() {
  logger.info('Testing configuration retrieval...');

  const config = dockerComposeGenerationService.getComposeConfig('test-config-1');
  if (config) {
    logger.info(`Retrieved configuration: ${config.name} with ${config.services.length} services`);
  } else {
    throw new Error('Failed to retrieve configuration');
  }

  logger.info('✅ Configuration retrieval test passed');
}

/**
 * Test error handling
 */
async function testErrorHandling() {
  logger.info('Testing error handling...');

  // Test with non-existent configuration
  const result = await dockerComposeGenerationService.generateCompose('non-existent-config');

  logger.info(`Error handling result:`, {
    hasErrors: result.errors.length > 0,
    errorCount: result.errors.length,
    firstError: result.errors[0],
  });

  if (result.errors.length === 0) {
    throw new Error('Expected error for non-existent configuration');
  }

  logger.info('✅ Error handling test passed');
}

/**
 * Test cleanup operations
 */
async function testCleanupOperations() {
  logger.info('Testing cleanup operations...');

  // Test cleanup of old files (with 0 days to force cleanup)
  dockerComposeGenerationService.cleanupOldFiles(0);

  logger.info('✅ Cleanup operations test passed');
}

// Run tests if this file is executed directly
if (require.main === module) {
  testDockerComposeGenerationService()
    .then(() => {
      logger.info('🎉 All tests completed successfully!');
      process.exit(0);
    })
    .catch((error) => {
      logger.error('💥 Tests failed:', undefined, error as Error);
      process.exit(1);
    });
} 