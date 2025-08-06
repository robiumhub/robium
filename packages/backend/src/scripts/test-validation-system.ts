import { validationService } from '../services/ValidationService';
import { environmentVariableService } from '../services/EnvironmentVariableService';
import { dockerfileGenerationService } from '../services/DockerfileGenerationService';
import { dockerComposeGenerationService } from '../services/DockerComposeGenerationService';
import { logger } from '../utils/logger';

async function testValidationSystem() {
  console.log('🧪 Testing Validation System\n');

  try {
    // Test 1: Create test configurations
    console.log('1. Creating test configurations...');
    
    // Create environment configuration
    const envConfig = await environmentVariableService.createEnvironmentConfig(
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
        }
      }
    );

    // Create Dockerfile configuration
    const dockerfileConfig = {
      id: 'test-dockerfile-123',
      name: 'Test Python App',
      version: '1.0.0',
      description: 'Test application for validation',
      type: 'python' as const,
      baseImage: 'python:3.9-slim',
      workdir: '/app',
      systemDependencies: ['curl'],
      pythonDependencies: ['flask'],
      environmentVariables: {
        DATABASE_URL: 'postgresql://user:password@localhost:5432/testdb',
        API_KEY: 'secret-api-key-123'
      },
      ports: ['8080'],
      command: 'python app.py'
    };

    // Create Docker Compose configuration
    const composeConfig = {
      id: 'test-compose-123',
      name: 'Test Compose App',
      version: '1.0.0',
      description: 'Test docker-compose for validation',
      environment: 'development' as const,
      variables: {
        DATABASE_URL: 'postgresql://user:password@localhost:5432/testdb',
        API_KEY: 'secret-api-key-123'
      },
      services: [
        {
          name: 'app',
          build: {
            context: '.',
            dockerfile: 'Dockerfile'
          },
          ports: ['8080:8080'],
          environment: {
            DATABASE_URL: 'postgresql://user:password@localhost:5432/testdb',
            API_KEY: 'secret-api-key-123'
          },
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

    console.log('✅ Test configurations created');

    // Test 2: Generate Docker configurations
    console.log('\n2. Generating Docker configurations...');
    
    // Register configurations
    dockerfileGenerationService.registerProjectConfig(dockerfileConfig);
    dockerComposeGenerationService.registerComposeConfig(composeConfig);

    // Generate Dockerfile
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

    // Generate Docker Compose
    const composeResult = await dockerComposeGenerationService.generateCompose(
      'test-compose-123',
      {
        template: 'docker-compose-development',
        outputPath: './test-output',
        validateOnly: false,
        includeComments: true
      }
    );

    console.log('✅ Docker configurations generated');

    // Test 3: Validate Dockerfile
    console.log('\n3. Validating Dockerfile...');
    const dockerfileValidation = validationService.validateDockerfile({
      content: dockerfileResult.content,
      config: dockerfileConfig,
      filePath: dockerfileResult.path
    });

    console.log(`✅ Dockerfile validation completed:`);
    console.log(`   - Valid: ${dockerfileValidation.isValid}`);
    console.log(`   - Errors: ${dockerfileValidation.errors.length}`);
    console.log(`   - Warnings: ${dockerfileValidation.warnings.length}`);
    console.log(`   - Suggestions: ${dockerfileValidation.suggestions.length}`);
    console.log(`   - Checks: ${dockerfileValidation.metadata.passedChecks}/${dockerfileValidation.metadata.totalChecks} passed`);

    // Test 4: Validate Docker Compose
    console.log('\n4. Validating Docker Compose...');
    const composeValidation = validationService.validateCompose({
      content: composeResult.content,
      config: composeConfig,
      filePath: composeResult.path
    });

    console.log(`✅ Docker Compose validation completed:`);
    console.log(`   - Valid: ${composeValidation.isValid}`);
    console.log(`   - Errors: ${composeValidation.errors.length}`);
    console.log(`   - Warnings: ${composeValidation.warnings.length}`);
    console.log(`   - Suggestions: ${composeValidation.suggestions.length}`);
    console.log(`   - Checks: ${composeValidation.metadata.passedChecks}/${composeValidation.metadata.totalChecks} passed`);

    // Test 5: Validate Environment Configuration
    console.log('\n5. Validating Environment Configuration...');
    const environmentValidation = validationService.validateEnvironment({
      config: envConfig,
      variables: envConfig.variables
    });

    console.log(`✅ Environment validation completed:`);
    console.log(`   - Valid: ${environmentValidation.isValid}`);
    console.log(`   - Errors: ${environmentValidation.errors.length}`);
    console.log(`   - Warnings: ${environmentValidation.warnings.length}`);
    console.log(`   - Suggestions: ${environmentValidation.suggestions.length}`);
    console.log(`   - Checks: ${environmentValidation.metadata.passedChecks}/${environmentValidation.metadata.totalChecks} passed`);

    // Test 6: Validate Complete Project
    console.log('\n6. Validating Complete Project...');
    const projectValidation = validationService.validateProject(
      {
        content: dockerfileResult.content,
        config: dockerfileConfig,
        filePath: dockerfileResult.path
      },
      {
        content: composeResult.content,
        config: composeConfig,
        filePath: composeResult.path
      },
      {
        config: envConfig,
        variables: envConfig.variables
      }
    );

    console.log(`✅ Project validation completed:`);
    console.log(`   - Valid: ${projectValidation.isValid}`);
    console.log(`   - Errors: ${projectValidation.errors.length}`);
    console.log(`   - Warnings: ${projectValidation.warnings.length}`);
    console.log(`   - Suggestions: ${projectValidation.suggestions.length}`);
    console.log(`   - Checks: ${projectValidation.metadata.passedChecks}/${projectValidation.metadata.totalChecks} passed`);
    console.log(`   - Validation time: ${projectValidation.metadata.validationTime}ms`);

    // Test 7: Test with problematic configurations
    console.log('\n7. Testing with problematic configurations...');
    
    // Create a problematic Dockerfile (missing FROM instruction)
    const problematicDockerfile = `# Problematic Dockerfile
WORKDIR /app
COPY . .
CMD ["python", "app.py"]`;

    const problematicDockerfileValidation = validationService.validateDockerfile({
      content: problematicDockerfile,
      config: dockerfileConfig,
      filePath: './problematic-dockerfile'
    });

    console.log(`✅ Problematic Dockerfile validation:`);
    console.log(`   - Valid: ${problematicDockerfileValidation.isValid}`);
    console.log(`   - Errors: ${problematicDockerfileValidation.errors.length}`);
    console.log(`   - Warnings: ${problematicDockerfileValidation.warnings.length}`);
    console.log(`   - Suggestions: ${problematicDockerfileValidation.suggestions.length}`);

    // Show specific errors
    if (problematicDockerfileValidation.errors.length > 0) {
      console.log('   - Error details:');
      for (const error of problematicDockerfileValidation.errors) {
        console.log(`     * ${error.message} (${error.severity})`);
        if (error.fix) {
          console.log(`       Fix: ${error.fix}`);
        }
      }
    }

    // Test 8: Test custom validation rules
    console.log('\n8. Testing custom validation rules...');
    
    // Add a custom validator
    validationService.addCustomValidator('custom-security-check', {
      type: 'dockerfile',
      validate: (context: any): any => {
        const errors: any[] = [];
        const warnings: any[] = [];
        const suggestions: any[] = [];

        // Check for hardcoded passwords
        if (context.content.includes('password=')) {
          errors.push({
            type: 'error',
            code: 'HARDCODED_PASSWORD',
            message: 'Hardcoded password detected in Dockerfile',
            severity: 'critical',
            fixable: true,
            fix: 'Use build args or environment variables for passwords'
          });
        }

        return {
          isValid: errors.length === 0,
          errors,
          warnings,
          suggestions,
          metadata: {
            totalChecks: 1,
            passedChecks: 1 - errors.length,
            failedChecks: errors.length,
            validationTime: 0
          }
        };
      }
    });

    // Test the custom validator
    const customValidation = validationService.validateDockerfile({
      content: dockerfileResult.content,
      config: dockerfileConfig,
      filePath: dockerfileResult.path
    });

    console.log(`✅ Custom validation completed:`);
    console.log(`   - Valid: ${customValidation.isValid}`);
    console.log(`   - Total checks: ${customValidation.metadata.totalChecks}`);

    // Test 9: Get validation statistics
    console.log('\n9. Getting validation statistics...');
    const stats = validationService.getValidationStats();
    
    console.log(`✅ Validation statistics:`);
    console.log(`   - Total rules: ${stats.totalRules}`);
    console.log(`   - Total validators: ${stats.totalValidators}`);
    console.log(`   - Rule types: ${stats.ruleTypes.join(', ')}`);

    // Test 10: Test validation with different scenarios
    console.log('\n10. Testing validation scenarios...');
    
    // Test with empty configurations
    const emptyDockerfileValidation = validationService.validateDockerfile({
      content: '',
      config: dockerfileConfig,
      filePath: './empty-dockerfile'
    });

    console.log(`✅ Empty Dockerfile validation:`);
    console.log(`   - Valid: ${emptyDockerfileValidation.isValid}`);
    console.log(`   - Errors: ${emptyDockerfileValidation.errors.length}`);

    // Test with complex Dockerfile
    const complexDockerfile = `# Multi-stage build
FROM python:3.9-slim as builder
WORKDIR /app
COPY requirements.txt .
RUN pip install --user -r requirements.txt

FROM python:3.9-slim
WORKDIR /app
COPY --from=builder /root/.local /root/.local
COPY . .
ENV PATH=/root/.local/bin:$PATH
USER nobody
EXPOSE 8080
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \\
  CMD curl -f http://localhost:8080/health || exit 1
CMD ["python", "app.py"]`;

    const complexDockerfileValidation = validationService.validateDockerfile({
      content: complexDockerfile,
      config: dockerfileConfig,
      filePath: './complex-dockerfile'
    });

    console.log(`✅ Complex Dockerfile validation:`);
    console.log(`   - Valid: ${complexDockerfileValidation.isValid}`);
    console.log(`   - Errors: ${complexDockerfileValidation.errors.length}`);
    console.log(`   - Warnings: ${complexDockerfileValidation.warnings.length}`);
    console.log(`   - Suggestions: ${complexDockerfileValidation.suggestions.length}`);
    console.log(`   - Complexity: ${complexDockerfileValidation.metadata.complexity}`);

    console.log('\n🎉 All validation system tests completed successfully!');

  } catch (error) {
    console.error('❌ Test failed:', error);
    logger.error('Validation system test failed:', error as Record<string, unknown>);
  }
}

// Run the test
testValidationSystem().catch(console.error); 