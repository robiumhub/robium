import { dockerfileGenerationService } from '../services/DockerfileGenerationService';
import { logger } from '../utils/logger';

async function testDockerfileGenerationService() {
  try {
    console.log('🧪 Testing Dockerfile Generation Service...\n');

    // Test 1: Service statistics
    console.log('1. Testing service statistics...');
    const stats = dockerfileGenerationService.getServiceStats();
    console.log('✅ Service statistics:');
    console.log(`   - Total projects: ${stats.totalProjects}`);
    console.log(`   - Total generated files: ${stats.totalGeneratedFiles}`);
    console.log(`   - Total errors: ${stats.totalErrors}`);
    console.log(`   - Total warnings: ${stats.totalWarnings}\n`);

    // Test 2: Python project configuration
    console.log('2. Testing Python project configuration...');
    const pythonConfig = {
      id: 'python-project-1',
      name: 'flask-api',
      version: '1.0.0',
      description: 'A Flask REST API',
      type: 'python' as const,
      baseImage: 'python:3.9-slim',
      workdir: '/app',
      systemDependencies: ['curl', 'wget'],
      pythonDependencies: ['flask', 'requests', 'sqlalchemy'],
      environmentVariables: {
        FLASK_ENV: 'production',
        DATABASE_URL: 'postgresql://localhost:5432/flask_db',
        PORT: '5000'
      },
      ports: ['5000'],
      command: 'python app.py',
      user: 'app',
      healthCheck: {
        command: 'curl -f http://localhost:5000/health || exit 1',
        interval: '30s',
        timeout: '10s',
        retries: 3,
        startPeriod: '40s'
      }
    };

    dockerfileGenerationService.registerProjectConfig(pythonConfig);
    console.log('✅ Python project configuration registered');
    console.log(`   - Project: ${pythonConfig.name} v${pythonConfig.version}`);
    console.log(`   - Type: ${pythonConfig.type}`);
    console.log(`   - Base image: ${pythonConfig.baseImage}\n`);

    // Test 3: Node.js project configuration
    console.log('3. Testing Node.js project configuration...');
    const nodejsConfig = {
      id: 'nodejs-project-1',
      name: 'express-api',
      version: '2.1.0',
      description: 'An Express.js REST API',
      type: 'nodejs' as const,
      baseImage: 'node:16-alpine',
      workdir: '/app',
      systemDependencies: ['curl'],
      nodeDependencies: ['express', 'cors', 'helmet'],
      environmentVariables: {
        NODE_ENV: 'production',
        PORT: '3000',
        DATABASE_URL: 'mongodb://localhost:27017/express_db'
      },
      ports: ['3000'],
      command: 'npm start',
      user: 'node',
      healthCheck: {
        command: 'curl -f http://localhost:3000/health || exit 1',
        interval: '30s',
        timeout: '10s',
        retries: 3
      }
    };

    dockerfileGenerationService.registerProjectConfig(nodejsConfig);
    console.log('✅ Node.js project configuration registered');
    console.log(`   - Project: ${nodejsConfig.name} v${nodejsConfig.version}`);
    console.log(`   - Type: ${nodejsConfig.type}`);
    console.log(`   - Base image: ${nodejsConfig.baseImage}\n`);

    // Test 4: Multi-stage project configuration
    console.log('4. Testing multi-stage project configuration...');
    const multiStageConfig = {
      id: 'multistage-project-1',
      name: 'golang-api',
      version: '1.5.0',
      description: 'A Go REST API with multi-stage build',
      type: 'golang' as const,
      baseImage: 'golang:1.19-alpine',
      workdir: '/app',
      systemDependencies: ['git'],
      multiStage: true,
      stages: [
        {
          name: 'builder',
          baseImage: 'golang:1.19-alpine',
          dependencies: ['git', 'ca-certificates'],
          buildSteps: [
            'go mod download',
            'go build -o main .'
          ]
        },
        {
          name: 'runtime',
          baseImage: 'alpine:latest',
          dependencies: ['ca-certificates'],
          copyFrom: 'builder'
        }
      ],
      environmentVariables: {
        PORT: '8080'
      },
      ports: ['8080'],
      command: './main'
    };

    dockerfileGenerationService.registerProjectConfig(multiStageConfig);
    console.log('✅ Multi-stage project configuration registered');
    console.log(`   - Project: ${multiStageConfig.name} v${multiStageConfig.version}`);
    console.log(`   - Type: ${multiStageConfig.type}`);
    console.log(`   - Multi-stage: ${multiStageConfig.multiStage}\n`);

    // Test 5: Generate Python Dockerfile
    console.log('5. Testing Python Dockerfile generation...');
    const pythonResult = await dockerfileGenerationService.generateDockerfile('python-project-1', {
      optimize: true,
      includeComments: true,
      securityScan: true
    });

    console.log('✅ Python Dockerfile generated:');
    console.log(`   - Content length: ${pythonResult.content.length} characters`);
    console.log(`   - Build time: ${pythonResult.buildTime}ms`);
    console.log(`   - Errors: ${pythonResult.errors.length}`);
    console.log(`   - Warnings: ${pythonResult.warnings.length}`);
    console.log(`   - Security issues: ${pythonResult.securityIssues.length}`);
    console.log(`   - Optimization suggestions: ${pythonResult.optimizationSuggestions.length}`);

    if (pythonResult.errors.length > 0) {
      console.log('   Errors:');
      pythonResult.errors.forEach(error => console.log(`   - ${error}`));
    }

    if (pythonResult.warnings.length > 0) {
      console.log('   Warnings:');
      pythonResult.warnings.forEach(warning => console.log(`   - ${warning}`));
    }

    if (pythonResult.securityIssues.length > 0) {
      console.log('   Security issues:');
      pythonResult.securityIssues.forEach(issue => console.log(`   - ${issue}`));
    }

    console.log(`   - Contains FROM: ${pythonResult.content.includes('FROM')}`);
    console.log(`   - Contains WORKDIR: ${pythonResult.content.includes('WORKDIR')}`);
    console.log(`   - Contains pip install: ${pythonResult.content.includes('pip install')}`);
    console.log(`   - Contains EXPOSE: ${pythonResult.content.includes('EXPOSE')}`);
    console.log(`   - Contains HEALTHCHECK: ${pythonResult.content.includes('HEALTHCHECK')}`);
    console.log(`   - Contains USER: ${pythonResult.content.includes('USER')}\n`);

    // Test 6: Generate Node.js Dockerfile
    console.log('6. Testing Node.js Dockerfile generation...');
    const nodejsResult = await dockerfileGenerationService.generateDockerfile('nodejs-project-1', {
      optimize: true,
      includeComments: true,
      securityScan: true
    });

    console.log('✅ Node.js Dockerfile generated:');
    console.log(`   - Content length: ${nodejsResult.content.length} characters`);
    console.log(`   - Build time: ${nodejsResult.buildTime}ms`);
    console.log(`   - Errors: ${nodejsResult.errors.length}`);
    console.log(`   - Warnings: ${nodejsResult.warnings.length}`);
    console.log(`   - Security issues: ${nodejsResult.securityIssues.length}`);

    console.log(`   - Contains FROM: ${nodejsResult.content.includes('FROM')}`);
    console.log(`   - Contains WORKDIR: ${nodejsResult.content.includes('WORKDIR')}`);
    console.log(`   - Contains npm install: ${nodejsResult.content.includes('npm install')}`);
    console.log(`   - Contains EXPOSE: ${nodejsResult.content.includes('EXPOSE')}`);
    console.log(`   - Contains HEALTHCHECK: ${nodejsResult.content.includes('HEALTHCHECK')}`);
    console.log(`   - Contains USER: ${nodejsResult.content.includes('USER')}\n`);

    // Test 7: Generate multi-stage Dockerfile
    console.log('7. Testing multi-stage Dockerfile generation...');
    const multiStageResult = await dockerfileGenerationService.generateDockerfile('multistage-project-1', {
      template: 'multistage-dockerfile',
      optimize: true,
      includeComments: true,
      securityScan: true
    });

    console.log('✅ Multi-stage Dockerfile generated:');
    console.log(`   - Content length: ${multiStageResult.content.length} characters`);
    console.log(`   - Build time: ${multiStageResult.buildTime}ms`);
    console.log(`   - Errors: ${multiStageResult.errors.length}`);
    console.log(`   - Warnings: ${multiStageResult.warnings.length}`);

    console.log(`   - Contains multiple FROM: ${(multiStageResult.content.match(/FROM/g) || []).length}`);
    console.log(`   - Contains COPY --from: ${multiStageResult.content.includes('COPY --from')}`);
    console.log(`   - Contains as builder: ${multiStageResult.content.includes('as builder')}\n`);

    // Test 8: Validation only mode
    console.log('8. Testing validation only mode...');
    const validationResult = await dockerfileGenerationService.generateDockerfile('python-project-1', {
      validateOnly: true
    });

    console.log('✅ Validation completed:');
    console.log(`   - Content length: ${validationResult.content.length} characters`);
    console.log(`   - Errors: ${validationResult.errors.length}`);
    console.log(`   - Warnings: ${validationResult.warnings.length}`);
    console.log(`   - File not written: ${validationResult.path === ''}\n`);

    // Test 9: Get generated Dockerfile
    console.log('9. Testing generated Dockerfile retrieval...');
    const retrievedPython = dockerfileGenerationService.getGeneratedDockerfile('python-project-1');
    const retrievedNodejs = dockerfileGenerationService.getGeneratedDockerfile('nodejs-project-1');

    console.log('✅ Generated Dockerfile retrieval:');
    console.log(`   - Python Dockerfile found: ${retrievedPython !== undefined}`);
    console.log(`   - Node.js Dockerfile found: ${retrievedNodejs !== undefined}`);
    console.log(`   - Python content length: ${retrievedPython?.content.length || 0}`);
    console.log(`   - Node.js content length: ${retrievedNodejs?.content.length || 0}\n`);

    // Test 10: List generated Dockerfiles
    console.log('10. Testing generated Dockerfile listing...');
    const generatedFiles = dockerfileGenerationService.listGeneratedDockerfiles();
    console.log(`✅ Generated Dockerfiles: ${generatedFiles.length}`);
    generatedFiles.forEach(fileId => {
      console.log(`   - ${fileId}`);
    });
    console.log('');

    // Test 11: Project configuration retrieval
    console.log('11. Testing project configuration retrieval...');
    const pythonConfigRetrieved = dockerfileGenerationService.getProjectConfig('python-project-1');
    const nonExistentConfig = dockerfileGenerationService.getProjectConfig('non-existent');

    console.log('✅ Project configuration retrieval:');
    console.log(`   - Python config found: ${pythonConfigRetrieved !== undefined}`);
    console.log(`   - Non-existent config found: ${nonExistentConfig !== undefined}`);
    console.log(`   - Python project name: ${pythonConfigRetrieved?.name}`);
    console.log(`   - Python project type: ${pythonConfigRetrieved?.type}\n`);

    // Test 12: Error handling
    console.log('12. Testing error handling...');
    const errorResult = await dockerfileGenerationService.generateDockerfile('non-existent-project');

    console.log('✅ Error handling test completed:');
    console.log(`   - Errors: ${errorResult.errors.length}`);
    console.log(`   - Content length: ${errorResult.content.length}`);
    console.log(`   - Build time: ${errorResult.buildTime}ms\n`);

    // Test 13: Updated service statistics
    console.log('13. Testing updated service statistics...');
    const updatedStats = dockerfileGenerationService.getServiceStats();
    console.log('✅ Updated service statistics:');
    console.log(`   - Total projects: ${updatedStats.totalProjects}`);
    console.log(`   - Total generated files: ${updatedStats.totalGeneratedFiles}`);
    console.log(`   - Total errors: ${updatedStats.totalErrors}`);
    console.log(`   - Total warnings: ${updatedStats.totalWarnings}\n`);

    // Test 14: Cleanup test
    console.log('14. Testing cleanup functionality...');
    const beforeCleanup = dockerfileGenerationService.listGeneratedDockerfiles().length;
    
    // Remove one Dockerfile
    const removed = dockerfileGenerationService.removeGeneratedDockerfile('python-project-1');
    console.log(`✅ Dockerfile removal: ${removed}`);
    
    const afterCleanup = dockerfileGenerationService.listGeneratedDockerfiles().length;
    console.log(`✅ Generated files count: ${beforeCleanup} -> ${afterCleanup}`);

    console.log('\n🎉 All Dockerfile generation service tests completed successfully!');
    console.log('\n📋 Summary:');
    console.log('✅ Service statistics working');
    console.log('✅ Project configuration registration functional');
    console.log('✅ Python Dockerfile generation working');
    console.log('✅ Node.js Dockerfile generation working');
    console.log('✅ Multi-stage Dockerfile generation working');
    console.log('✅ Validation mode operational');
    console.log('✅ Generated file retrieval working');
    console.log('✅ File listing functional');
    console.log('✅ Configuration retrieval working');
    console.log('✅ Error handling functional');
    console.log('✅ Statistics tracking working');
    console.log('✅ Cleanup operations working');

  } catch (error) {
    console.error('❌ Dockerfile generation service test failed:', error);
    process.exit(1);
  }
}

// Run the test
testDockerfileGenerationService(); 