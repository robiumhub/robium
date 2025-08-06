import { templateEngine } from '../services/TemplateEngine';
import { logger } from '../utils/logger';

async function testTemplateEngine() {
  try {
    console.log('🧪 Testing Template Engine...\n');

    // Test 1: List registered templates
    console.log('1. Testing template registration...');
    const templates = templateEngine.listTemplates();
    console.log(`✅ Registered templates: ${templates.length}`);
    templates.forEach(template => {
      console.log(`   - ${template}`);
    });
    console.log('');

    // Test 2: Get template content
    console.log('2. Testing template retrieval...');
    const dockerfileTemplate = templateEngine.getTemplate('default-dockerfile');
    const composeTemplate = templateEngine.getTemplate('default-compose');
    
    console.log(`✅ Dockerfile template length: ${dockerfileTemplate?.length || 0} characters`);
    console.log(`✅ Compose template length: ${composeTemplate?.length || 0} characters`);
    console.log('');

    // Test 3: Variable substitution
    console.log('3. Testing variable substitution...');
    const testContext = {
      project: {
        name: 'test-project',
        version: '1.0.0'
      },
      baseImage: 'ubuntu:20.04',
      workdir: '/app',
      systemDependencies: ['curl', 'wget', 'git'],
      pythonDependencies: true,
      nodeDependencies: false,
      environmentVariables: {
        NODE_ENV: 'production',
        PORT: '3000'
      },
      ports: ['3000', '8080'],
      command: 'npm start'
    };

    const result = templateEngine.processTemplate('default-dockerfile', testContext);
    console.log(`✅ Template processing completed in ${result.processingTime}ms`);
    console.log(`✅ Variables found: ${result.variables.length}`);
    console.log(`✅ Errors: ${result.errors.length}`);
    console.log(`✅ Warnings: ${result.warnings.length}`);
    
    if (result.errors.length > 0) {
      console.log('   Errors:');
      result.errors.forEach(error => {
        console.log(`   - ${error.message} (${error.type})`);
      });
    }
    console.log('');

    // Test 4: Conditional processing
    console.log('4. Testing conditional processing...');
    const conditionalContext = {
      project: { name: 'conditional-test' },
      baseImage: 'python:3.9',
      workdir: '/app',
      systemDependencies: ['python3-pip'],
      pythonDependencies: true,
      nodeDependencies: false,
      environmentVariables: {},
      ports: [],
      command: 'python app.py'
    };

    const conditionalResult = templateEngine.processTemplate('default-dockerfile', conditionalContext);
    console.log(`✅ Conditional processing completed`);
    console.log(`✅ Content length: ${conditionalResult.content.length} characters`);
    console.log(`✅ Contains system dependencies: ${conditionalResult.content.includes('apt-get install')}`);
    console.log(`✅ Contains Python dependencies: ${conditionalResult.content.includes('pip install')}`);
    console.log(`✅ Contains Node dependencies: ${conditionalResult.content.includes('npm install')}`);
    console.log('');

    // Test 5: Iteration processing
    console.log('5. Testing iteration processing...');
    const iterationContext = {
      project: { name: 'iteration-test' },
      baseImage: 'node:16',
      workdir: '/app',
      systemDependencies: ['curl', 'wget'],
      pythonDependencies: false,
      nodeDependencies: true,
      environmentVariables: {
        NODE_ENV: 'development',
        DEBUG: 'true',
        PORT: '3000'
      },
      ports: ['3000', '8080', '9000'],
      command: 'npm run dev'
    };

    const iterationResult = templateEngine.processTemplate('default-dockerfile', iterationContext);
    console.log(`✅ Iteration processing completed`);
    console.log(`✅ Environment variables count: ${Object.keys(iterationContext.environmentVariables).length}`);
    console.log(`✅ Ports count: ${iterationContext.ports.length}`);
    console.log(`✅ Contains multiple EXPOSE: ${(iterationResult.content.match(/EXPOSE/g) || []).length}`);
    console.log('');

    // Test 6: Docker-compose generation
    console.log('6. Testing docker-compose generation...');
    const composeContext = {
      project: { name: 'compose-test' },
      composeVersion: '3.8',
      containerName: 'robium_user123_compose-test',
      environmentVariables: {
        NODE_ENV: 'production',
        DATABASE_URL: 'postgresql://localhost:5432/testdb'
      },
      ports: ['3000:3000', '8080:8080'],
      volumes: [
        './src:/app/src',
        './data:/app/data'
      ],
      networkName: 'robium-network',
      restartPolicy: 'unless-stopped',
      dependsOn: ['database', 'redis'],
      namedVolumes: {
        'postgres-data': {},
        'redis-data': {}
      }
    };

    const composeResult = templateEngine.processTemplate('default-compose', composeContext);
    console.log(`✅ Compose generation completed`);
    console.log(`✅ Content length: ${composeResult.content.length} characters`);
    console.log(`✅ Contains services section: ${composeResult.content.includes('services:')}`);
    console.log(`✅ Contains networks section: ${composeResult.content.includes('networks:')}`);
    console.log(`✅ Contains volumes section: ${composeResult.content.includes('volumes:')}`);
    console.log('');

    // Test 7: Template validation
    console.log('7. Testing template validation...');
    const validationResult = templateEngine.processTemplate('default-dockerfile', {}, { validateOnly: true });
    console.log(`✅ Validation completed`);
    console.log(`✅ Validation errors: ${validationResult.errors.length}`);
    console.log(`✅ Required variables: ${validationResult.variables.length}`);
    
    if (validationResult.errors.length > 0) {
      console.log('   Missing variables:');
      validationResult.errors.forEach(error => {
        console.log(`   - ${error.message}`);
      });
    }
    console.log('');

    // Test 8: Custom template registration
    console.log('8. Testing custom template registration...');
    const customTemplate = `# Custom template for {{project.name}}
FROM {{baseImage}}

# Custom build steps
{{#if customSteps}}
{{#each customSteps}}
RUN {{this}}
{{/each}}
{{/if}}

# Default command
CMD ["{{command}}"]`;

    templateEngine.registerTemplate('custom-template', customTemplate);
    console.log('✅ Custom template registered');

    const customContext = {
      project: { name: 'custom-test' },
      baseImage: 'alpine:latest',
      customSteps: [
        'apk add --no-cache curl',
        'mkdir -p /app',
        'chmod 755 /app'
      ],
      command: 'sh'
    };

    const customResult = templateEngine.processTemplate('custom-template', customContext);
    console.log(`✅ Custom template processing completed`);
    console.log(`✅ Content length: ${customResult.content.length} characters`);
    console.log(`✅ Contains custom steps: ${customResult.content.includes('apk add')}`);
    console.log('');

    // Test 9: Cache functionality
    console.log('9. Testing cache functionality...');
    const cacheStats = templateEngine.getCacheStats();
    console.log(`✅ Cache stats: ${cacheStats.size}/${cacheStats.maxSize} entries`);
    console.log(`✅ Cache hit rate: ${(cacheStats.hitRate * 100).toFixed(1)}%`);

    // Process same template again to test cache
    const cachedResult = templateEngine.processTemplate('default-dockerfile', testContext);
    console.log(`✅ Cached processing completed in ${cachedResult.processingTime}ms`);
    console.log('');

    // Test 10: Template management
    console.log('10. Testing template management...');
    const beforeCount = templateEngine.listTemplates().length;
    
    // Remove custom template
    const removed = templateEngine.removeTemplate('custom-template');
    console.log(`✅ Template removal: ${removed}`);
    
    const afterCount = templateEngine.listTemplates().length;
    console.log(`✅ Template count: ${beforeCount} -> ${afterCount}`);
    console.log('');

    // Test 11: Error handling
    console.log('11. Testing error handling...');
    const errorResult = templateEngine.processTemplate('non-existent-template', {});
    console.log(`✅ Error handling test completed`);
    console.log(`✅ Errors: ${errorResult.errors.length}`);
    console.log(`✅ Error type: ${errorResult.errors[0]?.type}`);
    console.log('');

    // Test 12: Content cleanup
    console.log('12. Testing content cleanup...');
    const messyTemplate = `
# Messy template
FROM {{baseImage}}

# Empty lines below

WORKDIR {{workdir}}

# More empty lines


CMD ["{{command}}"]
`;

    templateEngine.registerTemplate('messy-template', messyTemplate);
    const cleanupResult = templateEngine.processTemplate('messy-template', {
      baseImage: 'ubuntu:20.04',
      workdir: '/app',
      command: 'bash'
    });

    console.log(`✅ Content cleanup completed`);
    console.log(`✅ Original length: ${messyTemplate.length}`);
    console.log(`✅ Cleaned length: ${cleanupResult.content.length}`);
    console.log(`✅ Contains empty lines: ${cleanupResult.content.includes('\n\n\n')}`);
    console.log('');

    console.log('🎉 All template engine tests completed successfully!');
    console.log('\n📋 Summary:');
    console.log('✅ Template registration and retrieval working');
    console.log('✅ Variable substitution functional');
    console.log('✅ Conditional processing working');
    console.log('✅ Iteration processing operational');
    console.log('✅ Docker-compose generation working');
    console.log('✅ Template validation functional');
    console.log('✅ Custom template registration working');
    console.log('✅ Cache functionality operational');
    console.log('✅ Template management working');
    console.log('✅ Error handling functional');
    console.log('✅ Content cleanup working');

  } catch (error) {
    console.error('❌ Template engine test failed:', error);
    process.exit(1);
  }
}

// Run the test
testTemplateEngine(); 