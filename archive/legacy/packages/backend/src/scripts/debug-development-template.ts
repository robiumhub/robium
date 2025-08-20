import { dockerComposeGenerationService } from '../services/DockerComposeGenerationService';
import { workspaceMountingService } from '../services/WorkspaceMountingService';

async function debugDevelopmentTemplate() {
  console.log('🧪 Debugging Development Template Volume Processing...\n');

  // Create a test mount point
  const mountPoint = await workspaceMountingService.createWorkspace({
    userId: 'test-user',
    projectId: 'test-project',
    workspacePath: '/test/workspace',
    containerPath: '/app/workspace'
  });

  console.log('1. Mount point created:');
  console.log('   ID:', mountPoint.id);
  console.log('   Host Path:', mountPoint.hostPath);
  console.log('   Container Path:', mountPoint.containerPath);

  // Test with the exact same service structure as the test
  const testService = {
    name: 'app',
    build: {
      context: '.',
      dockerfile: 'Dockerfile'
    },
    ports: ['3000:3000'],
    environment: {
      NODE_ENV: 'development',
      PORT: '3000'
    },
    restart: 'unless-stopped',
    volumes: [
      workspaceMountingService.generateMountConfig(mountPoint)
    ]
  };

  console.log('\n2. Test service:');
  console.log('   Volumes:', testService.volumes);
  console.log('   Volumes type:', typeof testService.volumes);
  console.log('   First volume type:', typeof testService.volumes[0]);

  // Create the exact same config as the test
  const testConfig = {
    id: 'test-config',
    name: 'Test Application',
    version: '1.0.0',
    description: 'Test application with workspace mounting',
    services: [testService],
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
    ]
  };

  // Test template context preparation
  const context = (dockerComposeGenerationService as any).prepareTemplateContext(testConfig, {});
  
  console.log('\n3. Template context:');
  console.log('   Services:', JSON.stringify(context.services, null, 2));
  console.log('   First service volumes:', context.services[0].volumes);
  console.log('   First service volumes type:', typeof context.services[0].volumes);
  console.log('   First volume type:', typeof context.services[0].volumes[0]);

  // Test with the exact development template
  const developmentTemplate = `# Generated docker-compose.yml for {{project.name}} (Development)
version: '{{composeVersion}}'

services:
{{#each services}}
  {{name}}:
    {{#if build}}
    build:
      context: {{build.context}}
    {{/if}}
    {{#if ports}}
    ports:
      {{#each ports}}
      - "{{this}}"
      {{/each}}
    {{/if}}
    {{#if volumes}}
    volumes:
      {{#each volumes}}
      - {{this}}
      {{/each}}
    {{/if}}
    {{#if environment}}
    environment:
      {{#each environment}}
      {{@key}}: {{this}}
      {{/each}}
    {{/if}}
    restart: unless-stopped
{{/each}}

{{#if networks}}
networks:
  {{#each networks}}
  {{name}}:
    driver: {{driver}}
  {{/each}}
{{/if}}

{{#if volumes}}
volumes:
  {{#each volumes}}
  {{name}}:
    driver: {{driver}}
  {{/each}}
{{/if}}`;

  const { templateEngine } = require('../services/TemplateEngine');
  templateEngine.registerTemplate('debug-development', developmentTemplate);
  
  const result = templateEngine.processTemplate('debug-development', context);
  
  console.log('\n4. Development template processing result:');
  console.log('   Content:');
  console.log(result.content);
  console.log('   Errors:', result.errors);
  console.log('   Warnings:', result.warnings);
}

debugDevelopmentTemplate().catch(console.error); 