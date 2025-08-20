import { dockerComposeGenerationService } from '../services/DockerComposeGenerationService';
import { workspaceMountingService } from '../services/WorkspaceMountingService';

async function debugVolumes() {
  console.log('🧪 Debugging Volume Processing...\n');

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

  // Test generateMountConfig
  const mountConfig = workspaceMountingService.generateMountConfig(mountPoint);
  console.log('\n2. Generated mount config:');
  console.log('   Type:', typeof mountConfig);
  console.log('   Value:', mountConfig);

  // Test with a simple service
  const testService = {
    name: 'test-service',
    image: 'test:latest',
    volumes: [mountConfig]
  };

  console.log('\n3. Test service volumes:');
  console.log('   Volumes array:', testService.volumes);
  console.log('   Volumes type:', typeof testService.volumes);
  console.log('   First volume type:', typeof testService.volumes[0]);

  // Test template context preparation
  const testConfig = {
    id: 'test-config',
    name: 'Test Config',
    version: '1.0.0',
    services: [testService]
  };

  const context = (dockerComposeGenerationService as any).prepareTemplateContext(testConfig, {});
  
  console.log('\n4. Template context:');
  console.log('   Services:', context.services);
  console.log('   First service volumes:', context.services[0].volumes);
  console.log('   First service volumes type:', typeof context.services[0].volumes);
  console.log('   First volume type:', typeof context.services[0].volumes[0]);

  // Test template processing
  const template = `volumes:
  {{#each volumes}}
  - {{this}}
  {{/each}}`;

  const { templateEngine } = require('../services/TemplateEngine');
  templateEngine.registerTemplate('debug-volumes', template);
  
  const result = templateEngine.processTemplate('debug-volumes', { volumes: context.services[0].volumes });
  
  console.log('\n5. Template processing result:');
  console.log('   Content:', result.content);
  console.log('   Errors:', result.errors);
}

debugVolumes().catch(console.error); 