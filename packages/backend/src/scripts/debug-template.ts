import { templateEngine } from '../services/TemplateEngine';
import { dockerComposeGenerationService } from '../services/DockerComposeGenerationService';

async function debugTemplate() {
  console.log('🧪 Debugging Template Processing...\n');

  // Test 1: Simple volume processing
  console.log('1. Testing simple volume processing...');
  const testTemplate = `volumes:
  {{#each volumes}}
  - {{this}}
  {{/each}}`;

  const testContext = {
    volumes: [
      '/host/path:/container/path',
      '/another/path:/another/container'
    ]
  };

  console.log('Test Context:', JSON.stringify(testContext, null, 2));
  console.log('Test Template:', testTemplate);

  // Register the template first
  templateEngine.registerTemplate('debug-template', testTemplate);

  const result = templateEngine.processTemplate('debug-template', testContext);
  
  console.log('Result:');
  console.log(result.content);
  console.log('Errors:', result.errors);
  console.log('Warnings:', result.warnings);
  console.log('');

  // Test 2: Development template with actual service context
  console.log('2. Testing development template with service context...');
  const serviceContext = {
    project: {
      name: 'Test Project',
      version: '1.0.0'
    },
    composeVersion: '3.8',
    services: [
      {
        name: 'app',
        volumes: [
          '/host/path:/container/path',
          '/another/path:/another/container'
        ]
      }
    ]
  };

  console.log('Service Context:', JSON.stringify(serviceContext, null, 2));

  // Check available templates after DockerComposeGenerationService initialization
  console.log('Available templates after DockerComposeGenerationService init:', templateEngine.listTemplates());

  const devResult = templateEngine.processTemplate('docker-compose-development', serviceContext);
  
  console.log('Development Template Result:');
  console.log(devResult.content);
  console.log('Errors:', devResult.errors);
  console.log('Warnings:', devResult.warnings);
}

debugTemplate(); 