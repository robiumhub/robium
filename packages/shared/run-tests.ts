import { testSchemaValidation } from './__tests__/schema-validation.test';
import { testModuleManager } from './__tests__/module-manager.test';
import { testCompatibilityValidationSystem } from './__tests__/compatibility-validation-system.test';

async function runTests() {
  console.log('Running schema validation tests...');
  await testSchemaValidation();
  
  console.log('Running module manager tests...');
  await testModuleManager();
  
  console.log('Running compatibility validation system tests...');
  await testCompatibilityValidationSystem();
  
  console.log('Tests completed!');
}

runTests().catch(console.error); 