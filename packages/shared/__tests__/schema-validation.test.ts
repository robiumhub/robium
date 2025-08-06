import { configValidator } from '../validation/config-validator';
import * as navigationProject from '../templates/navigation-project.json';
import * as manipulationProject from '../templates/manipulation-project.json';
import * as perceptionProject from '../templates/perception-project.json';
import * as customProject from '../templates/custom-project.json';

export async function testSchemaValidation() {
  const navResult = configValidator.validateProjectConfiguration((navigationProject as any).default || navigationProject);
  console.assert(navResult.valid, 'Navigation project should be valid');
  if (!navResult.valid) {
    console.error('Navigation project errors:', configValidator.getAllErrors(navResult));
  }

  const manipResult = configValidator.validateProjectConfiguration((manipulationProject as any).default || manipulationProject);
  console.assert(manipResult.valid, 'Manipulation project should be valid');
  if (!manipResult.valid) {
    console.error('Manipulation project errors:', configValidator.getAllErrors(manipResult));
  }

  const percepResult = configValidator.validateProjectConfiguration((perceptionProject as any).default || perceptionProject);
  console.assert(percepResult.valid, 'Perception project should be valid');
  if (!percepResult.valid) {
    console.error('Perception project errors:', configValidator.getAllErrors(percepResult));
  }

  const customResult = configValidator.validateProjectConfiguration((customProject as any).default || customProject);
  console.assert(customResult.valid, 'Custom project should be valid');
  if (!customResult.valid) {
    console.error('Custom project errors:', configValidator.getAllErrors(customResult));
  }

  const invalidConfig = { metadata: {} };
  const invalidResult = configValidator.validateProjectConfiguration(invalidConfig as any);
  console.assert(!invalidResult.valid, 'Invalid config should be invalid');
} 