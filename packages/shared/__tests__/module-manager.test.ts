import { ModuleManager } from '../utils/module-manager';
import * as path from 'path';

export async function testModuleManager() {
  const moduleManager = new ModuleManager(path.join(__dirname, '../modules'));

  // Test getting all modules
  const modules = await moduleManager.getAllModules();
  console.assert(modules.length > 0, 'Should have modules');
  
  // Check that we have the expected modules
  const moduleNames = modules.map(m => m.name);
  console.assert(moduleNames.includes('navigation_core'), 'Should have navigation_core module');
  console.assert(moduleNames.includes('sensor_fusion'), 'Should have sensor_fusion module');
  console.assert(moduleNames.includes('perception_basic'), 'Should have perception_basic module');
  console.assert(moduleNames.includes('manipulation_core'), 'Should have manipulation_core module');

  // Test getting module by name
  const module = await moduleManager.getModule('navigation_core');
  console.assert(module !== null, 'Should get navigation_core module');
  console.assert(module?.name === 'navigation_core', 'Module name should match');
  console.assert(module?.category === 'navigation', 'Module category should match');
  console.assert(module?.packages.includes('nav2_bringup'), 'Module should contain nav2_bringup package');

  // Test getting modules by category
  const navigationModules = await moduleManager.getModulesByCategory('navigation');
  console.assert(navigationModules.length === 1, 'Should have 1 navigation module');
  console.assert(navigationModules[0].name === 'navigation_core', 'Navigation module should be navigation_core');
  
  const perceptionModules = await moduleManager.getModulesByCategory('perception');
  console.assert(perceptionModules.length === 2, 'Should have 2 perception modules'); // sensor_fusion and perception_basic

  // Test validating module references
  const result = await moduleManager.validateModuleReferences([
    'navigation_core',
    'sensor_fusion',
    'nonexistent_module'
  ]);
  
  console.assert(result.valid.includes('navigation_core'), 'navigation_core should be valid');
  console.assert(result.valid.includes('sensor_fusion'), 'sensor_fusion should be valid');
  console.assert(result.invalid.includes('nonexistent_module'), 'nonexistent_module should be invalid');

  // Test getting packages from modules
  const packages = await moduleManager.getPackagesFromModules([
    'navigation_core',
    'sensor_fusion'
  ]);
  
  console.assert(packages.includes('nav2_bringup'), 'Should include nav2_bringup package');
  console.assert(packages.includes('robot_localization'), 'Should include robot_localization package');
  console.assert(packages.length > 0, 'Should have packages');
} 