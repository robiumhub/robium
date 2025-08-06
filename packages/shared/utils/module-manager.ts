import * as fs from 'fs';
import * as path from 'path';
import { ModuleMetadata } from '../types/module-metadata';

export class ModuleManager {
  private modulesPath: string;

  constructor(modulesPath: string = path.join(__dirname, '../modules')) {
    this.modulesPath = modulesPath;
  }

  /**
   * Get all available modules
   */
  async getAllModules(): Promise<ModuleMetadata[]> {
    const modules: ModuleMetadata[] = [];
    
    try {
      const files = await fs.promises.readdir(this.modulesPath);
      
      for (const file of files) {
        if (file.endsWith('.json')) {
          const modulePath = path.join(this.modulesPath, file);
          const content = await fs.promises.readFile(modulePath, 'utf-8');
          const module = JSON.parse(content) as ModuleMetadata;
          modules.push(module);
        }
      }
    } catch (error) {
      console.error('Error reading modules:', error);
    }
    
    return modules;
  }

  /**
   * Get a specific module by name
   */
  async getModule(name: string): Promise<ModuleMetadata | null> {
    try {
      const modulePath = path.join(this.modulesPath, `${name}.json`);
      const content = await fs.promises.readFile(modulePath, 'utf-8');
      return JSON.parse(content) as ModuleMetadata;
    } catch (error) {
      console.error(`Error reading module ${name}:`, error);
      return null;
    }
  }

  /**
   * Get modules by category
   */
  async getModulesByCategory(category: string): Promise<ModuleMetadata[]> {
    const allModules = await this.getAllModules();
    return allModules.filter(module => module.category === category);
  }

  /**
   * Get modules by tags
   */
  async getModulesByTags(tags: string[]): Promise<ModuleMetadata[]> {
    const allModules = await this.getAllModules();
    return allModules.filter(module => 
      module.tags && tags.some(tag => module.tags!.includes(tag))
    );
  }

  /**
   * Validate that all referenced modules exist
   */
  async validateModuleReferences(moduleNames: string[]): Promise<{
    valid: string[];
    invalid: string[];
  }> {
    const valid: string[] = [];
    const invalid: string[] = [];
    
    for (const name of moduleNames) {
      const module = await this.getModule(name);
      if (module) {
        valid.push(name);
      } else {
        invalid.push(name);
      }
    }
    
    return { valid, invalid };
  }

  /**
   * Get all ROS packages from a list of modules
   */
  async getPackagesFromModules(moduleNames: string[]): Promise<string[]> {
    const packages: string[] = [];
    
    for (const name of moduleNames) {
      const module = await this.getModule(name);
      if (module) {
        packages.push(...module.packages);
      }
    }
    
    return [...new Set(packages)]; // Remove duplicates
  }
} 