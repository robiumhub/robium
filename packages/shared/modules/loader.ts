import fs from 'fs/promises';
import path from 'path';
import glob from 'glob';
import {
  ModuleSpec,
  RobotSpec,
  DistroSpec,
  ModuleRegistry,
} from '../types/ros-modules';

export class ModuleLoader {
  private modules: Map<string, ModuleSpec> = new Map();
  private robots: Map<string, RobotSpec> = new Map();
  private distros: Map<string, DistroSpec> = new Map();
  private registry: ModuleRegistry | null = null;

  async loadModules(): Promise<void> {
    const moduleFiles = await glob('packages/shared/modules/**/*.json');

    for (const file of moduleFiles) {
      if (file.endsWith('registry.json')) continue; // Skip registry file

      const moduleSpec = await this.loadModuleFromFile(file);

      if (moduleSpec.category === 'robots') {
        this.robots.set(moduleSpec.id, moduleSpec as RobotSpec);
      } else {
        this.modules.set(moduleSpec.id, moduleSpec);
      }
    }
  }

  private async loadModuleFromFile(filePath: string): Promise<ModuleSpec> {
    const content = await fs.readFile(filePath, 'utf-8');
    return JSON.parse(content);
  }

  getModule(id: string): ModuleSpec | undefined {
    return this.modules.get(id) || this.robots.get(id);
  }

  getRobot(id: string): RobotSpec | undefined {
    return this.robots.get(id);
  }

  getAllModules(): ModuleSpec[] {
    return Array.from(this.modules.values());
  }

  getAllRobots(): RobotSpec[] {
    return Array.from(this.robots.values());
  }

  getModulesByCategory(category: string): ModuleSpec[] {
    return this.getAllModules().filter((m) => m.category === category);
  }

  getModulesByDistro(distro: string): ModuleSpec[] {
    return this.getAllModules().filter((m) => m.rosDistros.includes(distro));
  }

  getRobotsByDistro(distro: string): RobotSpec[] {
    return this.getAllRobots().filter((r) => r.rosDistros.includes(distro));
  }

  getCompatibleModules(robotId: string): ModuleSpec[] {
    const robot = this.getRobot(robotId);
    if (!robot) return [];

    return this.getAllModules().filter(
      (module) =>
        robot.supportedModules.includes(module.id) &&
        module.rosDistros.some((distro) => robot.rosDistros.includes(distro))
    );
  }

  resolveDependencies(moduleIds: string[]): string[] {
    const resolved: string[] = [];
    const visited = new Set<string>();

    const visit = (moduleId: string) => {
      if (visited.has(moduleId)) return;
      visited.add(moduleId);

      const module = this.getModule(moduleId);
      if (!module) return;

      // Visit dependencies first
      if (module.dependencies) {
        for (const dep of module.dependencies) {
          visit(dep);
        }
      }

      resolved.push(moduleId);
    };

    for (const moduleId of moduleIds) {
      visit(moduleId);
    }

    return resolved;
  }

  getRegistry(): ModuleRegistry {
    if (!this.registry) {
      this.registry = {
        distros: Array.from(this.distros.values()),
        robots: Array.from(this.robots.values()),
        modules: Array.from(this.modules.values()),
      };
    }
    return this.registry;
  }
}

