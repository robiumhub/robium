import {
  MetaCategoryManager,
  metaCategoryManager,
} from '../utils/meta-category-manager';
import { ROS_META_CATEGORIES } from '../types/meta-category';

describe('MetaCategoryManager', () => {
  let manager: MetaCategoryManager;

  beforeEach(() => {
    manager = new MetaCategoryManager();
  });

  describe('Initialization', () => {
    it('should initialize with all predefined categories', () => {
      const categories = manager.getAllCategories();
      expect(categories).toHaveLength(ROS_META_CATEGORIES.length);
    });

    it('should have unique IDs for all categories', () => {
      const categories = manager.getAllCategories();
      const ids = categories.map((cat) => cat.id);
      const uniqueIds = new Set(ids);
      expect(uniqueIds.size).toBe(ids.length);
    });
  });

  describe('Category Retrieval', () => {
    it('should get category by ID', () => {
      const category = manager.getCategory('actuator');
      expect(category).toBeDefined();
      expect(category?.name).toBe('Actuator Control');
    });

    it('should return undefined for non-existent category', () => {
      const category = manager.getCategory('non_existent' as any);
      expect(category).toBeUndefined();
    });

    it('should get all categories', () => {
      const categories = manager.getAllCategories();
      expect(categories).toBeInstanceOf(Array);
      expect(categories.length).toBeGreaterThan(0);
    });
  });

  describe('Filtering by Scope', () => {
    it('should filter categories by core scope', () => {
      const coreCategories = manager.getCategoriesByScope('core');
      expect(coreCategories.every((cat) => cat.scope === 'core')).toBe(true);
    });

    it('should filter categories by advanced scope', () => {
      const advancedCategories = manager.getCategoriesByScope('advanced');
      expect(advancedCategories.every((cat) => cat.scope === 'advanced')).toBe(
        true
      );
    });

    it('should filter categories by experimental scope', () => {
      const experimentalCategories =
        manager.getCategoriesByScope('experimental');
      expect(
        experimentalCategories.every((cat) => cat.scope === 'experimental')
      ).toBe(true);
    });
  });

  describe('Filtering by Complexity', () => {
    it('should filter categories by basic complexity', () => {
      const basicCategories = manager.getCategoriesByComplexity('basic');
      expect(
        basicCategories.every(
          (cat) => cat.classification.complexity === 'basic'
        )
      ).toBe(true);
    });

    it('should filter categories by advanced complexity', () => {
      const advancedCategories = manager.getCategoriesByComplexity('advanced');
      expect(
        advancedCategories.every(
          (cat) => cat.classification.complexity === 'advanced'
        )
      ).toBe(true);
    });
  });

  describe('Filtering by Robot Type', () => {
    it('should filter categories by mobile robot type', () => {
      const mobileCategories = manager.getCategoriesByRobotType('mobile');
      expect(
        mobileCategories.every((cat) =>
          cat.classification.robotType.includes('mobile')
        )
      ).toBe(true);
    });

    it('should filter categories by manipulator robot type', () => {
      const manipulatorCategories =
        manager.getCategoriesByRobotType('manipulator');
      expect(
        manipulatorCategories.every((cat) =>
          cat.classification.robotType.includes('manipulator')
        )
      ).toBe(true);
    });
  });

  describe('Search Functionality', () => {
    it('should search categories by name', () => {
      const results = manager.searchCategories('Actuator');
      expect(results.some((cat) => cat.name === 'Actuator Control')).toBe(true);
    });

    it('should search categories by description', () => {
      const results = manager.searchCategories('camera control');
      expect(results.some((cat) => cat.id === 'camera')).toBe(true);
    });

    it('should search categories by keywords', () => {
      const results = manager.searchCategories('vision');
      expect(results.some((cat) => cat.id === 'camera')).toBe(true);
    });

    it('should search categories by aliases', () => {
      const results = manager.searchCategories('teleoperation');
      expect(results.some((cat) => cat.id === 'remote_control')).toBe(true);
    });

    it('should return empty array for non-matching search', () => {
      const results = manager.searchCategories('nonexistent');
      expect(results).toHaveLength(0);
    });
  });

  describe('Hierarchy Management', () => {
    it('should get top-level categories', () => {
      const topLevel = manager.getTopLevelCategories();
      expect(topLevel.every((cat) => cat.hierarchy.level === 1)).toBe(true);
    });

    it('should get categories by level', () => {
      const level2Categories = manager.getCategoriesByLevel(2);
      expect(level2Categories.every((cat) => cat.hierarchy.level === 2)).toBe(
        true
      );
    });

    it('should get child categories', () => {
      const children = manager.getChildCategories('actuator');
      expect(children).toBeInstanceOf(Array);
    });

    it('should get related categories', () => {
      const related = manager.getRelatedCategories('actuator');
      expect(related).toBeInstanceOf(Array);
    });
  });

  describe('Compatibility Validation', () => {
    it('should validate ROS version compatibility', () => {
      const isCompatible = manager.validateRosVersionCompatibility(
        'actuator',
        'humble'
      );
      expect(typeof isCompatible).toBe('boolean');
    });

    it('should get hardware requirements', () => {
      const requirements = manager.getHardwareRequirements('actuator');
      expect(requirements).toBeInstanceOf(Array);
    });

    it('should get external dependencies', () => {
      const dependencies = manager.getExternalDependencies('actuator');
      expect(dependencies).toBeInstanceOf(Array);
    });
  });

  describe('Advanced Filtering', () => {
    it('should filter by multiple criteria', () => {
      const filtered = manager.filterCategories({
        scope: 'core',
        complexity: 'basic',
        robotType: 'mobile',
      });
      expect(filtered).toBeInstanceOf(Array);
    });

    it('should handle empty filters', () => {
      const filtered = manager.filterCategories({});
      expect(filtered).toHaveLength(manager.getAllCategories().length);
    });
  });

  describe('Statistics', () => {
    it('should return valid statistics', () => {
      const stats = manager.getStatistics();
      expect(stats.total).toBeGreaterThan(0);
      expect(stats.byScope).toBeDefined();
      expect(stats.byComplexity).toBeDefined();
      expect(stats.byLevel).toBeDefined();
    });

    it('should have correct total count', () => {
      const stats = manager.getStatistics();
      const totalFromCategories = manager.getAllCategories().length;
      expect(stats.total).toBe(totalFromCategories);
    });
  });

  describe('Import/Export', () => {
    it('should export to JSON', () => {
      const json = manager.exportToJson();
      expect(json).toBeDefined();
      expect(typeof json).toBe('string');
    });

    it('should import from JSON', () => {
      const originalCategories = manager.getAllCategories();
      const json = manager.exportToJson();

      manager.importFromJson(json);
      const importedCategories = manager.getAllCategories();

      expect(importedCategories).toHaveLength(originalCategories.length);
    });

    it('should throw error for invalid JSON', () => {
      expect(() => {
        manager.importFromJson('invalid json');
      }).toThrow();
    });
  });

  describe('Singleton Instance', () => {
    it('should provide singleton instance', () => {
      expect(metaCategoryManager).toBeInstanceOf(MetaCategoryManager);
    });

    it('should have same categories as new instance', () => {
      const newManager = new MetaCategoryManager();
      expect(metaCategoryManager.getAllCategories()).toHaveLength(
        newManager.getAllCategories().length
      );
    });
  });
});
