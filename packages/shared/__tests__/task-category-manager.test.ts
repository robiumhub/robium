import { TaskCategoryManager, taskCategoryManager } from '../utils/task-category-manager';
import { TaskCategory, EXAMPLE_TASK_CATEGORIES } from '../types/task-category';

describe('TaskCategoryManager', () => {
  let manager: TaskCategoryManager;

  beforeEach(() => {
    manager = new TaskCategoryManager();
  });

  describe('Initialization', () => {
    it('should initialize with all predefined task categories', () => {
      const categories = manager.getAllCategories();
      expect(categories).toHaveLength(EXAMPLE_TASK_CATEGORIES.length);
    });

    it('should have unique IDs for all categories', () => {
      const categories = manager.getAllCategories();
      const ids = categories.map(cat => cat.id);
      const uniqueIds = new Set(ids);
      expect(uniqueIds.size).toBe(ids.length);
    });
  });

  describe('Category Retrieval', () => {
    it('should get category by ID', () => {
      const category = manager.getCategory('motor_control');
      expect(category).toBeDefined();
      expect(category?.name).toBe('Motor Control');
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

  describe('Filtering by Meta-Category', () => {
    it('should filter categories by actuator meta-category', () => {
      const actuatorCategories = manager.getCategoriesByMetaCategory('actuator');
      expect(actuatorCategories.every(cat => cat.metaCategoryId === 'actuator')).toBe(true);
    });

    it('should filter categories by camera meta-category', () => {
      const cameraCategories = manager.getCategoriesByMetaCategory('camera');
      expect(cameraCategories.every(cat => cat.metaCategoryId === 'camera')).toBe(true);
    });
  });

  describe('Hierarchy Management', () => {
    it('should get top-level categories', () => {
      const topLevel = manager.getTopLevelCategories();
      expect(topLevel.every(cat => cat.hierarchy.level === 1)).toBe(true);
    });

    it('should get categories by level', () => {
      const level2Categories = manager.getCategoriesByLevel(2);
      expect(level2Categories.every(cat => cat.hierarchy.level === 2)).toBe(true);
    });

    it('should get child categories', () => {
      const children = manager.getChildCategories('motor_control');
      expect(children).toBeInstanceOf(Array);
    });

    it('should get parent category', () => {
      const parent = manager.getParentCategory('dc_motor_control');
      expect(parent?.id).toBe('motor_control');
    });
  });

  describe('Algorithm Management', () => {
    it('should get algorithm by ID', () => {
      const result = manager.getAlgorithm('pid_motor_control');
      expect(result).toBeDefined();
      expect(result?.algorithm.name).toBe('PID Motor Control');
    });

    it('should return undefined for non-existent algorithm', () => {
      const result = manager.getAlgorithm('non_existent');
      expect(result).toBeUndefined();
    });

    it('should get algorithms by complexity', () => {
      const basicAlgorithms = manager.getAlgorithmsByComplexity('basic');
      expect(basicAlgorithms.every(result => result.algorithm.complexity === 'basic')).toBe(true);
    });

    it('should get algorithms by status', () => {
      const stableAlgorithms = manager.getAlgorithmsByStatus('stable');
      expect(stableAlgorithms.every(result => result.algorithm.status === 'stable')).toBe(true);
    });

    it('should get algorithms by tags', () => {
      const pidAlgorithms = manager.getAlgorithmsByTags(['pid']);
      expect(pidAlgorithms.length).toBeGreaterThan(0);
    });

    it('should get algorithms by packages', () => {
      const nav2Algorithms = manager.getAlgorithmsByPackages(['nav2_planner']);
      expect(nav2Algorithms.length).toBeGreaterThan(0);
    });
  });

  describe('Search Functionality', () => {
    it('should search categories by name', () => {
      const results = manager.searchCategories('Motor');
      expect(results.some(cat => cat.name === 'Motor Control')).toBe(true);
    });

    it('should search categories by description', () => {
      const results = manager.searchCategories('camera calibration');
      expect(results.some(cat => cat.id === 'camera_calibration')).toBe(true);
    });

    it('should search algorithms by name', () => {
      const results = manager.searchAlgorithms('PID');
      expect(results.some(result => result.algorithm.name === 'PID Motor Control')).toBe(true);
    });

    it('should search algorithms by description', () => {
      const results = manager.searchAlgorithms('proportional');
      expect(results.some(result => result.algorithm.id === 'pid_motor_control')).toBe(true);
    });

    it('should search algorithms by tags', () => {
      const results = manager.searchAlgorithms('pid');
      expect(results.some(result => result.algorithm.id === 'pid_motor_control')).toBe(true);
    });
  });

  describe('Hierarchical Tree', () => {
    it('should return hierarchical tree structure', () => {
      const tree = manager.getHierarchicalTree();
      expect(tree).toBeInstanceOf(Array);
      expect(tree.length).toBeGreaterThan(0);
    });

    it('should have meta-categories in tree', () => {
      const tree = manager.getHierarchicalTree();
      const metaCategoryNames = tree.map(item => item.metaCategory);
      expect(metaCategoryNames).toContain('Actuator Control');
    });
  });

  describe('Validation', () => {
    it('should validate hierarchy without errors', () => {
      const validation = manager.validateHierarchy();
      expect(validation.valid).toBe(true);
      expect(validation.errors).toHaveLength(0);
    });
  });

  describe('Statistics', () => {
    it('should return valid statistics', () => {
      const stats = manager.getStatistics();
      expect(stats.totalCategories).toBeGreaterThan(0);
      expect(stats.totalAlgorithms).toBeGreaterThan(0);
      expect(stats.byMetaCategory).toBeDefined();
      expect(stats.byComplexity).toBeDefined();
      expect(stats.byStatus).toBeDefined();
      expect(stats.byLevel).toBeDefined();
    });

    it('should have correct total counts', () => {
      const stats = manager.getStatistics();
      const totalCategories = manager.getAllCategories().length;
      const totalAlgorithms = manager.getAllCategories().flatMap(cat => cat.algorithms).length;
      expect(stats.totalCategories).toBe(totalCategories);
      expect(stats.totalAlgorithms).toBe(totalAlgorithms);
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

  describe('Category Management', () => {
    it('should add new category', () => {
      const newCategory: TaskCategory = {
        id: 'test_category',
        metaCategoryId: 'actuator',
        name: 'Test Category',
        description: 'Test category for testing',
        icon: 'test_icon',
        hierarchy: {
          level: 1,
          children: [],
          order: 1
        },
        algorithms: []
      };

      manager.addCategory(newCategory);
      const category = manager.getCategory('test_category');
      expect(category).toBeDefined();
      expect(category?.name).toBe('Test Category');
    });

    it('should throw error when adding duplicate category', () => {
      const newCategory: TaskCategory = {
        id: 'motor_control', // Already exists
        metaCategoryId: 'actuator',
        name: 'Duplicate Category',
        description: 'Duplicate category',
        icon: 'test_icon',
        hierarchy: {
          level: 1,
          children: [],
          order: 1
        },
        algorithms: []
      };

      expect(() => {
        manager.addCategory(newCategory);
      }).toThrow();
    });

    it('should update existing category', () => {
      const category = manager.getCategory('motor_control');
      if (category) {
        const updatedCategory = { ...category, name: 'Updated Motor Control' };
        manager.updateCategory(updatedCategory);
        
        const updated = manager.getCategory('motor_control');
        expect(updated?.name).toBe('Updated Motor Control');
      }
    });

    it('should throw error when updating non-existent category', () => {
      const nonExistentCategory: TaskCategory = {
        id: 'non_existent',
        metaCategoryId: 'actuator',
        name: 'Non Existent',
        description: 'Non existent category',
        icon: 'test_icon',
        hierarchy: {
          level: 1,
          children: [],
          order: 1
        },
        algorithms: []
      };

      expect(() => {
        manager.updateCategory(nonExistentCategory);
      }).toThrow();
    });

    it('should remove category', () => {
      // First add a test category
      const testCategory: TaskCategory = {
        id: 'remove_test',
        metaCategoryId: 'actuator',
        name: 'Remove Test',
        description: 'Test category for removal',
        icon: 'test_icon',
        hierarchy: {
          level: 1,
          children: [],
          order: 1
        },
        algorithms: []
      };

      manager.addCategory(testCategory);
      expect(manager.getCategory('remove_test')).toBeDefined();

      manager.removeCategory('remove_test');
      expect(manager.getCategory('remove_test')).toBeUndefined();
    });

    it('should throw error when removing non-existent category', () => {
      expect(() => {
        manager.removeCategory('non_existent' as any);
      }).toThrow();
    });
  });

  describe('Singleton Instance', () => {
    it('should provide singleton instance', () => {
      expect(taskCategoryManager).toBeInstanceOf(TaskCategoryManager);
    });

    it('should have same categories as new instance', () => {
      const newManager = new TaskCategoryManager();
      expect(taskCategoryManager.getAllCategories()).toHaveLength(
        newManager.getAllCategories().length
      );
    });
  });
}); 