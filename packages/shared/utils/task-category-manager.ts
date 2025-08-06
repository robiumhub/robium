import { TaskCategory, Algorithm, EXAMPLE_TASK_CATEGORIES, TaskCategoryId, AlgorithmId } from '../types/task-category';
import { MetaCategoryId } from '../types/meta-category';
import { MetaCategoryManager, metaCategoryManager } from './meta-category-manager';

export class TaskCategoryManager {
  private categories: Map<string, TaskCategory>;
  private metaCategoryManager: MetaCategoryManager;

  constructor() {
    this.categories = new Map();
    this.metaCategoryManager = metaCategoryManager;
    EXAMPLE_TASK_CATEGORIES.forEach(category => {
      this.categories.set(category.id, category);
    });
  }

  /**
   * Get all task categories
   */
  getAllCategories(): TaskCategory[] {
    return Array.from(this.categories.values());
  }

  /**
   * Get a specific task category by ID
   */
  getCategory(id: TaskCategoryId): TaskCategory | undefined {
    return this.categories.get(id);
  }

  /**
   * Get categories by meta-category
   */
  getCategoriesByMetaCategory(metaCategoryId: MetaCategoryId): TaskCategory[] {
    return this.getAllCategories().filter(category => 
      category.metaCategoryId === metaCategoryId
    );
  }

  /**
   * Get categories by hierarchy level
   */
  getCategoriesByLevel(level: number): TaskCategory[] {
    return this.getAllCategories().filter(category => 
      category.hierarchy.level === level
    );
  }

  /**
   * Get top-level categories (level 1)
   */
  getTopLevelCategories(): TaskCategory[] {
    return this.getCategoriesByLevel(1);
  }

  /**
   * Get child categories
   */
  getChildCategories(categoryId: TaskCategoryId): TaskCategory[] {
    const category = this.getCategory(categoryId);
    if (!category) return [];

    return category.hierarchy.children
      .map(childId => this.getCategory(childId as TaskCategoryId))
      .filter((cat): cat is TaskCategory => cat !== undefined);
  }

  /**
   * Get parent category
   */
  getParentCategory(categoryId: TaskCategoryId): TaskCategory | undefined {
    const category = this.getCategory(categoryId);
    if (!category || !category.hierarchy.parent) return undefined;

    return this.getCategory(category.hierarchy.parent as TaskCategoryId);
  }

  /**
   * Get categories by complexity
   */
  getCategoriesByComplexity(complexity: Algorithm['complexity']): TaskCategory[] {
    return this.getAllCategories().filter(category =>
      category.algorithms.some(algorithm => algorithm.complexity === complexity)
    );
  }

  /**
   * Get categories by algorithm status
   */
  getCategoriesByAlgorithmStatus(status: NonNullable<Algorithm['status']>): TaskCategory[] {
    return this.getAllCategories().filter(category =>
      category.algorithms.some(algorithm => algorithm.status === status)
    );
  }

  /**
   * Search categories by name or description
   */
  searchCategories(query: string): TaskCategory[] {
    const lowerQuery = query.toLowerCase();
    return this.getAllCategories().filter(category =>
      category.name.toLowerCase().includes(lowerQuery) ||
      category.description.toLowerCase().includes(lowerQuery)
    );
  }

  /**
   * Search algorithms by name, description, or tags
   */
  searchAlgorithms(query: string): { category: TaskCategory; algorithm: Algorithm }[] {
    const lowerQuery = query.toLowerCase();
    const results: { category: TaskCategory; algorithm: Algorithm }[] = [];

    this.getAllCategories().forEach(category => {
      category.algorithms.forEach(algorithm => {
        if (
          algorithm.name.toLowerCase().includes(lowerQuery) ||
          algorithm.description.toLowerCase().includes(lowerQuery) ||
          algorithm.tags?.some(tag => tag.toLowerCase().includes(lowerQuery))
        ) {
          results.push({ category, algorithm });
        }
      });
    });

    return results;
  }

  /**
   * Get algorithm by ID
   */
  getAlgorithm(algorithmId: AlgorithmId): { category: TaskCategory; algorithm: Algorithm } | undefined {
    for (const category of this.getAllCategories()) {
      const algorithm = category.algorithms.find(alg => alg.id === algorithmId);
      if (algorithm) {
        return { category, algorithm };
      }
    }
    return undefined;
  }

  /**
   * Get algorithms by complexity
   */
  getAlgorithmsByComplexity(complexity: Algorithm['complexity']): { category: TaskCategory; algorithm: Algorithm }[] {
    const results: { category: TaskCategory; algorithm: Algorithm }[] = [];

    this.getAllCategories().forEach(category => {
      category.algorithms.forEach(algorithm => {
        if (algorithm.complexity === complexity) {
          results.push({ category, algorithm });
        }
      });
    });

    return results;
  }

  /**
   * Get algorithms by status
   */
  getAlgorithmsByStatus(status: NonNullable<Algorithm['status']>): { category: TaskCategory; algorithm: Algorithm }[] {
    const results: { category: TaskCategory; algorithm: Algorithm }[] = [];

    this.getAllCategories().forEach(category => {
      category.algorithms.forEach(algorithm => {
        if (algorithm.status === status) {
          results.push({ category, algorithm });
        }
      });
    });

    return results;
  }

  /**
   * Get algorithms by tags
   */
  getAlgorithmsByTags(tags: string[]): { category: TaskCategory; algorithm: Algorithm }[] {
    const results: { category: TaskCategory; algorithm: Algorithm }[] = [];

    this.getAllCategories().forEach(category => {
      category.algorithms.forEach(algorithm => {
        if (algorithm.tags && tags.some(tag => algorithm.tags!.includes(tag))) {
          results.push({ category, algorithm });
        }
      });
    });

    return results;
  }

  /**
   * Get algorithms by ROS packages
   */
  getAlgorithmsByPackages(packages: string[]): { category: TaskCategory; algorithm: Algorithm }[] {
    const results: { category: TaskCategory; algorithm: Algorithm }[] = [];

    this.getAllCategories().forEach(category => {
      category.algorithms.forEach(algorithm => {
        if (packages.some(pkg => algorithm.packages.includes(pkg))) {
          results.push({ category, algorithm });
        }
      });
    });

    return results;
  }

  /**
   * Get hierarchical tree structure
   */
  getHierarchicalTree(): {
    metaCategory: string;
    categories: {
      category: TaskCategory;
      children: TaskCategory[];
    }[];
  }[] {
    const metaCategories = this.metaCategoryManager.getAllCategories();
    const tree: {
      metaCategory: string;
      categories: {
        category: TaskCategory;
        children: TaskCategory[];
      }[];
    }[] = [];

    metaCategories.forEach(metaCategory => {
      const categories = this.getCategoriesByMetaCategory(metaCategory.id);
      const topLevelCategories = categories.filter(cat => cat.hierarchy.level === 1);

      const categoryTree = topLevelCategories.map(category => ({
        category,
        children: this.getChildCategories(category.id)
      }));

      tree.push({
        metaCategory: metaCategory.name,
        categories: categoryTree
      });
    });

    return tree;
  }

  /**
   * Validate category hierarchy
   */
  validateHierarchy(): { valid: boolean; errors: string[] } {
    const errors: string[] = [];
    const categories = this.getAllCategories();

    // Check for circular references
    const visited = new Set<string>();
    const recursionStack = new Set<string>();

    const hasCycle = (categoryId: string): boolean => {
      if (recursionStack.has(categoryId)) {
        errors.push(`Circular reference detected in category: ${categoryId}`);
        return true;
      }

      if (visited.has(categoryId)) {
        return false;
      }

      visited.add(categoryId);
      recursionStack.add(categoryId);

      const category = this.getCategory(categoryId as TaskCategoryId);
      if (category && category.hierarchy.parent) {
        if (hasCycle(category.hierarchy.parent)) {
          return true;
        }
      }

      recursionStack.delete(categoryId);
      return false;
    };

    categories.forEach(category => {
      if (!visited.has(category.id)) {
        hasCycle(category.id);
      }
    });

    // Check for orphaned categories
    categories.forEach(category => {
      if (category.hierarchy.parent) {
        const parent = this.getCategory(category.hierarchy.parent as TaskCategoryId);
        if (!parent) {
          errors.push(`Orphaned category: ${category.id} (parent: ${category.hierarchy.parent} not found)`);
        }
      }
    });

    // Check for missing child references
    categories.forEach(category => {
      category.hierarchy.children.forEach(childId => {
        const child = this.getCategory(childId as TaskCategoryId);
        if (!child) {
          errors.push(`Missing child category: ${childId} referenced by ${category.id}`);
        }
      });
    });

    return {
      valid: errors.length === 0,
      errors
    };
  }

  /**
   * Get category statistics
   */
  getStatistics(): {
    totalCategories: number;
    totalAlgorithms: number;
    byMetaCategory: Record<string, number>;
    byComplexity: Record<Algorithm['complexity'], number>;
    byStatus: Record<NonNullable<Algorithm['status']>, number>;
    byLevel: Record<number, number>;
  } {
    const categories = this.getAllCategories();
    const algorithms = categories.flatMap(cat => cat.algorithms);

    const byMetaCategory: Record<string, number> = {};
    const byComplexity: Record<Algorithm['complexity'], number> = {
      basic: 0,
      intermediate: 0,
      advanced: 0,
      expert: 0
    };
    const byStatus: Record<NonNullable<Algorithm['status']>, number> = {
      stable: 0,
      beta: 0,
      experimental: 0,
      deprecated: 0
    };
    const byLevel: Record<number, number> = {};

    categories.forEach(category => {
      byMetaCategory[category.metaCategoryId] = (byMetaCategory[category.metaCategoryId] || 0) + 1;
      byLevel[category.hierarchy.level] = (byLevel[category.hierarchy.level] || 0) + 1;
    });

    algorithms.forEach(algorithm => {
      byComplexity[algorithm.complexity]++;
      if (algorithm.status) {
        byStatus[algorithm.status]++;
      }
    });

    return {
      totalCategories: categories.length,
      totalAlgorithms: algorithms.length,
      byMetaCategory,
      byComplexity,
      byStatus,
      byLevel
    };
  }

  /**
   * Export categories to JSON
   */
  exportToJson(): string {
    return JSON.stringify(this.getAllCategories(), null, 2);
  }

  /**
   * Import categories from JSON
   */
  importFromJson(json: string): void {
    try {
      const categories = JSON.parse(json) as TaskCategory[];
      this.categories.clear();
      categories.forEach(category => {
        this.categories.set(category.id, category);
      });
    } catch (error) {
      throw new Error(`Failed to import task categories: ${error}`);
    }
  }

  /**
   * Add a new task category
   */
  addCategory(category: TaskCategory): void {
    if (this.categories.has(category.id)) {
      throw new Error(`Task category with ID ${category.id} already exists`);
    }
    this.categories.set(category.id, category);
  }

  /**
   * Update an existing task category
   */
  updateCategory(category: TaskCategory): void {
    if (!this.categories.has(category.id)) {
      throw new Error(`Task category with ID ${category.id} does not exist`);
    }
    this.categories.set(category.id, category);
  }

  /**
   * Remove a task category
   */
  removeCategory(categoryId: TaskCategoryId): void {
    if (!this.categories.has(categoryId)) {
      throw new Error(`Task category with ID ${categoryId} does not exist`);
    }
    this.categories.delete(categoryId);
  }
}

// Export singleton instance
export const taskCategoryManager = new TaskCategoryManager(); 