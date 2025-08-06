import { MetaCategory, ROS_META_CATEGORIES, MetaCategoryId, MetaCategoryScope, MetaCategoryComplexity } from '../types/meta-category';

export class MetaCategoryManager {
  private categories: Map<string, MetaCategory>;

  constructor() {
    this.categories = new Map();
    ROS_META_CATEGORIES.forEach(category => {
      this.categories.set(category.id, category);
    });
  }

  /**
   * Get all meta-categories
   */
  getAllCategories(): MetaCategory[] {
    return Array.from(this.categories.values());
  }

  /**
   * Get a specific meta-category by ID
   */
  getCategory(id: MetaCategoryId): MetaCategory | undefined {
    return this.categories.get(id);
  }

  /**
   * Get categories by scope
   */
  getCategoriesByScope(scope: MetaCategoryScope): MetaCategory[] {
    return this.getAllCategories().filter(category => category.scope === scope);
  }

  /**
   * Get categories by complexity level
   */
  getCategoriesByComplexity(complexity: MetaCategoryComplexity): MetaCategory[] {
    return this.getAllCategories().filter(category => 
      category.classification.complexity === complexity
    );
  }

  /**
   * Get categories by robot type
   */
  getCategoriesByRobotType(robotType: string): MetaCategory[] {
    return this.getAllCategories().filter(category =>
      category.classification.robotType.includes(robotType)
    );
  }

  /**
   * Get categories by use case
   */
  getCategoriesByUseCase(useCase: string): MetaCategory[] {
    return this.getAllCategories().filter(category =>
      category.classification.useCase.includes(useCase)
    );
  }

  /**
   * Search categories by keyword
   */
  searchCategories(query: string): MetaCategory[] {
    const lowerQuery = query.toLowerCase();
    return this.getAllCategories().filter(category =>
      category.name.toLowerCase().includes(lowerQuery) ||
      category.description.toLowerCase().includes(lowerQuery) ||
      category.taxonomy.keywords.some(keyword => 
        keyword.toLowerCase().includes(lowerQuery)
      ) ||
      category.taxonomy.aliases.some(alias => 
        alias.toLowerCase().includes(lowerQuery)
      )
    );
  }

  /**
   * Get related categories
   */
  getRelatedCategories(categoryId: MetaCategoryId): MetaCategory[] {
    const category = this.getCategory(categoryId);
    if (!category) return [];

    return category.taxonomy.relatedCategories
      .map(relatedId => this.getCategory(relatedId as MetaCategoryId))
      .filter((cat): cat is MetaCategory => cat !== undefined);
  }

  /**
   * Get child categories
   */
  getChildCategories(categoryId: MetaCategoryId): MetaCategory[] {
    const category = this.getCategory(categoryId);
    if (!category) return [];

    return category.hierarchy.children
      .map(childId => this.getCategory(childId as MetaCategoryId))
      .filter((cat): cat is MetaCategory => cat !== undefined);
  }

  /**
   * Get parent category
   */
  getParentCategory(categoryId: MetaCategoryId): MetaCategory | undefined {
    const category = this.getCategory(categoryId);
    if (!category || !category.hierarchy.parent) return undefined;

    return this.getCategory(category.hierarchy.parent as MetaCategoryId);
  }

  /**
   * Get categories by hierarchy level
   */
  getCategoriesByLevel(level: number): MetaCategory[] {
    return this.getAllCategories().filter(category => 
      category.hierarchy.level === level
    );
  }

  /**
   * Get top-level categories (level 1)
   */
  getTopLevelCategories(): MetaCategory[] {
    return this.getCategoriesByLevel(1);
  }

  /**
   * Validate category compatibility with ROS version
   */
  validateRosVersionCompatibility(categoryId: MetaCategoryId, rosVersion: string): boolean {
    const category = this.getCategory(categoryId);
    if (!category || !category.constraints) return true; // No constraints means compatible

    return category.constraints.rosVersion.includes(rosVersion);
  }

  /**
   * Get hardware requirements for a category
   */
  getHardwareRequirements(categoryId: MetaCategoryId): string[] {
    const category = this.getCategory(categoryId);
    return category?.constraints?.hardwareRequirements || [];
  }

  /**
   * Get external dependencies for a category
   */
  getExternalDependencies(categoryId: MetaCategoryId): string[] {
    const category = this.getCategory(categoryId);
    return category?.constraints?.dependencies || [];
  }

  /**
   * Filter categories by multiple criteria
   */
  filterCategories(filters: {
    scope?: MetaCategoryScope;
    complexity?: MetaCategoryComplexity;
    robotType?: string;
    useCase?: string;
    level?: number;
    rosVersion?: string;
  }): MetaCategory[] {
    let categories = this.getAllCategories();

    if (filters.scope) {
      categories = categories.filter(cat => cat.scope === filters.scope);
    }

    if (filters.complexity) {
      categories = categories.filter(cat => cat.classification.complexity === filters.complexity);
    }

    if (filters.robotType) {
      categories = categories.filter(cat => 
        cat.classification.robotType.includes(filters.robotType!)
      );
    }

    if (filters.useCase) {
      categories = categories.filter(cat => 
        cat.classification.useCase.includes(filters.useCase!)
      );
    }

    if (filters.level) {
      categories = categories.filter(cat => cat.hierarchy.level === filters.level);
    }

    if (filters.rosVersion) {
      categories = categories.filter(cat => 
        this.validateRosVersionCompatibility(cat.id, filters.rosVersion!)
      );
    }

    return categories;
  }

  /**
   * Get category statistics
   */
  getStatistics(): {
    total: number;
    byScope: Record<MetaCategoryScope, number>;
    byComplexity: Record<MetaCategoryComplexity, number>;
    byLevel: Record<number, number>;
  } {
    const categories = this.getAllCategories();
    const byScope: Record<MetaCategoryScope, number> = {
      core: 0,
      advanced: 0,
      experimental: 0,
      deprecated: 0
    };
    const byComplexity: Record<MetaCategoryComplexity, number> = {
      basic: 0,
      intermediate: 0,
      advanced: 0,
      expert: 0
    };
    const byLevel: Record<number, number> = {};

    categories.forEach(category => {
      byScope[category.scope]++;
      byComplexity[category.classification.complexity]++;
      byLevel[category.hierarchy.level] = (byLevel[category.hierarchy.level] || 0) + 1;
    });

    return {
      total: categories.length,
      byScope,
      byComplexity,
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
      const categories = JSON.parse(json) as MetaCategory[];
      this.categories.clear();
      categories.forEach(category => {
        this.categories.set(category.id, category);
      });
    } catch (error) {
      throw new Error(`Failed to import categories: ${error}`);
    }
  }
}

// Export singleton instance
export const metaCategoryManager = new MetaCategoryManager(); 