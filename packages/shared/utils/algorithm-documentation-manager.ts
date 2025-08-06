import { readFileSync, writeFileSync, existsSync, mkdirSync } from 'fs';
import { join, dirname } from 'path';
import {
  AlgorithmDocumentation,
  AlgorithmSearchCriteria,
  AlgorithmSearchResult,
  ValidationResult,
  AlgorithmDocumentationTemplate,
  AlgorithmId,
} from '../types/algorithm-documentation';

/**
 * Manager class for algorithm documentation operations
 * Provides validation, search, CRUD operations, and template management
 */
export class AlgorithmDocumentationManager {
  private algorithms: Map<AlgorithmId, AlgorithmDocumentation> = new Map();
  private dataPath: string;

  constructor(dataPath: string = './data/algorithms') {
    this.dataPath = dataPath;
    this.ensureDataDirectory();
  }

  /**
   * Ensure the data directory exists
   */
  private ensureDataDirectory(): void {
    if (!existsSync(this.dataPath)) {
      mkdirSync(this.dataPath, { recursive: true });
    }
  }

  /**
   * Load algorithm documentation from file
   */
  loadFromFile(algorithmId: AlgorithmId): AlgorithmDocumentation | null {
    const filePath = join(this.dataPath, `${algorithmId}.json`);
    
    if (!existsSync(filePath)) {
      return null;
    }

    try {
      const data = readFileSync(filePath, 'utf-8');
      const algorithm = JSON.parse(data) as AlgorithmDocumentation;
      this.algorithms.set(algorithmId, algorithm);
      return algorithm;
    } catch (error) {
      console.error(`Error loading algorithm ${algorithmId}:`, error);
      return null;
    }
  }

  /**
   * Load all algorithm documentation from directory
   */
  loadAll(): AlgorithmDocumentation[] {
    const algorithms: AlgorithmDocumentation[] = [];
    
    try {
      const files = require('fs').readdirSync(this.dataPath);
      
      for (const file of files) {
        if (file.endsWith('.json')) {
          const algorithmId = file.replace('.json', '');
          const algorithm = this.loadFromFile(algorithmId);
          if (algorithm) {
            algorithms.push(algorithm);
          }
        }
      }
    } catch (error) {
      console.error('Error loading algorithms:', error);
    }

    return algorithms;
  }

  /**
   * Save algorithm documentation to file
   */
  saveToFile(algorithm: AlgorithmDocumentation): boolean {
    try {
      const filePath = join(this.dataPath, `${algorithm.id}.json`);
      const data = JSON.stringify(algorithm, null, 2);
      writeFileSync(filePath, data, 'utf-8');
      this.algorithms.set(algorithm.id, algorithm);
      return true;
    } catch (error) {
      console.error(`Error saving algorithm ${algorithm.id}:`, error);
      return false;
    }
  }

  /**
   * Validate algorithm documentation against schema
   */
  validate(algorithm: AlgorithmDocumentation): ValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Basic validation
    if (!algorithm.id || algorithm.id.trim() === '') {
      errors.push('Algorithm ID is required');
    }

    if (!algorithm.name || algorithm.name.trim() === '') {
      errors.push('Algorithm name is required');
    }

    if (!algorithm.version || algorithm.version.trim() === '') {
      errors.push('Algorithm version is required');
    }

    // Task definition validation
    if (!algorithm.taskDefinition) {
      errors.push('Task definition is required');
    } else {
      if (!algorithm.taskDefinition.title) {
        errors.push('Task definition title is required');
      }
      if (!algorithm.taskDefinition.description) {
        errors.push('Task definition description is required');
      }
      if (!algorithm.taskDefinition.problemStatement) {
        errors.push('Task definition problem statement is required');
      }
    }

    // Input/Output specification validation
    if (!algorithm.inputOutputSpecification) {
      errors.push('Input/Output specification is required');
    } else {
      if (!Array.isArray(algorithm.inputOutputSpecification.inputs)) {
        errors.push('Input specification must be an array');
      }
      if (!Array.isArray(algorithm.inputOutputSpecification.outputs)) {
        errors.push('Output specification must be an array');
      }
    }

    // Parameters validation
    if (!algorithm.parameters) {
      errors.push('Parameters specification is required');
    } else {
      if (!Array.isArray(algorithm.parameters.configurable)) {
        errors.push('Configurable parameters must be an array');
      }
    }

    // Dependencies validation
    if (!algorithm.dependencies) {
      errors.push('Dependencies specification is required');
    } else {
      if (!Array.isArray(algorithm.dependencies.rosPackages)) {
        errors.push('ROS packages must be an array');
      }
    }

    // Metadata validation
    if (!algorithm.metadata) {
      errors.push('Metadata is required');
    } else {
      if (!algorithm.metadata.createdAt) {
        errors.push('Creation timestamp is required');
      }
      if (!algorithm.metadata.updatedAt) {
        errors.push('Update timestamp is required');
      }
      if (!algorithm.metadata.status) {
        errors.push('Status is required');
      }
    }

    // Warning checks
    if (!algorithm.performance) {
      warnings.push('Performance metrics are not specified');
    }

    if (!algorithm.implementation) {
      warnings.push('Implementation details are not specified');
    }

    if (!algorithm.testing) {
      warnings.push('Testing information is not specified');
    }

    if (!algorithm.documentation) {
      warnings.push('Documentation resources are not specified');
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
    };
  }

  /**
   * Create new algorithm documentation from template
   */
  createFromTemplate(
    algorithmId: AlgorithmId,
    template: AlgorithmDocumentationTemplate
  ): AlgorithmDocumentation | null {
    const now = new Date().toISOString();
    
    const algorithm: AlgorithmDocumentation = {
      id: algorithmId,
      name: (template.metadata as any)?.name || algorithmId,
      version: template.metadata?.version || '1.0.0',
      taskDefinition: {
        title: template.taskDefinition?.title || '',
        description: template.taskDefinition?.description || '',
        problemStatement: template.taskDefinition?.problemStatement || '',
        useCases: template.taskDefinition?.useCases,
        assumptions: template.taskDefinition?.assumptions,
        limitations: template.taskDefinition?.limitations,
      },
      inputOutputSpecification: {
        inputs: template.inputOutputSpecification?.inputs || [],
        outputs: template.inputOutputSpecification?.outputs || [],
        sideEffects: template.inputOutputSpecification?.sideEffects,
      },
      parameters: {
        configurable: template.parameters?.configurable || [],
        advanced: template.parameters?.advanced,
      },
      dependencies: {
        rosPackages: template.dependencies?.rosPackages || [],
        systemLibraries: template.dependencies?.systemLibraries,
        pythonPackages: template.dependencies?.pythonPackages,
        algorithms: template.dependencies?.algorithms,
      },
      performance: template.performance,
      implementation: template.implementation as any,
      testing: template.testing,
      documentation: template.documentation,
      metadata: {
        createdAt: now,
        updatedAt: now,
        version: template.metadata?.version,
        maintainer: template.metadata?.maintainer,
        license: template.metadata?.license,
        status: template.metadata?.status || 'experimental',
        tags: template.metadata?.tags,
        categories: template.metadata?.categories,
      },
    };

    // Validate the created algorithm
    const validation = this.validate(algorithm);
    if (!validation.isValid) {
      console.error('Invalid algorithm template:', validation.errors);
      return null;
    }

    return algorithm;
  }

  /**
   * Search algorithms based on criteria
   */
  search(criteria: AlgorithmSearchCriteria): AlgorithmSearchResult[] {
    const results: AlgorithmSearchResult[] = [];

    for (const algorithm of this.algorithms.values()) {
      let relevanceScore = 0;
      const matchedCriteria: string[] = [];

      // Name matching
      if (criteria.name && algorithm.name.toLowerCase().includes(criteria.name.toLowerCase())) {
        relevanceScore += 10;
        matchedCriteria.push('name');
      }

      // Tag matching
      if (criteria.tags && algorithm.metadata.tags) {
        const matchedTags = criteria.tags.filter(tag => 
          algorithm.metadata.tags!.includes(tag)
        );
        if (matchedTags.length > 0) {
          relevanceScore += matchedTags.length * 5;
          matchedCriteria.push('tags');
        }
      }

      // Category matching
      if (criteria.categories && algorithm.metadata.categories) {
        const matchedCategories = criteria.categories.filter(category => 
          algorithm.metadata.categories!.includes(category)
        );
        if (matchedCategories.length > 0) {
          relevanceScore += matchedCategories.length * 5;
          matchedCriteria.push('categories');
        }
      }

      // Status matching
      if (criteria.status && criteria.status.includes(algorithm.metadata.status)) {
        relevanceScore += 3;
        matchedCriteria.push('status');
      }

      // Language matching
      if (criteria.language && algorithm.implementation?.language && 
          criteria.language.includes(algorithm.implementation.language)) {
        relevanceScore += 3;
        matchedCriteria.push('language');
      }

      // Complexity matching
      if (criteria.complexity && algorithm.performance?.complexity && 
          criteria.complexity.includes(algorithm.performance.complexity)) {
        relevanceScore += 3;
        matchedCriteria.push('complexity');
      }

      // Meta-category matching (if available in categories)
      if (criteria.metaCategory && algorithm.metadata.categories) {
        if (algorithm.metadata.categories.includes(criteria.metaCategory)) {
          relevanceScore += 5;
          matchedCriteria.push('metaCategory');
        }
      }

      // Task category matching (if available in categories)
      if (criteria.taskCategory && algorithm.metadata.categories) {
        if (algorithm.metadata.categories.includes(criteria.taskCategory)) {
          relevanceScore += 5;
          matchedCriteria.push('taskCategory');
        }
      }

      if (relevanceScore > 0) {
        results.push({
          algorithm,
          relevanceScore,
          matchedCriteria,
        });
      }
    }

    // Sort by relevance score (descending)
    return results.sort((a, b) => b.relevanceScore - a.relevanceScore);
  }

  /**
   * Get algorithm by ID
   */
  getById(algorithmId: AlgorithmId): AlgorithmDocumentation | null {
    return this.algorithms.get(algorithmId) || null;
  }

  /**
   * Get all algorithms
   */
  getAll(): AlgorithmDocumentation[] {
    return Array.from(this.algorithms.values());
  }

  /**
   * Update algorithm documentation
   */
  update(algorithmId: AlgorithmId, updates: Partial<AlgorithmDocumentation>): boolean {
    const existing = this.getById(algorithmId);
    if (!existing) {
      return false;
    }

    const updated: AlgorithmDocumentation = {
      ...existing,
      ...updates,
      metadata: {
        ...existing.metadata,
        ...updates.metadata,
        updatedAt: new Date().toISOString(),
      },
    };

    const validation = this.validate(updated);
    if (!validation.isValid) {
      console.error('Invalid algorithm updates:', validation.errors);
      return false;
    }

    return this.saveToFile(updated);
  }

  /**
   * Delete algorithm documentation
   */
  delete(algorithmId: AlgorithmId): boolean {
    try {
      const filePath = join(this.dataPath, `${algorithmId}.json`);
      if (existsSync(filePath)) {
        require('fs').unlinkSync(filePath);
        this.algorithms.delete(algorithmId);
        return true;
      }
      return false;
    } catch (error) {
      console.error(`Error deleting algorithm ${algorithmId}:`, error);
      return false;
    }
  }

  /**
   * Get algorithms by status
   */
  getByStatus(status: string): AlgorithmDocumentation[] {
    return this.getAll().filter(algorithm => algorithm.metadata.status === status);
  }

  /**
   * Get algorithms by language
   */
  getByLanguage(language: string): AlgorithmDocumentation[] {
    return this.getAll().filter(algorithm => 
      algorithm.implementation?.language === language
    );
  }

  /**
   * Get algorithms by complexity
   */
  getByComplexity(complexity: string): AlgorithmDocumentation[] {
    return this.getAll().filter(algorithm => 
      algorithm.performance?.complexity === complexity
    );
  }

  /**
   * Get statistics about algorithms
   */
  getStatistics(): Record<string, unknown> {
    const algorithms = this.getAll();
    
    const statusCounts: Record<string, number> = {};
    const languageCounts: Record<string, number> = {};
    const complexityCounts: Record<string, number> = {};
    const categoryCounts: Record<string, number> = {};

    for (const algorithm of algorithms) {
      // Status counts
      statusCounts[algorithm.metadata.status] = (statusCounts[algorithm.metadata.status] || 0) + 1;

      // Language counts
      if (algorithm.implementation?.language) {
        languageCounts[algorithm.implementation.language] = 
          (languageCounts[algorithm.implementation.language] || 0) + 1;
      }

      // Complexity counts
      if (algorithm.performance?.complexity) {
        complexityCounts[algorithm.performance.complexity] = 
          (complexityCounts[algorithm.performance.complexity] || 0) + 1;
      }

      // Category counts
      if (algorithm.metadata.categories) {
        for (const category of algorithm.metadata.categories) {
          categoryCounts[category] = (categoryCounts[category] || 0) + 1;
        }
      }
    }

    return {
      total: algorithms.length,
      statusCounts,
      languageCounts,
      complexityCounts,
      categoryCounts,
    };
  }

  /**
   * Export algorithms to JSON file
   */
  exportToFile(filePath: string): boolean {
    try {
      const data = {
        algorithms: this.getAll(),
        exportDate: new Date().toISOString(),
        total: this.getAll().length,
      };
      
      const dir = dirname(filePath);
      if (!existsSync(dir)) {
        mkdirSync(dir, { recursive: true });
      }
      
      writeFileSync(filePath, JSON.stringify(data, null, 2), 'utf-8');
      return true;
    } catch (error) {
      console.error('Error exporting algorithms:', error);
      return false;
    }
  }

  /**
   * Import algorithms from JSON file
   */
  importFromFile(filePath: string): boolean {
    try {
      if (!existsSync(filePath)) {
        console.error(`Import file not found: ${filePath}`);
        return false;
      }

      const data = JSON.parse(readFileSync(filePath, 'utf-8'));
      const algorithms = data.algorithms || [];

      let successCount = 0;
      let errorCount = 0;

      for (const algorithm of algorithms) {
        const validation = this.validate(algorithm);
        if (validation.isValid) {
          if (this.saveToFile(algorithm)) {
            successCount++;
          } else {
            errorCount++;
          }
        } else {
          console.error(`Invalid algorithm ${algorithm.id}:`, validation.errors);
          errorCount++;
        }
      }

      console.log(`Import completed: ${successCount} successful, ${errorCount} errors`);
      return errorCount === 0;
    } catch (error) {
      console.error('Error importing algorithms:', error);
      return false;
    }
  }
} 