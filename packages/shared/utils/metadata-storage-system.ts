import { readFileSync, writeFileSync, existsSync, mkdirSync, readdirSync } from 'fs';
import { join, dirname } from 'path';
import {
  AlgorithmDocumentation,
  AlgorithmSearchCriteria,
  AlgorithmSearchResult,
  ValidationResult,
  AlgorithmId,
} from '../types/algorithm-documentation';

/**
 * Index structure for efficient querying
 */
interface MetadataIndex {
  byId: Map<AlgorithmId, string>; // ID -> file path
  byName: Map<string, AlgorithmId[]>; // Name -> IDs
  byStatus: Map<string, AlgorithmId[]>; // Status -> IDs
  byLanguage: Map<string, AlgorithmId[]>; // Language -> IDs
  byComplexity: Map<string, AlgorithmId[]>; // Complexity -> IDs
  byTags: Map<string, AlgorithmId[]>; // Tag -> IDs
  byCategories: Map<string, AlgorithmId[]>; // Category -> IDs
  byMetaCategory: Map<string, AlgorithmId[]>; // Meta-category -> IDs
  byTaskCategory: Map<string, AlgorithmId[]>; // Task category -> IDs
  byDependencies: Map<string, AlgorithmId[]>; // Dependency -> IDs
  byPackages: Map<string, AlgorithmId[]>; // ROS package -> IDs
  byCreationDate: Map<string, AlgorithmId[]>; // Date -> IDs
  byUpdateDate: Map<string, AlgorithmId[]>; // Date -> IDs
}

/**
 * Storage configuration
 */
interface StorageConfig {
  dataPath: string;
  indexPath: string;
  backupPath: string;
  maxBackups: number;
  autoIndex: boolean;
  compression: boolean;
}

/**
 * Query result with pagination
 */
interface QueryResult<T> {
  data: T[];
  total: number;
  page: number;
  pageSize: number;
  totalPages: number;
  hasNext: boolean;
  hasPrevious: boolean;
}

/**
 * Advanced query options
 */
interface QueryOptions {
  page?: number;
  pageSize?: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
  includeDeleted?: boolean;
}

/**
 * Metadata Storage System
 * Provides database-like storage with indexing and efficient querying
 */
export class MetadataStorageSystem {
  private config: StorageConfig;
  private index: MetadataIndex;
  private cache: Map<AlgorithmId, AlgorithmDocumentation>;
  private cacheSize: number;
  private cacheHits: number;
  private cacheMisses: number;

  constructor(config: Partial<StorageConfig> = {}) {
    this.config = {
      dataPath: './data/algorithms',
      indexPath: './data/index',
      backupPath: './data/backups',
      maxBackups: 10,
      autoIndex: true,
      compression: false,
      ...config,
    };

    this.index = this.createEmptyIndex();
    this.cache = new Map();
    this.cacheSize = 100;
    this.cacheHits = 0;
    this.cacheMisses = 0;

    this.ensureDirectories();
    this.loadIndex();
  }

  /**
   * Create empty index structure
   */
  private createEmptyIndex(): MetadataIndex {
    return {
      byId: new Map(),
      byName: new Map(),
      byStatus: new Map(),
      byLanguage: new Map(),
      byComplexity: new Map(),
      byTags: new Map(),
      byCategories: new Map(),
      byMetaCategory: new Map(),
      byTaskCategory: new Map(),
      byDependencies: new Map(),
      byPackages: new Map(),
      byCreationDate: new Map(),
      byUpdateDate: new Map(),
    };
  }

  /**
   * Ensure all required directories exist
   */
  private ensureDirectories(): void {
    const directories = [
      this.config.dataPath,
      this.config.indexPath,
      this.config.backupPath,
    ];

    for (const dir of directories) {
      if (!existsSync(dir)) {
        mkdirSync(dir, { recursive: true });
      }
    }
  }

  /**
   * Load index from disk
   */
  private loadIndex(): void {
    const indexFile = join(this.config.indexPath, 'metadata-index.json');
    
    if (existsSync(indexFile)) {
      try {
        const data = readFileSync(indexFile, 'utf-8');
        const indexData = JSON.parse(data);
        
        // Reconstruct Maps from serialized data
        for (const [key, value] of Object.entries(indexData)) {
          if (this.index[key as keyof MetadataIndex]) {
            const map = this.index[key as keyof MetadataIndex] as Map<string, any>;
            for (const [mapKey, mapValue] of Object.entries(value as Record<string, any>)) {
              map.set(mapKey, mapValue);
            }
          }
        }
      } catch (error) {
        console.error('Error loading index:', error);
        this.rebuildIndex();
      }
    } else {
      this.rebuildIndex();
    }
  }

  /**
   * Save index to disk
   */
  private saveIndex(): void {
    try {
      const indexFile = join(this.config.indexPath, 'metadata-index.json');
      const serializedIndex: Record<string, Record<string, any>> = {};

      // Serialize Maps to plain objects
      for (const [key, map] of Object.entries(this.index)) {
        serializedIndex[key] = {};
        for (const [mapKey, mapValue] of map as Map<string, any>) {
          serializedIndex[key][mapKey] = mapValue;
        }
      }

      writeFileSync(indexFile, JSON.stringify(serializedIndex, null, 2), 'utf-8');
    } catch (error) {
      console.error('Error saving index:', error);
    }
  }

  /**
   * Rebuild index from all algorithm files
   */
  private rebuildIndexInternal(): void {
    console.log('Rebuilding metadata index...');
    
    this.index = this.createEmptyIndex();
    
    try {
      const files = readdirSync(this.config.dataPath);
      
      for (const file of files) {
        if (file.endsWith('.json')) {
          const algorithmId = file.replace('.json', '');
          const algorithm = this.loadAlgorithmFromFile(algorithmId);
          
          if (algorithm) {
            this.addToIndex(algorithmId, algorithm);
          }
        }
      }
      
      this.saveIndex();
      console.log('Index rebuild completed');
    } catch (error) {
      console.error('Error rebuilding index:', error);
    }
  }

  /**
   * Add algorithm to all relevant indexes
   */
  private addToIndex(algorithmId: AlgorithmId, algorithm: AlgorithmDocumentation): void {
    const filePath = join(this.config.dataPath, `${algorithmId}.json`);
    
    // Basic indexes
    this.index.byId.set(algorithmId, filePath);
    
    // Name index
    const nameKey = algorithm.name.toLowerCase();
    if (!this.index.byName.has(nameKey)) {
      this.index.byName.set(nameKey, []);
    }
    this.index.byName.get(nameKey)!.push(algorithmId);
    
    // Status index
    const status = algorithm.metadata.status;
    if (!this.index.byStatus.has(status)) {
      this.index.byStatus.set(status, []);
    }
    this.index.byStatus.get(status)!.push(algorithmId);
    
    // Language index
    if (algorithm.implementation?.language) {
      const language = algorithm.implementation.language;
      if (!this.index.byLanguage.has(language)) {
        this.index.byLanguage.set(language, []);
      }
      this.index.byLanguage.get(language)!.push(algorithmId);
    }
    
    // Complexity index
    if (algorithm.performance?.complexity) {
      const complexity = algorithm.performance.complexity;
      if (!this.index.byComplexity.has(complexity)) {
        this.index.byComplexity.set(complexity, []);
      }
      this.index.byComplexity.get(complexity)!.push(algorithmId);
    }
    
    // Tags index
    if (algorithm.metadata.tags) {
      for (const tag of algorithm.metadata.tags) {
        if (!this.index.byTags.has(tag)) {
          this.index.byTags.set(tag, []);
        }
        this.index.byTags.get(tag)!.push(algorithmId);
      }
    }
    
    // Categories index
    if (algorithm.metadata.categories) {
      for (const category of algorithm.metadata.categories) {
        if (!this.index.byCategories.has(category)) {
          this.index.byCategories.set(category, []);
        }
        this.index.byCategories.get(category)!.push(algorithmId);
      }
    }
    
    // Dependencies index
    if (algorithm.dependencies.algorithms) {
      for (const dep of algorithm.dependencies.algorithms) {
        if (!this.index.byDependencies.has(dep)) {
          this.index.byDependencies.set(dep, []);
        }
        this.index.byDependencies.get(dep)!.push(algorithmId);
      }
    }
    
    // Packages index
    for (const pkg of algorithm.dependencies.rosPackages) {
      if (!this.index.byPackages.has(pkg.name)) {
        this.index.byPackages.set(pkg.name, []);
      }
      this.index.byPackages.get(pkg.name)!.push(algorithmId);
    }
    
    // Date indexes
    const creationDate = algorithm.metadata.createdAt.split('T')[0];
    if (!this.index.byCreationDate.has(creationDate)) {
      this.index.byCreationDate.set(creationDate, []);
    }
    this.index.byCreationDate.get(creationDate)!.push(algorithmId);
    
    const updateDate = algorithm.metadata.updatedAt.split('T')[0];
    if (!this.index.byUpdateDate.has(updateDate)) {
      this.index.byUpdateDate.set(updateDate, []);
    }
    this.index.byUpdateDate.get(updateDate)!.push(algorithmId);
  }

  /**
   * Remove algorithm from all indexes
   */
  private removeFromIndex(algorithmId: AlgorithmId): void {
    // Remove from all indexes
    for (const [key, map] of Object.entries(this.index)) {
      if (key === 'byId') {
        map.delete(algorithmId);
      } else {
        for (const [indexKey, ids] of map as Map<string, AlgorithmId[]>) {
          const filteredIds = ids.filter(id => id !== algorithmId);
          if (filteredIds.length === 0) {
            map.delete(indexKey);
          } else {
            map.set(indexKey, filteredIds);
          }
        }
      }
    }
  }

  /**
   * Load algorithm from file
   */
  private loadAlgorithmFromFile(algorithmId: AlgorithmId): AlgorithmDocumentation | null {
    const filePath = join(this.config.dataPath, `${algorithmId}.json`);
    
    if (!existsSync(filePath)) {
      return null;
    }

    try {
      const data = readFileSync(filePath, 'utf-8');
      return JSON.parse(data) as AlgorithmDocumentation;
    } catch (error) {
      console.error(`Error loading algorithm ${algorithmId}:`, error);
      return null;
    }
  }

  /**
   * Save algorithm to file
   */
  private saveAlgorithmToFile(algorithm: AlgorithmDocumentation): boolean {
    try {
      const filePath = join(this.config.dataPath, `${algorithm.id}.json`);
      const data = JSON.stringify(algorithm, null, 2);
      writeFileSync(filePath, data, 'utf-8');
      return true;
    } catch (error) {
      console.error(`Error saving algorithm ${algorithm.id}:`, error);
      return false;
    }
  }

  /**
   * Get algorithm by ID with caching
   */
  getById(algorithmId: AlgorithmId): AlgorithmDocumentation | null {
    // Check cache first
    if (this.cache.has(algorithmId)) {
      this.cacheHits++;
      return this.cache.get(algorithmId)!;
    }

    this.cacheMisses++;
    
    // Load from file
    const algorithm = this.loadAlgorithmFromFile(algorithmId);
    
    if (algorithm) {
      // Add to cache
      this.cache.set(algorithmId, algorithm);
      
      // Maintain cache size
      if (this.cache.size > this.cacheSize) {
        const firstKey = this.cache.keys().next().value;
        if (firstKey) {
          this.cache.delete(firstKey);
        }
      }
    }
    
    return algorithm;
  }

  /**
   * Save algorithm with indexing
   */
  save(algorithm: AlgorithmDocumentation): boolean {
    // Validate algorithm
    const validation = this.validate(algorithm);
    if (!validation.isValid) {
      console.error('Invalid algorithm:', validation.errors);
      return false;
    }

    // Remove from index if exists
    this.removeFromIndex(algorithm.id);
    
    // Save to file
    const success = this.saveAlgorithmToFile(algorithm);
    
    if (success && this.config.autoIndex) {
      // Add to index
      this.addToIndex(algorithm.id, algorithm);
      this.saveIndex();
      
      // Update cache
      this.cache.set(algorithm.id, algorithm);
    }
    
    return success;
  }

  /**
   * Delete algorithm
   */
  delete(algorithmId: AlgorithmId): boolean {
    try {
      const filePath = join(this.config.dataPath, `${algorithmId}.json`);
      
      if (existsSync(filePath)) {
        // Remove from index
        this.removeFromIndex(algorithmId);
        this.saveIndex();
        
        // Remove from cache
        this.cache.delete(algorithmId);
        
        // Delete file
        const { unlinkSync } = require('fs');
        unlinkSync(filePath);
        
        return true;
      }
      
      return false;
    } catch (error) {
      console.error(`Error deleting algorithm ${algorithmId}:`, error);
      return false;
    }
  }

  /**
   * Update algorithm
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

    return this.save(updated);
  }

  /**
   * Advanced search with multiple criteria
   */
  search(criteria: AlgorithmSearchCriteria, options: QueryOptions = {}): QueryResult<AlgorithmSearchResult> {
    const {
      page = 1,
      pageSize = 20,
      sortBy = 'relevance',
      sortOrder = 'desc',
    } = options;

    const results: AlgorithmSearchResult[] = [];
    const matchedIds = new Set<AlgorithmId>();

    // Build search results
    for (const algorithm of this.getAll()) {
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

      if (relevanceScore > 0) {
        results.push({
          algorithm,
          relevanceScore,
          matchedCriteria,
        });
        matchedIds.add(algorithm.id);
      }
    }

    // Sort results
    if (sortBy === 'relevance') {
      results.sort((a, b) => 
        sortOrder === 'desc' ? b.relevanceScore - a.relevanceScore : a.relevanceScore - b.relevanceScore
      );
    } else if (sortBy === 'name') {
      results.sort((a, b) => 
        sortOrder === 'desc' ? b.algorithm.name.localeCompare(a.algorithm.name) : a.algorithm.name.localeCompare(b.algorithm.name)
      );
    } else if (sortBy === 'createdAt') {
      results.sort((a, b) => 
        sortOrder === 'desc' ? 
          new Date(b.algorithm.metadata.createdAt).getTime() - new Date(a.algorithm.metadata.createdAt).getTime() :
          new Date(a.algorithm.metadata.createdAt).getTime() - new Date(b.algorithm.metadata.createdAt).getTime()
      );
    }

    // Pagination
    const total = results.length;
    const totalPages = Math.ceil(total / pageSize);
    const startIndex = (page - 1) * pageSize;
    const endIndex = startIndex + pageSize;
    const paginatedResults = results.slice(startIndex, endIndex);

    return {
      data: paginatedResults,
      total,
      page,
      pageSize,
      totalPages,
      hasNext: page < totalPages,
      hasPrevious: page > 1,
    };
  }

  /**
   * Get all algorithms
   */
  getAll(): AlgorithmDocumentation[] {
    const algorithms: AlgorithmDocumentation[] = [];
    
    for (const algorithmId of this.index.byId.keys()) {
      const algorithm = this.getById(algorithmId);
      if (algorithm) {
        algorithms.push(algorithm);
      }
    }
    
    return algorithms;
  }

  /**
   * Get algorithms by status using index
   */
  getByStatus(status: string): AlgorithmDocumentation[] {
    const ids = this.index.byStatus.get(status) || [];
    return ids.map(id => this.getById(id)).filter(Boolean) as AlgorithmDocumentation[];
  }

  /**
   * Get algorithms by language using index
   */
  getByLanguage(language: string): AlgorithmDocumentation[] {
    const ids = this.index.byLanguage.get(language) || [];
    return ids.map(id => this.getById(id)).filter(Boolean) as AlgorithmDocumentation[];
  }

  /**
   * Get algorithms by complexity using index
   */
  getByComplexity(complexity: string): AlgorithmDocumentation[] {
    const ids = this.index.byComplexity.get(complexity) || [];
    return ids.map(id => this.getById(id)).filter(Boolean) as AlgorithmDocumentation[];
  }

  /**
   * Get algorithms by tag using index
   */
  getByTag(tag: string): AlgorithmDocumentation[] {
    const ids = this.index.byTags.get(tag) || [];
    return ids.map(id => this.getById(id)).filter(Boolean) as AlgorithmDocumentation[];
  }

  /**
   * Get algorithms by category using index
   */
  getByCategory(category: string): AlgorithmDocumentation[] {
    const ids = this.index.byCategories.get(category) || [];
    return ids.map(id => this.getById(id)).filter(Boolean) as AlgorithmDocumentation[];
  }

  /**
   * Get algorithms by ROS package using index
   */
  getByPackage(packageName: string): AlgorithmDocumentation[] {
    const ids = this.index.byPackages.get(packageName) || [];
    return ids.map(id => this.getById(id)).filter(Boolean) as AlgorithmDocumentation[];
  }

  /**
   * Get algorithms created on specific date
   */
  getByCreationDate(date: string): AlgorithmDocumentation[] {
    const ids = this.index.byCreationDate.get(date) || [];
    return ids.map(id => this.getById(id)).filter(Boolean) as AlgorithmDocumentation[];
  }

  /**
   * Get algorithms updated on specific date
   */
  getByUpdateDate(date: string): AlgorithmDocumentation[] {
    const ids = this.index.byUpdateDate.get(date) || [];
    return ids.map(id => this.getById(id)).filter(Boolean) as AlgorithmDocumentation[];
  }

  /**
   * Validate algorithm
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

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
    };
  }

  /**
   * Get storage statistics
   */
  getStatistics(): Record<string, unknown> {
    const algorithms = this.getAll();
    
    const statusCounts: Record<string, number> = {};
    const languageCounts: Record<string, number> = {};
    const complexityCounts: Record<string, number> = {};
    const categoryCounts: Record<string, number> = {};
    const tagCounts: Record<string, number> = {};
    const packageCounts: Record<string, number> = {};

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

      // Tag counts
      if (algorithm.metadata.tags) {
        for (const tag of algorithm.metadata.tags) {
          tagCounts[tag] = (tagCounts[tag] || 0) + 1;
        }
      }

      // Package counts
      for (const pkg of algorithm.dependencies.rosPackages) {
        packageCounts[pkg.name] = (packageCounts[pkg.name] || 0) + 1;
      }
    }

    return {
      total: algorithms.length,
      statusCounts,
      languageCounts,
      complexityCounts,
      categoryCounts,
      tagCounts,
      packageCounts,
      cacheStats: {
        size: this.cache.size,
        hits: this.cacheHits,
        misses: this.cacheMisses,
        hitRate: this.cacheHits / (this.cacheHits + this.cacheMisses),
      },
      indexStats: {
        byId: this.index.byId.size,
        byName: this.index.byName.size,
        byStatus: this.index.byStatus.size,
        byLanguage: this.index.byLanguage.size,
        byComplexity: this.index.byComplexity.size,
        byTags: this.index.byTags.size,
        byCategories: this.index.byCategories.size,
        byPackages: this.index.byPackages.size,
      },
    };
  }

  /**
   * Create backup of all data
   */
  createBackup(): boolean {
    try {
      const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
      const backupDir = join(this.config.backupPath, `backup-${timestamp}`);
      
      if (!existsSync(backupDir)) {
        mkdirSync(backupDir, { recursive: true });
      }

      // Copy all algorithm files
      const { copyFileSync } = require('fs');
      const files = readdirSync(this.config.dataPath);
      
      for (const file of files) {
        if (file.endsWith('.json')) {
          const sourcePath = join(this.config.dataPath, file);
          const destPath = join(backupDir, file);
          copyFileSync(sourcePath, destPath);
        }
      }

      // Copy index
      const indexSource = join(this.config.indexPath, 'metadata-index.json');
      const indexDest = join(backupDir, 'metadata-index.json');
      if (existsSync(indexSource)) {
        copyFileSync(indexSource, indexDest);
      }

      // Clean up old backups
      this.cleanupOldBackups();

      return true;
    } catch (error) {
      console.error('Error creating backup:', error);
      return false;
    }
  }

  /**
   * Clean up old backups
   */
  private cleanupOldBackups(): void {
    try {
      const backups = readdirSync(this.config.backupPath)
        .filter(dir => dir.startsWith('backup-'))
        .sort()
        .reverse();

      if (backups.length > this.config.maxBackups) {
        const toDelete = backups.slice(this.config.maxBackups);
        
        for (const backup of toDelete) {
          const backupPath = join(this.config.backupPath, backup);
          const { rmdirSync } = require('fs');
          rmdirSync(backupPath, { recursive: true });
        }
      }
    } catch (error) {
      console.error('Error cleaning up backups:', error);
    }
  }

  /**
   * Rebuild index from scratch
   */
  rebuildIndex(): void {
    this.rebuildIndexInternal();
  }

  /**
   * Clear cache
   */
  clearCache(): void {
    this.cache.clear();
    this.cacheHits = 0;
    this.cacheMisses = 0;
  }

  /**
   * Export data to JSON file
   */
  exportToFile(filePath: string): boolean {
    try {
      const data = {
        algorithms: this.getAll(),
        exportDate: new Date().toISOString(),
        total: this.getAll().length,
        statistics: this.getStatistics(),
      };
      
      const dir = dirname(filePath);
      if (!existsSync(dir)) {
        mkdirSync(dir, { recursive: true });
      }
      
      writeFileSync(filePath, JSON.stringify(data, null, 2), 'utf-8');
      return true;
    } catch (error) {
      console.error('Error exporting data:', error);
      return false;
    }
  }

  /**
   * Import data from JSON file
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
          if (this.save(algorithm)) {
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
      console.error('Error importing data:', error);
      return false;
    }
  }
} 