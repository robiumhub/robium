import { readFileSync, writeFileSync, existsSync, mkdirSync, readdirSync } from 'fs';
import { join, dirname } from 'path';
import { MetadataStorageSystem } from './metadata-storage-system';
import { AlgorithmDocumentation, AlgorithmId } from '../types/algorithm-documentation';
import {
  DiscoveryQuery,
  DiscoveryFilters,
  DiscoverySorting,
  DiscoveryPagination,
  DiscoveryResult,
  DiscoveryResponse,
  DiscoveryStatistics,
  DiscoverySuggestion,
  RecommendationCriteria,
  RecommendationResult,
  BrowsingInterface,
  CategoryNode,
  FacetedSearch,
  Facet,
  FacetOption,
  SearchEngineConfig,
  ComparisonCriteria,
  ComparisonResult,
  ComparisonSummary,
  DiscoverySession,
  PopularityMetrics,
  SearchField,
  SortField,
} from '../types/algorithm-discovery';

/**
 * Algorithm Discovery System
 * Provides comprehensive search, discovery, and recommendation capabilities for ROS algorithms
 */
export class AlgorithmDiscoverySystem {
  private config: SearchEngineConfig;
  private metadataStorage: MetadataStorageSystem;
  private searchIndex: Map<string, Set<AlgorithmId>>;
  private popularityMetrics: Map<AlgorithmId, PopularityMetrics>;
  private sessions: Map<string, DiscoverySession>;
  private cache: Map<string, DiscoveryResponse>;
  private cacheHits: number;
  private cacheMisses: number;

  constructor(
    metadataStorage: MetadataStorageSystem,
    config: Partial<SearchEngineConfig> = {}
  ) {
    this.metadataStorage = metadataStorage;
    this.config = {
      indexPath: './data/search-index',
      cacheEnabled: true,
      cacheSize: 1000,
      maxResults: 1000,
      relevanceThreshold: 0.1,
      fuzzySearch: true,
      synonymExpansion: true,
      stemming: true,
      stopWords: ['the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for', 'of', 'with', 'by'],
      boostFactors: {
        name: 3.0,
        description: 2.0,
        tags: 1.5,
        categories: 1.2,
        content: 1.0,
      },
      ...config,
    };

    this.searchIndex = new Map();
    this.popularityMetrics = new Map();
    this.sessions = new Map();
    this.cache = new Map();
    this.cacheHits = 0;
    this.cacheMisses = 0;

    this.ensureDirectories();
    this.buildSearchIndex();
    this.loadPopularityMetrics();
  }

  /**
   * Ensure required directories exist
   */
  private ensureDirectories(): void {
    const dirs = [this.config.indexPath, dirname(this.config.indexPath)];
    for (const dir of dirs) {
      if (!existsSync(dir)) {
        mkdirSync(dir, { recursive: true });
      }
    }
  }

  /**
   * Build search index from algorithm documentation
   */
  private buildSearchIndex(): void {
    try {
      const algorithms = this.metadataStorage.getAll();
      
      for (const algorithm of algorithms) {
        this.indexAlgorithm(algorithm);
      }

      this.saveSearchIndex();
    } catch (error) {
      console.error('Failed to build search index:', error);
    }
  }

  /**
   * Index a single algorithm for search
   */
  private indexAlgorithm(algorithm: AlgorithmDocumentation): void {
    const terms = this.extractSearchTerms(algorithm);
    
    for (const term of terms) {
      if (!this.searchIndex.has(term)) {
        this.searchIndex.set(term, new Set());
      }
      this.searchIndex.get(term)!.add(algorithm.id);
    }
  }

  /**
   * Extract search terms from algorithm documentation
   */
  private extractSearchTerms(algorithm: AlgorithmDocumentation): string[] {
    const terms = new Set<string>();

    // Extract from name
    terms.add(...this.tokenize(algorithm.name));

    // Extract from description
    if (algorithm.taskDefinition.description) {
      terms.add(...this.tokenize(algorithm.taskDefinition.description));
    }

    // Extract from tags
    if (algorithm.metadata.tags) {
      terms.add(...algorithm.metadata.tags);
    }

    // Extract from categories
    if (algorithm.metadata.categories) {
      terms.add(...algorithm.metadata.categories);
    }

    // Extract from dependencies
    for (const pkg of algorithm.dependencies.rosPackages) {
      terms.add(pkg.name);
      if (pkg.purpose) {
        terms.add(...this.tokenize(pkg.purpose));
      }
    }

    // Apply stemming and filtering
    return Array.from(terms)
      .map(term => this.stem(term))
      .filter(term => !this.config.stopWords.includes(term.toLowerCase()))
      .filter(term => term.length > 2);
  }

  /**
   * Tokenize text into search terms
   */
  private tokenize(text: string): string[] {
    return text
      .toLowerCase()
      .replace(/[^\w\s]/g, ' ')
      .split(/\s+/)
      .filter(term => term.length > 0);
  }

  /**
   * Simple stemming implementation
   */
  private stem(term: string): string {
    // Simple Porter stemming implementation
    const term_lower = term.toLowerCase();
    
    // Remove common suffixes
    if (term_lower.endsWith('ing')) {
      return term.slice(0, -3);
    }
    if (term_lower.endsWith('ed')) {
      return term.slice(0, -2);
    }
    if (term_lower.endsWith('s')) {
      return term.slice(0, -1);
    }
    
    return term;
  }

  /**
   * Save search index to disk
   */
  private saveSearchIndex(): void {
    try {
      const indexData = Object.fromEntries(
        Array.from(this.searchIndex.entries()).map(([term, algorithms]) => [
          term,
          Array.from(algorithms),
        ])
      );

      const indexPath = join(this.config.indexPath, 'search-index.json');
      writeFileSync(indexPath, JSON.stringify(indexData, null, 2), 'utf-8');
    } catch (error) {
      console.error('Failed to save search index:', error);
    }
  }

  /**
   * Load popularity metrics
   */
  private loadPopularityMetrics(): void {
    try {
      const metricsPath = join(this.config.indexPath, 'popularity-metrics.json');
      if (existsSync(metricsPath)) {
        const data = JSON.parse(readFileSync(metricsPath, 'utf-8'));
        for (const [id, metrics] of Object.entries(data)) {
          this.popularityMetrics.set(id, metrics as PopularityMetrics);
        }
      }
    } catch (error) {
      console.error('Failed to load popularity metrics:', error);
    }
  }

  /**
   * Search for algorithms based on query
   */
  search(query: DiscoveryQuery): DiscoveryResponse {
    const startTime = Date.now();
    const cacheKey = this.generateCacheKey(query);

    // Check cache first
    if (this.config.cacheEnabled && this.cache.has(cacheKey)) {
      this.cacheHits++;
      return this.cache.get(cacheKey)!;
    }

    this.cacheMisses++;

    try {
      // Execute search
      const results = this.executeSearch(query);
      
      // Apply filters
      const filteredResults = this.applyFilters(results, query.filters);
      
      // Apply sorting
      const sortedResults = this.applySorting(filteredResults, query.sorting);
      
      // Apply pagination
      const paginatedResults = this.applyPagination(sortedResults, query.pagination);
      
      // Calculate statistics
      const statistics: DiscoveryStatistics = {
        searchTime: Date.now() - startTime,
        filterTime: 0, // TODO: Track filter time
        sortTime: 0, // TODO: Track sort time
        totalTime: Date.now() - startTime,
        cacheHits: this.cacheHits,
        cacheMisses: this.cacheMisses,
        resultCount: paginatedResults.length,
        filterCounts: this.calculateFilterCounts(filteredResults),
      };

      // Generate suggestions
      const suggestions = this.generateSuggestions(query, filteredResults);

      const response: DiscoveryResponse = {
        results: paginatedResults,
        totalResults: filteredResults.length,
        page: query.pagination.page,
        pageSize: query.pagination.pageSize,
        totalPages: Math.ceil(filteredResults.length / query.pagination.pageSize),
        hasNext: query.pagination.page < Math.ceil(filteredResults.length / query.pagination.pageSize),
        hasPrevious: query.pagination.page > 1,
        query,
        statistics,
        suggestions,
      };

      // Cache the result
      if (this.config.cacheEnabled && this.cache.size < this.config.cacheSize) {
        this.cache.set(cacheKey, response);
      }

      return response;
    } catch (error) {
      return {
        results: [],
        totalResults: 0,
        page: query.pagination.page,
        pageSize: query.pagination.pageSize,
        totalPages: 0,
        hasNext: false,
        hasPrevious: false,
        query,
        statistics: {
          searchTime: Date.now() - startTime,
          filterTime: 0,
          sortTime: 0,
          totalTime: Date.now() - startTime,
          cacheHits: this.cacheHits,
          cacheMisses: this.cacheMisses,
          resultCount: 0,
          filterCounts: {},
        },
        suggestions: [],
      };
    }
  }

  /**
   * Execute the actual search
   */
  private executeSearch(query: DiscoveryQuery): DiscoveryResult[] {
    const results: DiscoveryResult[] = [];
    const algorithms = this.metadataStorage.getAll();

    if (query.text) {
      // Text-based search
      const searchTerms = this.tokenize(query.text);
      const matchingAlgorithms = new Set<AlgorithmId>();

      for (const term of searchTerms) {
        if (this.searchIndex.has(term)) {
          for (const algorithmId of this.searchIndex.get(term)!) {
            matchingAlgorithms.add(algorithmId);
          }
        }
      }

      // Calculate relevance scores
      for (const algorithmId of matchingAlgorithms) {
        const algorithm = this.metadataStorage.getById(algorithmId);
        if (algorithm) {
          const relevanceScore = this.calculateRelevanceScore(algorithm, query.text);
          if (relevanceScore >= this.config.relevanceThreshold) {
            results.push({
              algorithm,
              relevanceScore,
              matchedCriteria: this.getMatchedCriteria(algorithm, query.text),
            });
          }
        }
      }
    } else {
      // No text query, return all algorithms with default relevance
      for (const algorithm of algorithms) {
        results.push({
          algorithm,
          relevanceScore: 1.0,
          matchedCriteria: [],
        });
      }
    }

    return results;
  }

  /**
   * Calculate relevance score for an algorithm
   */
  private calculateRelevanceScore(algorithm: AlgorithmDocumentation, query: string): number {
    let score = 0;
    const queryTerms = this.tokenize(query);
    const algorithmTerms = this.extractSearchTerms(algorithm);

    for (const queryTerm of queryTerms) {
      for (const algorithmTerm of algorithmTerms) {
        if (queryTerm === algorithmTerm) {
          score += 1.0;
        } else if (this.config.fuzzySearch && this.calculateSimilarity(queryTerm, algorithmTerm) > 0.8) {
          score += 0.8;
        }
      }
    }

    // Apply boost factors
    if (algorithmTerms.includes(query.toLowerCase())) {
      score *= this.config.boostFactors.name;
    }

    return Math.min(score, 1.0);
  }

  /**
   * Calculate similarity between two terms
   */
  private calculateSimilarity(term1: string, term2: string): number {
    // Simple Levenshtein distance-based similarity
    const distance = this.levenshteinDistance(term1, term2);
    const maxLength = Math.max(term1.length, term2.length);
    return 1 - (distance / maxLength);
  }

  /**
   * Calculate Levenshtein distance
   */
  private levenshteinDistance(str1: string, str2: string): number {
    const matrix = Array(str2.length + 1).fill(null).map(() => Array(str1.length + 1).fill(null));

    for (let i = 0; i <= str1.length; i++) matrix[0][i] = i;
    for (let j = 0; j <= str2.length; j++) matrix[j][0] = j;

    for (let j = 1; j <= str2.length; j++) {
      for (let i = 1; i <= str1.length; i++) {
        const indicator = str1[i - 1] === str2[j - 1] ? 0 : 1;
        matrix[j][i] = Math.min(
          matrix[j][i - 1] + 1,
          matrix[j - 1][i] + 1,
          matrix[j - 1][i - 1] + indicator
        );
      }
    }

    return matrix[str2.length][str1.length];
  }

  /**
   * Get matched criteria for an algorithm
   */
  private getMatchedCriteria(algorithm: AlgorithmDocumentation, query: string): string[] {
    const criteria: string[] = [];
    const queryTerms = this.tokenize(query);

    if (queryTerms.some(term => algorithm.name.toLowerCase().includes(term))) {
      criteria.push('name');
    }

    if (queryTerms.some(term => algorithm.taskDefinition.description.toLowerCase().includes(term))) {
      criteria.push('description');
    }

    if (algorithm.metadata.tags && queryTerms.some(term => algorithm.metadata.tags!.includes(term))) {
      criteria.push('tags');
    }

    return criteria;
  }

  /**
   * Apply filters to search results
   */
  private applyFilters(results: DiscoveryResult[], filters: DiscoveryFilters): DiscoveryResult[] {
    return results.filter(result => {
      const algorithm = result.algorithm;

      // Meta-category filter
      if (filters.metaCategories && filters.metaCategories.length > 0) {
        if (!algorithm.metadata.categories?.some(cat => filters.metaCategories!.includes(cat))) {
          return false;
        }
      }

      // Task category filter
      if (filters.taskCategories && filters.taskCategories.length > 0) {
        if (!algorithm.metadata.categories?.some(cat => filters.taskCategories!.includes(cat))) {
          return false;
        }
      }

      // Complexity filter
      if (filters.complexity && filters.complexity.length > 0) {
        if (!algorithm.performance?.complexity || !filters.complexity.includes(algorithm.performance.complexity)) {
          return false;
        }
      }

      // Status filter
      if (filters.status && filters.status.length > 0) {
        if (!filters.status.includes(algorithm.metadata.status)) {
          return false;
        }
      }

      // Language filter
      if (filters.language && filters.language.length > 0) {
        if (!algorithm.implementation?.language || !filters.language.includes(algorithm.implementation.language)) {
          return false;
        }
      }

      // Tags filter
      if (filters.tags && filters.tags.length > 0) {
        if (!algorithm.metadata.tags?.some(tag => filters.tags!.includes(tag))) {
          return false;
        }
      }

      // Date range filter
      if (filters.dateRange) {
        const createdAt = new Date(algorithm.metadata.createdAt);
        if (filters.dateRange.createdAfter && createdAt < new Date(filters.dateRange.createdAfter)) {
          return false;
        }
        if (filters.dateRange.createdBefore && createdAt > new Date(filters.dateRange.createdBefore)) {
          return false;
        }
      }

      return true;
    });
  }

  /**
   * Apply sorting to search results
   */
  private applySorting(results: DiscoveryResult[], sorting: DiscoverySorting): DiscoveryResult[] {
    return results.sort((a, b) => {
      let comparison = 0;

      switch (sorting.field) {
        case 'relevance':
          comparison = b.relevanceScore - a.relevanceScore;
          break;
        case 'name':
          comparison = a.algorithm.name.localeCompare(b.algorithm.name);
          break;
        case 'complexity':
          comparison = this.compareComplexity(a.algorithm.performance?.complexity, b.algorithm.performance?.complexity);
          break;
        case 'status':
          comparison = a.algorithm.metadata.status.localeCompare(b.algorithm.metadata.status);
          break;
        case 'createdAt':
          comparison = new Date(a.algorithm.metadata.createdAt).getTime() - new Date(b.algorithm.metadata.createdAt).getTime();
          break;
        case 'updatedAt':
          comparison = new Date(a.algorithm.metadata.updatedAt).getTime() - new Date(b.algorithm.metadata.updatedAt).getTime();
          break;
        case 'popularity':
          comparison = this.getPopularityScore(b.algorithm.id) - this.getPopularityScore(a.algorithm.id);
          break;
      }

      if (sorting.order === 'desc') {
        comparison = -comparison;
      }

      // Apply secondary sorting if needed
      if (comparison === 0 && sorting.secondaryField) {
        // TODO: Implement secondary sorting
      }

      return comparison;
    });
  }

  /**
   * Compare complexity levels
   */
  private compareComplexity(a?: string, b?: string): number {
    const complexityOrder = ['basic', 'intermediate', 'advanced', 'expert'];
    const aIndex = a ? complexityOrder.indexOf(a) : -1;
    const bIndex = b ? complexityOrder.indexOf(b) : -1;
    return aIndex - bIndex;
  }

  /**
   * Get popularity score for an algorithm
   */
  private getPopularityScore(algorithmId: AlgorithmId): number {
    const metrics = this.popularityMetrics.get(algorithmId);
    if (!metrics) return 0;
    return metrics.viewCount + metrics.downloadCount * 2 + metrics.rating * 10;
  }

  /**
   * Apply pagination to search results
   */
  private applyPagination(results: DiscoveryResult[], pagination: DiscoveryPagination): DiscoveryResult[] {
    const start = (pagination.page - 1) * pagination.pageSize;
    const end = start + pagination.pageSize;
    return results.slice(start, end);
  }

  /**
   * Calculate filter counts
   */
  private calculateFilterCounts(results: DiscoveryResult[]): Record<string, number> {
    const counts: Record<string, number> = {};
    
    for (const result of results) {
      const algorithm = result.algorithm;
      
      // Count by status
      counts[`status_${algorithm.metadata.status}`] = (counts[`status_${algorithm.metadata.status}`] || 0) + 1;
      
      // Count by complexity
      if (algorithm.performance?.complexity) {
        counts[`complexity_${algorithm.performance.complexity}`] = (counts[`complexity_${algorithm.performance.complexity}`] || 0) + 1;
      }
      
      // Count by language
      if (algorithm.implementation?.language) {
        counts[`language_${algorithm.implementation.language}`] = (counts[`language_${algorithm.implementation.language}`] || 0) + 1;
      }
    }
    
    return counts;
  }

  /**
   * Generate search suggestions
   */
  private generateSuggestions(query: DiscoveryQuery, results: DiscoveryResult[]): DiscoverySuggestion[] {
    const suggestions: DiscoverySuggestion[] = [];

    if (query.text && results.length === 0) {
      // No results found, suggest alternatives
      suggestions.push({
        type: 'spelling',
        text: this.suggestSpelling(query.text),
        confidence: 0.8,
        reason: 'No exact matches found, try this spelling',
      });
    }

    // Suggest related categories
    if (query.filters.metaCategories && query.filters.metaCategories.length > 0) {
      suggestions.push({
        type: 'category',
        text: 'Try expanding your category search',
        confidence: 0.6,
        reason: 'Narrow category filter may be limiting results',
      });
    }

    return suggestions;
  }

  /**
   * Suggest spelling corrections
   */
  private suggestSpelling(text: string): string {
    // Simple spelling suggestion - in a real implementation, this would use a dictionary
    return text; // Placeholder
  }

  /**
   * Generate cache key for query
   */
  private generateCacheKey(query: DiscoveryQuery): string {
    return JSON.stringify(query);
  }

  /**
   * Get algorithm recommendations
   */
  getRecommendations(criteria: RecommendationCriteria): RecommendationResult[] {
    const baseAlgorithm = this.metadataStorage.getById(criteria.algorithmId);
    if (!baseAlgorithm) {
      return [];
    }

    const allAlgorithms = this.metadataStorage.getAll();
    const recommendations: RecommendationResult[] = [];

    for (const algorithm of allAlgorithms) {
      if (algorithm.id === criteria.algorithmId) continue;

      const score = this.calculateSimilarityScore(baseAlgorithm, algorithm, criteria);
      if (score >= (criteria.similarity?.threshold || 0.5)) {
        recommendations.push({
          algorithm,
          score,
          reason: this.generateRecommendationReason(baseAlgorithm, algorithm, criteria),
          similarityFactors: this.getSimilarityFactors(baseAlgorithm, algorithm),
        });
      }
    }

    return recommendations
      .sort((a, b) => b.score - a.score)
      .slice(0, criteria.similarity?.maxResults || 10);
  }

  /**
   * Calculate similarity score between algorithms
   */
  private calculateSimilarityScore(
    algorithm1: AlgorithmDocumentation,
    algorithm2: AlgorithmDocumentation,
    criteria: RecommendationCriteria
  ): number {
    let score = 0;
    const basedOn = criteria.similarity?.basedOn || 'all';

    if (basedOn === 'functionality' || basedOn === 'all') {
      score += this.calculateFunctionalitySimilarity(algorithm1, algorithm2) * 0.4;
    }

    if (basedOn === 'complexity' || basedOn === 'all') {
      score += this.calculateComplexitySimilarity(algorithm1, algorithm2) * 0.2;
    }

    if (basedOn === 'dependencies' || basedOn === 'all') {
      score += this.calculateDependencySimilarity(algorithm1, algorithm2) * 0.2;
    }

    if (basedOn === 'performance' || basedOn === 'all') {
      score += this.calculatePerformanceSimilarity(algorithm1, algorithm2) * 0.2;
    }

    return Math.min(score, 1.0);
  }

  /**
   * Calculate functionality similarity
   */
  private calculateFunctionalitySimilarity(alg1: AlgorithmDocumentation, alg2: AlgorithmDocumentation): number {
    let similarity = 0;
    
    // Compare task definitions
    const desc1 = alg1.taskDefinition.description.toLowerCase();
    const desc2 = alg2.taskDefinition.description.toLowerCase();
    const commonWords = this.getCommonWords(desc1, desc2);
    similarity += commonWords.length / Math.max(desc1.split(' ').length, desc2.split(' ').length) * 0.5;
    
    // Compare tags
    if (alg1.metadata.tags && alg2.metadata.tags) {
      const commonTags = alg1.metadata.tags.filter(tag => alg2.metadata.tags!.includes(tag));
      similarity += commonTags.length / Math.max(alg1.metadata.tags.length, alg2.metadata.tags.length) * 0.5;
    }
    
    return similarity;
  }

  /**
   * Calculate complexity similarity
   */
  private calculateComplexitySimilarity(alg1: AlgorithmDocumentation, alg2: AlgorithmDocumentation): number {
    if (!alg1.performance?.complexity || !alg2.performance?.complexity) return 0.5;
    
    const complexityOrder = ['basic', 'intermediate', 'advanced', 'expert'];
    const index1 = complexityOrder.indexOf(alg1.performance.complexity);
    const index2 = complexityOrder.indexOf(alg2.performance.complexity);
    
    const distance = Math.abs(index1 - index2);
    return 1 - (distance / (complexityOrder.length - 1));
  }

  /**
   * Calculate dependency similarity
   */
  private calculateDependencySimilarity(alg1: AlgorithmDocumentation, alg2: AlgorithmDocumentation): number {
    const deps1 = alg1.dependencies.rosPackages.map(p => p.name);
    const deps2 = alg2.dependencies.rosPackages.map(p => p.name);
    
    const commonDeps = deps1.filter(dep => deps2.includes(dep));
    return commonDeps.length / Math.max(deps1.length, deps2.length);
  }

  /**
   * Calculate performance similarity
   */
  private calculatePerformanceSimilarity(alg1: AlgorithmDocumentation, alg2: AlgorithmDocumentation): number {
    if (!alg1.performance?.complexity || !alg2.performance?.complexity) return 0.5;
    return alg1.performance.complexity === alg2.performance.complexity ? 1.0 : 0.0;
  }

  /**
   * Get common words between two texts
   */
  private getCommonWords(text1: string, text2: string): string[] {
    const words1 = new Set(this.tokenize(text1));
    const words2 = new Set(this.tokenize(text2));
    return Array.from(words1).filter(word => words2.has(word));
  }

  /**
   * Generate recommendation reason
   */
  private generateRecommendationReason(
    base: AlgorithmDocumentation,
    recommended: AlgorithmDocumentation,
    criteria: RecommendationCriteria
  ): string {
    const factors = this.getSimilarityFactors(base, recommended);
    
    if (factors.includes('same_category')) {
      return `Similar algorithm in the same category`;
    }
    
    if (factors.includes('similar_complexity')) {
      return `Similar complexity level`;
    }
    
    if (factors.includes('shared_dependencies')) {
      return `Shares dependencies with your selected algorithm`;
    }
    
    return `Similar functionality and characteristics`;
  }

  /**
   * Get similarity factors between algorithms
   */
  private getSimilarityFactors(alg1: AlgorithmDocumentation, alg2: AlgorithmDocumentation): string[] {
    const factors: string[] = [];
    
    // Check categories
    if (alg1.metadata.categories && alg2.metadata.categories) {
      const commonCategories = alg1.metadata.categories.filter(cat => alg2.metadata.categories!.includes(cat));
      if (commonCategories.length > 0) {
        factors.push('same_category');
      }
    }
    
    // Check complexity
    if (alg1.performance?.complexity === alg2.performance?.complexity) {
      factors.push('similar_complexity');
    }
    
    // Check dependencies
    const deps1 = alg1.dependencies.rosPackages.map(p => p.name);
    const deps2 = alg2.dependencies.rosPackages.map(p => p.name);
    const commonDeps = deps1.filter(dep => deps2.includes(dep));
    if (commonDeps.length > 0) {
      factors.push('shared_dependencies');
    }
    
    return factors;
  }

  /**
   * Compare algorithms
   */
  compareAlgorithms(criteria: ComparisonCriteria): ComparisonResult {
    const algorithms = criteria.algorithms
      .map(id => this.metadataStorage.getById(id))
      .filter((alg): alg is AlgorithmDocumentation => alg !== null);

    if (algorithms.length < 2) {
      throw new Error('At least 2 algorithms are required for comparison');
    }

    const comparison: Record<string, Record<string, unknown>> = {};
    const summary: ComparisonSummary = {
      bestByField: {},
      tradeoffs: [],
      compatibility: {},
    };

    // Compare each field
    for (const field of criteria.comparisonFields) {
      comparison[field] = {};
      
      for (const algorithm of algorithms) {
        comparison[field][algorithm.id] = this.extractFieldValue(algorithm, field);
      }
      
      // Find best in this field
      const bestAlgorithm = this.findBestInField(algorithms, field);
      if (bestAlgorithm) {
        summary.bestByField[field] = bestAlgorithm.id;
      }
    }

    // Generate recommendations
    const recommendations = this.generateComparisonRecommendations(algorithms, comparison);

    return {
      algorithms,
      comparison,
      summary,
      recommendations,
    };
  }

  /**
   * Extract field value for comparison
   */
  private extractFieldValue(algorithm: AlgorithmDocumentation, field: string): unknown {
    switch (field) {
      case 'performance':
        return algorithm.performance;
      case 'complexity':
        return algorithm.performance?.complexity;
      case 'dependencies':
        return algorithm.dependencies;
      case 'compatibility':
        return algorithm.implementation;
      case 'documentation':
        return algorithm.documentation;
      default:
        return null;
    }
  }

  /**
   * Find best algorithm in a specific field
   */
  private findBestInField(algorithms: AlgorithmDocumentation[], field: string): AlgorithmDocumentation | null {
    // This is a simplified implementation - in practice, you'd have more sophisticated logic
    return algorithms[0] || null;
  }

  /**
   * Generate comparison recommendations
   */
  private generateComparisonRecommendations(
    algorithms: AlgorithmDocumentation[],
    comparison: Record<string, Record<string, unknown>>
  ): string[] {
    const recommendations: string[] = [];
    
    // Add general recommendations based on comparison
    recommendations.push(`Compared ${algorithms.length} algorithms across ${Object.keys(comparison).length} criteria`);
    
    return recommendations;
  }

  /**
   * Get browsing interface
   */
  getBrowsingInterface(): BrowsingInterface {
    const categories = this.buildCategoryTree();
    
    return {
      categories,
      filters: {
        searchText: '',
        selectedCategories: [],
        complexityRange: ['basic', 'expert'],
        statusFilter: [],
        languageFilter: [],
        tagFilter: [],
        dateRange: ['', ''],
      },
      view: 'grid',
      grouping: 'category',
      sorting: {
        field: 'name',
        order: 'asc',
      },
    };
  }

  /**
   * Build category tree for browsing
   */
  private buildCategoryTree(): CategoryNode[] {
    const algorithms = this.metadataStorage.getAll();
    const categoryMap = new Map<string, CategoryNode>();

    // Build category nodes
    for (const algorithm of algorithms) {
      if (algorithm.metadata.categories) {
        for (const category of algorithm.metadata.categories) {
          if (!categoryMap.has(category)) {
            categoryMap.set(category, {
              id: category,
              name: category,
              type: 'task-category',
              algorithmCount: 0,
              path: [category],
            });
          }
          categoryMap.get(category)!.algorithmCount++;
        }
      }
    }

    return Array.from(categoryMap.values());
  }

  /**
   * Get faceted search interface
   */
  getFacetedSearch(): FacetedSearch {
    const algorithms = this.metadataStorage.getAll();
    const facets: Facet[] = [];

    // Build status facet
    const statusCounts = new Map<string, number>();
    for (const algorithm of algorithms) {
      const status = algorithm.metadata.status;
      statusCounts.set(status, (statusCounts.get(status) || 0) + 1);
    }

    facets.push({
      name: 'Status',
      type: 'status',
      options: Array.from(statusCounts.entries()).map(([value, count]) => ({
        value,
        label: value,
        count,
        selected: false,
      })),
      multiSelect: true,
    });

    // Build complexity facet
    const complexityCounts = new Map<string, number>();
    for (const algorithm of algorithms) {
      const complexity = algorithm.performance?.complexity || 'unknown';
      complexityCounts.set(complexity, (complexityCounts.get(complexity) || 0) + 1);
    }

    facets.push({
      name: 'Complexity',
      type: 'complexity',
      options: Array.from(complexityCounts.entries()).map(([value, count]) => ({
        value,
        label: value,
        count,
        selected: false,
      })),
      multiSelect: true,
    });

    return {
      facets,
      selectedFacets: {},
      results: [],
      totalResults: 0,
    };
  }

  /**
   * Track discovery session
   */
  trackSession(session: DiscoverySession): void {
    this.sessions.set(session.sessionId, session);
  }

  /**
   * Get session analytics
   */
  getSessionAnalytics(): Record<string, unknown> {
    const sessions = Array.from(this.sessions.values());
    
    return {
      totalSessions: sessions.length,
      averageSessionDuration: sessions.reduce((sum, s) => sum + s.sessionDuration, 0) / sessions.length,
      mostViewedAlgorithms: this.getMostViewedAlgorithms(),
      popularFilters: this.getPopularFilters(),
    };
  }

  /**
   * Get most viewed algorithms
   */
  private getMostViewedAlgorithms(): AlgorithmId[] {
    const viewCounts = new Map<AlgorithmId, number>();
    
    for (const session of this.sessions.values()) {
      for (const algorithmId of session.viewedAlgorithms) {
        viewCounts.set(algorithmId, (viewCounts.get(algorithmId) || 0) + 1);
      }
    }
    
    return Array.from(viewCounts.entries())
      .sort((a, b) => b[1] - a[1])
      .slice(0, 10)
      .map(([id]) => id);
  }

  /**
   * Get popular filters
   */
  private getPopularFilters(): Record<string, number> {
    const filterCounts: Record<string, number> = {};
    
    for (const session of this.sessions.values()) {
      for (const filter of session.filters) {
        const filterKey = JSON.stringify(filter);
        filterCounts[filterKey] = (filterCounts[filterKey] || 0) + 1;
      }
    }
    
    return filterCounts;
  }

  /**
   * Get system statistics
   */
  getStatistics(): Record<string, unknown> {
    return {
      totalAlgorithms: this.metadataStorage.getAll().length,
      searchIndexSize: this.searchIndex.size,
      cacheSize: this.cache.size,
      cacheHitRate: this.cacheHits / (this.cacheHits + this.cacheMisses),
      totalSessions: this.sessions.size,
      config: this.config,
    };
  }

  /**
   * Clear caches
   */
  clearCaches(): void {
    this.cache.clear();
    this.cacheHits = 0;
    this.cacheMisses = 0;
  }

  /**
   * Rebuild search index
   */
  rebuildSearchIndex(): void {
    this.searchIndex.clear();
    this.buildSearchIndex();
  }
} 