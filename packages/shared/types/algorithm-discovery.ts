// Algorithm Discovery Types
// Comprehensive type definitions for algorithm search, discovery, and recommendation

import { AlgorithmDocumentation, AlgorithmId } from './algorithm-documentation';

/**
 * Search query for algorithm discovery
 */
export interface DiscoveryQuery {
  text?: string;
  filters: DiscoveryFilters;
  sorting: DiscoverySorting;
  pagination: DiscoveryPagination;
  includeSimilar?: boolean;
  includeAlternatives?: boolean;
}

/**
 * Search filters for algorithm discovery
 */
export interface DiscoveryFilters {
  metaCategories?: string[];
  taskCategories?: string[];
  complexity?: ('basic' | 'intermediate' | 'advanced' | 'expert')[];
  status?: ('stable' | 'beta' | 'experimental' | 'deprecated')[];
  language?: ('C++' | 'Python' | 'C' | 'Java')[];
  tags?: string[];
  packages?: string[];
  dependencies?: string[];
  performance?: {
    maxComplexity?: string;
    maxMemoryUsage?: string;
    maxExecutionTime?: string;
  };
  compatibility?: {
    rosVersion?: string;
    platform?: string;
    architecture?: string;
  };
  requirements?: {
    hardware?: string[];
    sensors?: string[];
    capabilities?: string[];
  };
  dateRange?: {
    createdAfter?: string;
    createdBefore?: string;
    updatedAfter?: string;
    updatedBefore?: string;
  };
}

/**
 * Sorting options for algorithm discovery
 */
export interface DiscoverySorting {
  field: 'relevance' | 'name' | 'complexity' | 'status' | 'createdAt' | 'updatedAt' | 'popularity';
  order: 'asc' | 'desc';
  secondaryField?: string;
  secondaryOrder?: 'asc' | 'desc';
}

/**
 * Pagination options for algorithm discovery
 */
export interface DiscoveryPagination {
  page: number;
  pageSize: number;
  maxResults?: number;
}

/**
 * Discovery result with relevance scoring
 */
export interface DiscoveryResult {
  algorithm: AlgorithmDocumentation;
  relevanceScore: number;
  matchedCriteria: string[];
  similarityScore?: number;
  alternativeScore?: number;
  popularityScore?: number;
  compatibilityScore?: number;
  recommendations?: string[];
}

/**
 * Discovery response with results and metadata
 */
export interface DiscoveryResponse {
  results: DiscoveryResult[];
  totalResults: number;
  page: number;
  pageSize: number;
  totalPages: number;
  hasNext: boolean;
  hasPrevious: boolean;
  query: DiscoveryQuery;
  statistics: DiscoveryStatistics;
  suggestions?: DiscoverySuggestion[];
}

/**
 * Discovery statistics
 */
export interface DiscoveryStatistics {
  searchTime: number;
  filterTime: number;
  sortTime: number;
  totalTime: number;
  cacheHits: number;
  cacheMisses: number;
  resultCount: number;
  filterCounts: Record<string, number>;
}

/**
 * Discovery suggestions for improving search
 */
export interface DiscoverySuggestion {
  type: 'spelling' | 'synonym' | 'category' | 'tag' | 'related';
  text: string;
  confidence: number;
  reason: string;
}

/**
 * Algorithm recommendation criteria
 */
export interface RecommendationCriteria {
  algorithmId: AlgorithmId;
  context?: {
    projectType?: string;
    useCase?: string;
    constraints?: string[];
    preferences?: string[];
  };
  similarity?: {
    basedOn?: 'functionality' | 'complexity' | 'dependencies' | 'performance' | 'all';
    threshold?: number;
    maxResults?: number;
  };
  alternatives?: {
    include?: boolean;
    reason?: string;
    maxResults?: number;
  };
}

/**
 * Algorithm recommendation result
 */
export interface RecommendationResult {
  algorithm: AlgorithmDocumentation;
  score: number;
  reason: string;
  similarityFactors: string[];
  alternatives?: AlgorithmDocumentation[];
  warnings?: string[];
  suggestions?: string[];
}

/**
 * Browsing interface for algorithm exploration
 */
export interface BrowsingInterface {
  categories: CategoryNode[];
  filters: BrowsingFilters;
  view: 'grid' | 'list' | 'tree' | 'table';
  grouping: 'category' | 'complexity' | 'status' | 'language' | 'none';
  sorting: DiscoverySorting;
}

/**
 * Category tree node for browsing
 */
export interface CategoryNode {
  id: string;
  name: string;
  description?: string;
  type: 'meta-category' | 'task-category' | 'subcategory';
  children?: CategoryNode[];
  algorithmCount: number;
  path: string[];
  metadata?: Record<string, unknown>;
}

/**
 * Browsing filters
 */
export interface BrowsingFilters {
  searchText?: string;
  selectedCategories?: string[];
  complexityRange?: [string, string];
  statusFilter?: string[];
  languageFilter?: string[];
  tagFilter?: string[];
  dateRange?: [string, string];
}

/**
 * Faceted search interface
 */
export interface FacetedSearch {
  facets: Facet[];
  selectedFacets: Record<string, string[]>;
  results: DiscoveryResult[];
  totalResults: number;
}

/**
 * Search facet with options
 */
export interface Facet {
  name: string;
  type: 'category' | 'tag' | 'status' | 'language' | 'complexity' | 'date';
  options: FacetOption[];
  multiSelect: boolean;
  hierarchical?: boolean;
}

/**
 * Facet option
 */
export interface FacetOption {
  value: string;
  label: string;
  count: number;
  selected: boolean;
  children?: FacetOption[];
}

/**
 * Search engine configuration
 */
export interface SearchEngineConfig {
  indexPath: string;
  cacheEnabled: boolean;
  cacheSize: number;
  maxResults: number;
  relevanceThreshold: number;
  fuzzySearch: boolean;
  synonymExpansion: boolean;
  stemming: boolean;
  stopWords: string[];
  boostFactors: Record<string, number>;
}

/**
 * Search index metadata
 */
export interface SearchIndexMetadata {
  version: string;
  createdAt: string;
  lastUpdated: string;
  totalDocuments: number;
  totalTerms: number;
  indexSize: number;
  statistics: Record<string, unknown>;
}

/**
 * Search performance metrics
 */
export interface SearchPerformanceMetrics {
  queryTime: number;
  indexTime: number;
  filterTime: number;
  sortTime: number;
  totalTime: number;
  memoryUsage: number;
  cacheHitRate: number;
  throughput: number;
}

/**
 * Algorithm comparison criteria
 */
export interface ComparisonCriteria {
  algorithms: AlgorithmId[];
  comparisonFields: ('performance' | 'complexity' | 'dependencies' | 'compatibility' | 'documentation')[];
  includeDetails?: boolean;
  format?: 'table' | 'chart' | 'detailed';
}

/**
 * Algorithm comparison result
 */
export interface ComparisonResult {
  algorithms: AlgorithmDocumentation[];
  comparison: Record<string, Record<string, unknown>>;
  summary: ComparisonSummary;
  recommendations: string[];
  visualization?: unknown; // Chart data or visualization config
}

/**
 * Comparison summary
 */
export interface ComparisonSummary {
  bestOverall?: AlgorithmId;
  bestByField: Record<string, AlgorithmId>;
  tradeoffs: string[];
  compatibility: Record<string, boolean>;
}

/**
 * Discovery session for tracking user behavior
 */
export interface DiscoverySession {
  sessionId: string;
  userId?: string;
  startTime: string;
  endTime?: string;
  queries: DiscoveryQuery[];
  viewedAlgorithms: AlgorithmId[];
  selectedAlgorithms: AlgorithmId[];
  filters: DiscoveryFilters[];
  sessionDuration: number;
  metadata: Record<string, unknown>;
}

/**
 * Popularity metrics for algorithms
 */
export interface PopularityMetrics {
  algorithmId: AlgorithmId;
  viewCount: number;
  downloadCount: number;
  rating: number;
  reviewCount: number;
  usageCount: number;
  lastUpdated: string;
  trend: 'increasing' | 'decreasing' | 'stable';
}

// Type aliases for convenience
export type SearchField = keyof DiscoveryFilters;
export type SortField = DiscoverySorting['field'];
export type FilterType = Facet['type'];
export type RecommendationType = 'similar' | 'alternative' | 'complementary' | 'upgrade' | 'downgrade'; 