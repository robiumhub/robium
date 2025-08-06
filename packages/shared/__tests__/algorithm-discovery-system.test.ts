import { AlgorithmDiscoverySystem } from '../utils/algorithm-discovery-system';
import { MetadataStorageSystem } from '../utils/metadata-storage-system';
import { AlgorithmDocumentation } from '../types/algorithm-documentation';
import {
  DiscoveryQuery,
  DiscoveryFilters,
  DiscoverySorting,
  DiscoveryPagination,
  RecommendationCriteria,
  ComparisonCriteria,
  DiscoverySession,
} from '../types/algorithm-discovery';
import { readFileSync, writeFileSync, existsSync, mkdirSync, readdirSync } from 'fs';
import { join } from 'path';

// Mock file system operations for testing
jest.mock('fs', () => ({
  readFileSync: jest.fn(),
  writeFileSync: jest.fn(),
  existsSync: jest.fn(),
  mkdirSync: jest.fn(),
  readdirSync: jest.fn(),
}));

describe('AlgorithmDiscoverySystem', () => {
  let discoverySystem: AlgorithmDiscoverySystem;
  let metadataStorage: MetadataStorageSystem;
  let mockIndexPath: string;

  const mockAlgorithm1: AlgorithmDocumentation = {
    id: 'navigation-algorithm-1',
    name: 'A* Path Planning',
    version: '1.0.0',
    taskDefinition: {
      title: 'A* Path Planning Algorithm',
      description: 'Efficient path planning using A* algorithm for robot navigation',
      problemStatement: 'Find optimal path from start to goal while avoiding obstacles',
    },
    inputOutputSpecification: {
      inputs: [
        {
          name: 'start_pose',
          type: 'geometry_msgs/Pose',
          description: 'Starting position',
          required: true,
        },
        {
          name: 'goal_pose',
          type: 'geometry_msgs/Pose',
          description: 'Goal position',
          required: true,
        },
      ],
      outputs: [
        {
          name: 'path',
          type: 'nav_msgs/Path',
          description: 'Computed path',
        },
      ],
    },
    parameters: {
      configurable: [
        {
          name: 'heuristic_weight',
          type: 'float',
          description: 'Weight for heuristic function',
          default: 1.0,
        },
      ],
    },
    dependencies: {
      rosPackages: [
        {
          name: 'nav2_core',
          version: '1.0.0',
          purpose: 'Navigation framework',
          optional: false,
        },
      ],
    },
    performance: {
      complexity: 'O(n log n)',
      executionTime: {
        typical: '10ms',
      },
    },
    implementation: {
      language: 'C++',
      architecture: 'Plugin-based',
    },
    metadata: {
      createdAt: '2024-01-01T00:00:00Z',
      updatedAt: '2024-01-01T00:00:00Z',
      status: 'stable' as const,
      tags: ['navigation', 'path-planning', 'a-star'],
      categories: ['navigation', 'planning'],
    },
  };

  const mockAlgorithm2: AlgorithmDocumentation = {
    id: 'navigation-algorithm-2',
    name: 'Dijkstra Path Planning',
    version: '1.0.0',
    taskDefinition: {
      title: 'Dijkstra Path Planning Algorithm',
      description: 'Classic shortest path algorithm for robot navigation',
      problemStatement: 'Find shortest path from start to goal',
    },
    inputOutputSpecification: {
      inputs: [
        {
          name: 'start_pose',
          type: 'geometry_msgs/Pose',
          description: 'Starting position',
          required: true,
        },
        {
          name: 'goal_pose',
          type: 'geometry_msgs/Pose',
          description: 'Goal position',
          required: true,
        },
      ],
      outputs: [
        {
          name: 'path',
          type: 'nav_msgs/Path',
          description: 'Computed path',
        },
      ],
    },
    parameters: {
      configurable: [
        {
          name: 'cost_weight',
          type: 'float',
          description: 'Cost weight for path calculation',
          default: 1.0,
        },
      ],
    },
    dependencies: {
      rosPackages: [
        {
          name: 'nav2_core',
          version: '1.0.0',
          purpose: 'Navigation framework',
          optional: false,
        },
      ],
    },
    performance: {
      complexity: 'O(n²)',
      executionTime: {
        typical: '50ms',
      },
    },
    implementation: {
      language: 'C++',
      architecture: 'Plugin-based',
    },
    metadata: {
      createdAt: '2024-01-02T00:00:00Z',
      updatedAt: '2024-01-02T00:00:00Z',
      status: 'stable' as const,
      tags: ['navigation', 'path-planning', 'dijkstra'],
      categories: ['navigation', 'planning'],
    },
  };

  const mockAlgorithm3: AlgorithmDocumentation = {
    id: 'perception-algorithm-1',
    name: 'Object Detection',
    version: '1.0.0',
    taskDefinition: {
      title: 'Object Detection Algorithm',
      description: 'Detect objects in camera images using deep learning',
      problemStatement: 'Identify and locate objects in image data',
    },
    inputOutputSpecification: {
      inputs: [
        {
          name: 'image',
          type: 'sensor_msgs/Image',
          description: 'Input image',
          required: true,
        },
      ],
      outputs: [
        {
          name: 'detections',
          type: 'vision_msgs/DetectionArray',
          description: 'Detected objects',
        },
      ],
    },
    parameters: {
      configurable: [
        {
          name: 'confidence_threshold',
          type: 'float',
          description: 'Detection confidence threshold',
          default: 0.5,
        },
      ],
    },
    dependencies: {
      rosPackages: [
        {
          name: 'vision_msgs',
          version: '1.0.0',
          purpose: 'Vision message types',
          optional: false,
        },
      ],
    },
    performance: {
      complexity: 'O(n)',
      executionTime: {
        typical: '100ms',
      },
    },
    implementation: {
      language: 'Python',
      architecture: 'Neural network',
    },
    metadata: {
      createdAt: '2024-01-03T00:00:00Z',
      updatedAt: '2024-01-03T00:00:00Z',
      status: 'beta' as const,
      tags: ['perception', 'object-detection', 'deep-learning'],
      categories: ['perception', 'vision'],
    },
  };

  beforeEach(() => {
    mockIndexPath = './test-search-index';
    
    // Mock metadata storage
    metadataStorage = {
      getAll: jest.fn().mockReturnValue([mockAlgorithm1, mockAlgorithm2, mockAlgorithm3]),
      getById: jest.fn().mockImplementation((id: string) => {
        const algorithms = [mockAlgorithm1, mockAlgorithm2, mockAlgorithm3];
        return algorithms.find(alg => alg.id === id) || null;
      }),
    } as any;

    // Mock file system operations
    (existsSync as jest.Mock).mockReturnValue(true);
    (readdirSync as jest.Mock).mockReturnValue([]);
    (readFileSync as jest.Mock).mockReturnValue('{}');

    discoverySystem = new AlgorithmDiscoverySystem(metadataStorage, {
      indexPath: mockIndexPath,
    });
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  describe('constructor', () => {
    it('should initialize with default configuration', () => {
      expect(discoverySystem).toBeDefined();
    });

    it('should build search index on initialization', () => {
      expect(metadataStorage.getAll).toHaveBeenCalled();
    });

    it('should create required directories', () => {
      expect(mkdirSync).toHaveBeenCalled();
    });
  });

  describe('search', () => {
    const mockQuery: DiscoveryQuery = {
      text: 'path planning',
      filters: {
        metaCategories: ['navigation'],
        complexity: ['basic', 'intermediate'],
        status: ['stable'],
      },
      sorting: {
        field: 'relevance',
        order: 'desc',
      },
      pagination: {
        page: 1,
        pageSize: 10,
      },
    };

    it('should perform text-based search', () => {
      const result = discoverySystem.search(mockQuery);

      expect(result.results).toBeDefined();
      expect(result.totalResults).toBeGreaterThan(0);
      expect(result.statistics).toBeDefined();
      expect(result.query).toEqual(mockQuery);
    });

    it('should apply filters correctly', () => {
      const result = discoverySystem.search(mockQuery);

      // Should only return navigation algorithms
      for (const discoveryResult of result.results) {
        expect(discoveryResult.algorithm.metadata.categories).toContain('navigation');
      }
    });

    it('should handle empty search results', () => {
      const emptyQuery: DiscoveryQuery = {
        text: 'nonexistent algorithm',
        filters: {},
        sorting: { field: 'relevance', order: 'desc' },
        pagination: { page: 1, pageSize: 10 },
      };

      const result = discoverySystem.search(emptyQuery);

      expect(result.results.length).toBe(0);
      expect(result.totalResults).toBe(0);
      expect(result.suggestions).toBeDefined();
    });

    it('should apply pagination correctly', () => {
      const paginatedQuery: DiscoveryQuery = {
        ...mockQuery,
        pagination: { page: 1, pageSize: 2 },
      };

      const result = discoverySystem.search(paginatedQuery);

      expect(result.results.length).toBeLessThanOrEqual(2);
      expect(result.pageSize).toBe(2);
    });

    it('should calculate relevance scores', () => {
      const result = discoverySystem.search(mockQuery);

      for (const discoveryResult of result.results) {
        expect(discoveryResult.relevanceScore).toBeGreaterThan(0);
        expect(discoveryResult.relevanceScore).toBeLessThanOrEqual(1);
      }
    });

    it('should provide matched criteria', () => {
      const result = discoverySystem.search(mockQuery);

      for (const discoveryResult of result.results) {
        expect(discoveryResult.matchedCriteria).toBeDefined();
        expect(Array.isArray(discoveryResult.matchedCriteria)).toBe(true);
      }
    });
  });

  describe('getRecommendations', () => {
    const mockCriteria: RecommendationCriteria = {
      algorithmId: 'navigation-algorithm-1',
      similarity: {
        basedOn: 'functionality',
        threshold: 0.5,
        maxResults: 5,
      },
    };

    it('should provide algorithm recommendations', () => {
      const recommendations = discoverySystem.getRecommendations(mockCriteria);

      expect(recommendations).toBeDefined();
      expect(Array.isArray(recommendations)).toBe(true);
    });

    it('should calculate similarity scores', () => {
      const recommendations = discoverySystem.getRecommendations(mockCriteria);

      for (const recommendation of recommendations) {
        expect(recommendation.score).toBeGreaterThan(0);
        expect(recommendation.score).toBeLessThanOrEqual(1);
        expect(recommendation.reason).toBeDefined();
        expect(recommendation.similarityFactors).toBeDefined();
      }
    });

    it('should not recommend the same algorithm', () => {
      const recommendations = discoverySystem.getRecommendations(mockCriteria);

      for (const recommendation of recommendations) {
        expect(recommendation.algorithm.id).not.toBe('navigation-algorithm-1');
      }
    });

    it('should respect similarity threshold', () => {
      const highThresholdCriteria: RecommendationCriteria = {
        ...mockCriteria,
        similarity: { ...mockCriteria.similarity!, threshold: 0.9 },
      };

      const recommendations = discoverySystem.getRecommendations(highThresholdCriteria);

      for (const recommendation of recommendations) {
        expect(recommendation.score).toBeGreaterThanOrEqual(0.9);
      }
    });

    it('should limit results based on maxResults', () => {
      const limitedCriteria: RecommendationCriteria = {
        ...mockCriteria,
        similarity: { ...mockCriteria.similarity!, maxResults: 1 },
      };

      const recommendations = discoverySystem.getRecommendations(limitedCriteria);

      expect(recommendations.length).toBeLessThanOrEqual(1);
    });
  });

  describe('compareAlgorithms', () => {
    const mockComparisonCriteria: ComparisonCriteria = {
      algorithms: ['navigation-algorithm-1', 'navigation-algorithm-2'],
      comparisonFields: ['performance', 'complexity', 'dependencies'],
    };

    it('should compare algorithms successfully', () => {
      const result = discoverySystem.compareAlgorithms(mockComparisonCriteria);

      expect(result.algorithms).toBeDefined();
      expect(result.algorithms.length).toBe(2);
      expect(result.comparison).toBeDefined();
      expect(result.summary).toBeDefined();
      expect(result.recommendations).toBeDefined();
    });

    it('should include all requested comparison fields', () => {
      const result = discoverySystem.compareAlgorithms(mockComparisonCriteria);

      expect(result.comparison).toHaveProperty('performance');
      expect(result.comparison).toHaveProperty('complexity');
      expect(result.comparison).toHaveProperty('dependencies');
    });

    it('should provide comparison summary', () => {
      const result = discoverySystem.compareAlgorithms(mockComparisonCriteria);

      expect(result.summary.bestByField).toBeDefined();
      expect(result.summary.tradeoffs).toBeDefined();
      expect(result.summary.compatibility).toBeDefined();
    });

    it('should throw error for insufficient algorithms', () => {
      const invalidCriteria: ComparisonCriteria = {
        algorithms: ['navigation-algorithm-1'],
        comparisonFields: ['performance'],
      };

      expect(() => {
        discoverySystem.compareAlgorithms(invalidCriteria);
      }).toThrow('At least 2 algorithms are required for comparison');
    });

    it('should handle missing algorithms gracefully', () => {
      const criteriaWithMissingAlg: ComparisonCriteria = {
        algorithms: ['navigation-algorithm-1', 'nonexistent-algorithm'],
        comparisonFields: ['performance'],
      };

      const result = discoverySystem.compareAlgorithms(criteriaWithMissingAlg);

      expect(result.algorithms.length).toBe(1);
    });
  });

  describe('getBrowsingInterface', () => {
    it('should provide browsing interface', () => {
      const browsingInterface = discoverySystem.getBrowsingInterface();

      expect(browsingInterface.categories).toBeDefined();
      expect(browsingInterface.filters).toBeDefined();
      expect(browsingInterface.view).toBeDefined();
      expect(browsingInterface.grouping).toBeDefined();
      expect(browsingInterface.sorting).toBeDefined();
    });

    it('should build category tree', () => {
      const browsingInterface = discoverySystem.getBrowsingInterface();

      expect(browsingInterface.categories.length).toBeGreaterThan(0);
      
      for (const category of browsingInterface.categories) {
        expect(category.id).toBeDefined();
        expect(category.name).toBeDefined();
        expect(category.type).toBeDefined();
        expect(category.algorithmCount).toBeGreaterThan(0);
        expect(category.path).toBeDefined();
      }
    });

    it('should provide default filters', () => {
      const browsingInterface = discoverySystem.getBrowsingInterface();

      expect(browsingInterface.filters.searchText).toBe('');
      expect(browsingInterface.filters.selectedCategories).toEqual([]);
      expect(browsingInterface.filters.complexityRange).toEqual(['basic', 'expert']);
    });
  });

  describe('getFacetedSearch', () => {
    it('should provide faceted search interface', () => {
      const facetedSearch = discoverySystem.getFacetedSearch();

      expect(facetedSearch.facets).toBeDefined();
      expect(facetedSearch.selectedFacets).toBeDefined();
      expect(facetedSearch.results).toBeDefined();
      expect(facetedSearch.totalResults).toBeDefined();
    });

    it('should build facets from algorithm data', () => {
      const facetedSearch = discoverySystem.getFacetedSearch();

      expect(facetedSearch.facets.length).toBeGreaterThan(0);
      
      for (const facet of facetedSearch.facets) {
        expect(facet.name).toBeDefined();
        expect(facet.type).toBeDefined();
        expect(facet.options).toBeDefined();
        expect(facet.multiSelect).toBeDefined();
      }
    });

    it('should include status facet', () => {
      const facetedSearch = discoverySystem.getFacetedSearch();

      const statusFacet = facetedSearch.facets.find(f => f.type === 'status');
      expect(statusFacet).toBeDefined();
      expect(statusFacet!.options.length).toBeGreaterThan(0);
    });

    it('should include complexity facet', () => {
      const facetedSearch = discoverySystem.getFacetedSearch();

      const complexityFacet = facetedSearch.facets.find(f => f.type === 'complexity');
      expect(complexityFacet).toBeDefined();
      expect(complexityFacet!.options.length).toBeGreaterThan(0);
    });
  });

  describe('trackSession', () => {
    it('should track discovery session', () => {
      const session: DiscoverySession = {
        sessionId: 'test-session-1',
        startTime: new Date().toISOString(),
        queries: [],
        viewedAlgorithms: [],
        selectedAlgorithms: [],
        filters: [],
        sessionDuration: 0,
        metadata: {},
      };

      discoverySystem.trackSession(session);

      // No direct way to verify tracking, but should not throw error
      expect(() => discoverySystem.trackSession(session)).not.toThrow();
    });
  });

  describe('getSessionAnalytics', () => {
    it('should provide session analytics', () => {
      const analytics = discoverySystem.getSessionAnalytics();

      expect(analytics.totalSessions).toBeDefined();
      expect(analytics.averageSessionDuration).toBeDefined();
      expect(analytics.mostViewedAlgorithms).toBeDefined();
      expect(analytics.popularFilters).toBeDefined();
    });
  });

  describe('getStatistics', () => {
    it('should provide system statistics', () => {
      const stats = discoverySystem.getStatistics();

      expect(stats.totalAlgorithms).toBeDefined();
      expect(stats.searchIndexSize).toBeDefined();
      expect(stats.cacheSize).toBeDefined();
      expect(stats.cacheHitRate).toBeDefined();
      expect(stats.totalSessions).toBeDefined();
      expect(stats.config).toBeDefined();
    });
  });

  describe('clearCaches', () => {
    it('should clear caches', () => {
      expect(() => discoverySystem.clearCaches()).not.toThrow();
    });
  });

  describe('rebuildSearchIndex', () => {
    it('should rebuild search index', () => {
      expect(() => discoverySystem.rebuildSearchIndex()).not.toThrow();
    });
  });

  describe('error handling', () => {
    it('should handle file system errors gracefully', () => {
      (readdirSync as jest.Mock).mockImplementation(() => {
        throw new Error('File system error');
      });

      // Should not throw error during construction
      expect(() => {
        new AlgorithmDiscoverySystem(metadataStorage, { indexPath: mockIndexPath });
      }).not.toThrow();
    });

    it('should handle invalid search queries gracefully', () => {
      const invalidQuery: DiscoveryQuery = {
        text: '',
        filters: {},
        sorting: { field: 'relevance', order: 'desc' },
        pagination: { page: 0, pageSize: 0 }, // Invalid pagination
      };

      const result = discoverySystem.search(invalidQuery);

      expect(result.results).toBeDefined();
      expect(Array.isArray(result.results)).toBe(true);
    });
  });

  describe('search performance', () => {
    it('should provide search performance metrics', () => {
      const query: DiscoveryQuery = {
        text: 'test',
        filters: {},
        sorting: { field: 'relevance', order: 'desc' },
        pagination: { page: 1, pageSize: 10 },
      };

      const result = discoverySystem.search(query);

      expect(result.statistics.searchTime).toBeGreaterThan(0);
      expect(result.statistics.totalTime).toBeGreaterThan(0);
      expect(result.statistics.cacheHits).toBeDefined();
      expect(result.statistics.cacheMisses).toBeDefined();
    });
  });
}); 