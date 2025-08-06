import { MetadataStorageSystem } from '../utils/metadata-storage-system';
import { AlgorithmDocumentation } from '../types/algorithm-documentation';
import { readFileSync, writeFileSync, existsSync, mkdirSync, readdirSync, unlinkSync, rmdirSync } from 'fs';
import { join } from 'path';

// Mock file system operations for testing
jest.mock('fs', () => ({
  readFileSync: jest.fn(),
  writeFileSync: jest.fn(),
  existsSync: jest.fn(),
  mkdirSync: jest.fn(),
  readdirSync: jest.fn(),
  unlinkSync: jest.fn(),
  rmdirSync: jest.fn(),
  copyFileSync: jest.fn(),
}));

describe('MetadataStorageSystem', () => {
  let storage: MetadataStorageSystem;
  let testDataPath: string;
  let testIndexPath: string;
  let testBackupPath: string;

  const mockAlgorithm: AlgorithmDocumentation = {
    id: 'test-algorithm',
    name: 'Test Algorithm',
    version: '1.0.0',
    taskDefinition: {
      title: 'Test Algorithm',
      description: 'A test algorithm for unit testing',
      problemStatement: 'Test problem statement',
      useCases: ['testing'],
      assumptions: ['test assumption'],
      limitations: ['test limitation'],
    },
    inputOutputSpecification: {
      inputs: [
        {
          name: 'input1',
          type: 'sensor_msgs/LaserScan',
          description: 'Test input',
          required: true,
        },
      ],
      outputs: [
        {
          name: 'output1',
          type: 'geometry_msgs/Pose',
          description: 'Test output',
        },
      ],
    },
    parameters: {
      configurable: [
        {
          name: 'param1',
          type: 'float',
          description: 'Test parameter',
          default: 1.0,
        },
      ],
    },
    dependencies: {
      rosPackages: [
        {
          name: 'test_package',
          purpose: 'Testing',
        },
      ],
    },
    metadata: {
      createdAt: '2024-01-01T00:00:00.000Z',
      updatedAt: '2024-01-01T00:00:00.000Z',
      status: 'stable',
      tags: ['test', 'algorithm'],
      categories: ['navigation'],
    },
  };

  beforeEach(() => {
    testDataPath = './test-data/algorithms';
    testIndexPath = './test-data/index';
    testBackupPath = './test-data/backups';
    
    storage = new MetadataStorageSystem({
      dataPath: testDataPath,
      indexPath: testIndexPath,
      backupPath: testBackupPath,
    });
    
    // Reset mocks
    jest.clearAllMocks();
    
    // Mock existsSync to return true for directories
    (existsSync as jest.Mock).mockReturnValue(true);
  });

  afterEach(() => {
    // Clean up test data
    try {
      if (existsSync(testDataPath)) {
        rmdirSync(testDataPath, { recursive: true });
      }
      if (existsSync(testIndexPath)) {
        rmdirSync(testIndexPath, { recursive: true });
      }
      if (existsSync(testBackupPath)) {
        rmdirSync(testBackupPath, { recursive: true });
      }
    } catch (error) {
      // Ignore cleanup errors
    }
  });

  describe('constructor', () => {
    it('should create directories if they do not exist', () => {
      (existsSync as jest.Mock).mockReturnValue(false);
      
      new MetadataStorageSystem({
        dataPath: testDataPath,
        indexPath: testIndexPath,
        backupPath: testBackupPath,
      });
      
      expect(mkdirSync).toHaveBeenCalledWith(testDataPath, { recursive: true });
      expect(mkdirSync).toHaveBeenCalledWith(testIndexPath, { recursive: true });
      expect(mkdirSync).toHaveBeenCalledWith(testBackupPath, { recursive: true });
    });

    it('should load existing index if available', () => {
      const mockIndexData = {
        byId: { 'test-algorithm': './test-data/algorithms/test-algorithm.json' },
        byName: { 'test algorithm': ['test-algorithm'] },
        byStatus: { 'stable': ['test-algorithm'] },
      };
      
      (existsSync as jest.Mock).mockImplementation((path: string) => 
        path === join(testIndexPath, 'metadata-index.json')
      );
      (readFileSync as jest.Mock).mockReturnValue(JSON.stringify(mockIndexData));
      
      const newStorage = new MetadataStorageSystem({
        dataPath: testDataPath,
        indexPath: testIndexPath,
        backupPath: testBackupPath,
      });
      
      expect(readFileSync).toHaveBeenCalledWith(join(testIndexPath, 'metadata-index.json'), 'utf-8');
    });
  });

  describe('save and getById', () => {
    it('should save algorithm and retrieve it by ID', () => {
      const result = storage.save(mockAlgorithm);
      
      expect(result).toBe(true);
      expect(writeFileSync).toHaveBeenCalledWith(
        join(testDataPath, 'test-algorithm.json'),
        JSON.stringify(mockAlgorithm, null, 2),
        'utf-8'
      );
      
      // Mock file read for getById
      (readFileSync as jest.Mock).mockReturnValue(JSON.stringify(mockAlgorithm));
      
      const retrieved = storage.getById('test-algorithm');
      expect(retrieved).toEqual(mockAlgorithm);
    });

    it('should use cache for subsequent getById calls', () => {
      // First call - load from file
      (readFileSync as jest.Mock).mockReturnValue(JSON.stringify(mockAlgorithm));
      
      const firstCall = storage.getById('test-algorithm');
      expect(firstCall).toEqual(mockAlgorithm);
      
      // Second call - should use cache
      const secondCall = storage.getById('test-algorithm');
      expect(secondCall).toEqual(mockAlgorithm);
      
      // readFileSync should only be called once
      expect(readFileSync).toHaveBeenCalledTimes(1);
    });

    it('should return null for non-existent algorithm', () => {
      (existsSync as jest.Mock).mockReturnValue(false);
      
      const result = storage.getById('non-existent');
      
      expect(result).toBeNull();
    });
  });

  describe('update', () => {
    it('should update existing algorithm', () => {
      // Save initial algorithm
      storage.save(mockAlgorithm);
      
      // Mock file read for getById
      (readFileSync as jest.Mock).mockReturnValue(JSON.stringify(mockAlgorithm));
      
      const updates = {
        name: 'Updated Algorithm',
        metadata: {
          status: 'beta' as const,
        },
      } as Partial<AlgorithmDocumentation>;
      
      const result = storage.update('test-algorithm', updates);
      
      expect(result).toBe(true);
      
      // Verify the updated algorithm was saved
      const updatedAlgorithm = { ...mockAlgorithm, ...updates };
      expect(writeFileSync).toHaveBeenCalledWith(
        join(testDataPath, 'test-algorithm.json'),
        JSON.stringify(updatedAlgorithm, null, 2),
        'utf-8'
      );
    });

    it('should return false for non-existent algorithm', () => {
      const result = storage.update('non-existent', { name: 'Updated' });
      
      expect(result).toBe(false);
    });
  });

  describe('delete', () => {
    it('should delete algorithm and remove from index', () => {
      // Save algorithm first
      storage.save(mockAlgorithm);
      
      // Mock file exists for deletion
      (existsSync as jest.Mock).mockReturnValue(true);
      
      const result = storage.delete('test-algorithm');
      
      expect(result).toBe(true);
      expect(unlinkSync).toHaveBeenCalledWith(join(testDataPath, 'test-algorithm.json'));
    });

    it('should return false for non-existent algorithm', () => {
      (existsSync as jest.Mock).mockReturnValue(false);
      
      const result = storage.delete('non-existent');
      
      expect(result).toBe(false);
    });
  });

  describe('search', () => {
    beforeEach(() => {
      // Add test algorithms
      storage.save(mockAlgorithm);
      
      const algorithm2 = {
        ...mockAlgorithm,
        id: 'test-algorithm-2',
        name: 'Another Test Algorithm',
        metadata: {
          ...mockAlgorithm.metadata,
          tags: ['test', 'another'],
          categories: ['perception'],
        },
      };
      storage.save(algorithm2);
    });

    it('should search by name', () => {
      const results = storage.search({ name: 'Test' });
      
      expect(results.data).toHaveLength(2);
      expect(results.total).toBe(2);
      expect(results.data[0].algorithm.name).toContain('Test');
    });

    it('should search by tags', () => {
      const results = storage.search({ tags: ['test'] });
      
      expect(results.data).toHaveLength(2);
      expect(results.data[0].matchedCriteria).toContain('tags');
    });

    it('should search by categories', () => {
      const results = storage.search({ categories: ['navigation'] });
      
      expect(results.data).toHaveLength(1);
      expect(results.data[0].algorithm.metadata.categories).toContain('navigation');
    });

    it('should support pagination', () => {
      const results = storage.search({ name: 'Test' }, { page: 1, pageSize: 1 });
      
      expect(results.data).toHaveLength(1);
      expect(results.total).toBe(2);
      expect(results.page).toBe(1);
      expect(results.pageSize).toBe(1);
      expect(results.totalPages).toBe(2);
      expect(results.hasNext).toBe(true);
      expect(results.hasPrevious).toBe(false);
    });

    it('should support sorting', () => {
      const results = storage.search({ name: 'Test' }, { sortBy: 'name', sortOrder: 'asc' });
      
      expect(results.data[0].algorithm.name).toBe('Another Test Algorithm');
      expect(results.data[1].algorithm.name).toBe('Test Algorithm');
    });

    it('should return empty results for no matches', () => {
      const results = storage.search({ name: 'NonExistent' });
      
      expect(results.data).toHaveLength(0);
      expect(results.total).toBe(0);
    });
  });

  describe('filtering methods', () => {
    beforeEach(() => {
      const algorithm1 = { ...mockAlgorithm, metadata: { ...mockAlgorithm.metadata, status: 'stable' } };
      const algorithm2 = { ...mockAlgorithm, id: 'test-2', metadata: { ...mockAlgorithm.metadata, status: 'beta' } };
      const algorithm3 = { ...mockAlgorithm, id: 'test-3', implementation: { language: 'Python' } };
      
      storage.save(algorithm1);
      storage.save(algorithm2);
      storage.save(algorithm3);
    });

    it('should filter by status', () => {
      const results = storage.getByStatus('stable');
      
      expect(results).toHaveLength(1);
      expect(results[0].metadata.status).toBe('stable');
    });

    it('should filter by language', () => {
      const results = storage.getByLanguage('Python');
      
      expect(results).toHaveLength(1);
      expect(results[0].implementation?.language).toBe('Python');
    });

    it('should filter by complexity', () => {
      const algorithm = { ...mockAlgorithm, performance: { complexity: 'O(n)' } };
      storage.save(algorithm);
      
      const results = storage.getByComplexity('O(n)');
      
      expect(results).toHaveLength(1);
      expect(results[0].performance?.complexity).toBe('O(n)');
    });

    it('should filter by tag', () => {
      const results = storage.getByTag('test');
      
      expect(results.length).toBeGreaterThan(0);
      expect(results[0].metadata.tags).toContain('test');
    });

    it('should filter by category', () => {
      const results = storage.getByCategory('navigation');
      
      expect(results.length).toBeGreaterThan(0);
      expect(results[0].metadata.categories).toContain('navigation');
    });

    it('should filter by package', () => {
      const results = storage.getByPackage('test_package');
      
      expect(results.length).toBeGreaterThan(0);
      expect(results[0].dependencies.rosPackages.some(pkg => pkg.name === 'test_package')).toBe(true);
    });
  });

  describe('validation', () => {
    it('should validate correct algorithm', () => {
      const result = storage.validate(mockAlgorithm);
      
      expect(result.isValid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    it('should detect missing required fields', () => {
      const invalidAlgorithm = { ...mockAlgorithm };
      delete (invalidAlgorithm as any).id;
      delete (invalidAlgorithm as any).name;
      
      const result = storage.validate(invalidAlgorithm);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Algorithm ID is required');
      expect(result.errors).toContain('Algorithm name is required');
    });

    it('should reject invalid algorithm on save', () => {
      const invalidAlgorithm = { ...mockAlgorithm };
      delete (invalidAlgorithm as any).id;
      
      const result = storage.save(invalidAlgorithm);
      
      expect(result).toBe(false);
    });
  });

  describe('statistics', () => {
    it('should generate comprehensive statistics', () => {
      const algorithm1 = { ...mockAlgorithm, metadata: { ...mockAlgorithm.metadata, status: 'stable' } };
      const algorithm2 = { ...mockAlgorithm, id: 'test-2', metadata: { ...mockAlgorithm.metadata, status: 'beta' } };
      const algorithm3 = { ...mockAlgorithm, id: 'test-3', implementation: { language: 'Python' } };
      
      storage.save(algorithm1);
      storage.save(algorithm2);
      storage.save(algorithm3);
      
      const stats = storage.getStatistics();
      
      expect(stats.total).toBe(3);
      expect(stats.statusCounts).toEqual({ stable: 1, beta: 1 });
      expect(stats.languageCounts).toEqual({ Python: 1 });
      expect(stats.cacheStats).toBeDefined();
      expect(stats.indexStats).toBeDefined();
    });
  });

  describe('backup and restore', () => {
    it('should create backup successfully', () => {
      storage.save(mockAlgorithm);
      
      const result = storage.createBackup();
      
      expect(result).toBe(true);
    });

    it('should handle backup creation errors gracefully', () => {
      (readdirSync as jest.Mock).mockImplementation(() => {
        throw new Error('Directory read error');
      });
      
      const result = storage.createBackup();
      
      expect(result).toBe(false);
    });
  });

  describe('import/export', () => {
    it('should export data to file', () => {
      storage.save(mockAlgorithm);
      
      const result = storage.exportToFile('./test-export.json');
      
      expect(result).toBe(true);
      expect(writeFileSync).toHaveBeenCalledWith(
        './test-export.json',
        expect.stringContaining('test-algorithm'),
        'utf-8'
      );
    });

    it('should import data from file', () => {
      const exportData = {
        algorithms: [mockAlgorithm],
        exportDate: '2024-01-01T00:00:00.000Z',
        total: 1,
      };
      
      (existsSync as jest.Mock).mockReturnValue(true);
      (readFileSync as jest.Mock).mockReturnValue(JSON.stringify(exportData));
      
      const result = storage.importFromFile('./test-import.json');
      
      expect(result).toBe(true);
    });

    it('should handle import errors gracefully', () => {
      (existsSync as jest.Mock).mockReturnValue(false);
      
      const result = storage.importFromFile('./non-existent.json');
      
      expect(result).toBe(false);
    });
  });

  describe('cache management', () => {
    it('should clear cache', () => {
      // Add to cache first
      (readFileSync as jest.Mock).mockReturnValue(JSON.stringify(mockAlgorithm));
      storage.getById('test-algorithm');
      
      storage.clearCache();
      
      // Cache should be empty
      const stats = storage.getStatistics();
      expect(stats.cacheStats.size).toBe(0);
    });
  });

  describe('index management', () => {
    it('should rebuild index', () => {
      storage.save(mockAlgorithm);
      
      storage.rebuildIndex();
      
      // Index should be rebuilt
      const stats = storage.getStatistics();
      expect(stats.indexStats.byId).toBeGreaterThan(0);
    });
  });
}); 