import { AlgorithmDocumentationManager } from '../utils/algorithm-documentation-manager';
import { AlgorithmDocumentation, AlgorithmDocumentationTemplate } from '../types/algorithm-documentation';
import { readFileSync, writeFileSync, existsSync, mkdirSync, unlinkSync, rmdirSync } from 'fs';
import { join } from 'path';

// Mock file system operations for testing
jest.mock('fs', () => ({
  readFileSync: jest.fn(),
  writeFileSync: jest.fn(),
  existsSync: jest.fn(),
  mkdirSync: jest.fn(),
  unlinkSync: jest.fn(),
  rmdirSync: jest.fn(),
}));

// Mock require for readdirSync
const mockReaddirSync = jest.fn();
jest.mock('fs', () => ({
  ...jest.requireActual('fs'),
  readdirSync: mockReaddirSync,
}));

describe('AlgorithmDocumentationManager', () => {
  let manager: AlgorithmDocumentationManager;
  let testDataPath: string;

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
    manager = new AlgorithmDocumentationManager(testDataPath);
    
    // Reset mocks
    jest.clearAllMocks();
    
    // Mock existsSync to return true for directory
    (existsSync as jest.Mock).mockReturnValue(true);
  });

  afterEach(() => {
    // Clean up test data
    try {
      if (existsSync(testDataPath)) {
        rmdirSync(testDataPath, { recursive: true });
      }
    } catch (error) {
      // Ignore cleanup errors
    }
  });

  describe('constructor', () => {
    it('should create data directory if it does not exist', () => {
      (existsSync as jest.Mock).mockReturnValue(false);
      
      new AlgorithmDocumentationManager(testDataPath);
      
      expect(mkdirSync).toHaveBeenCalledWith(testDataPath, { recursive: true });
    });
  });

  describe('loadFromFile', () => {
    it('should load algorithm from file successfully', () => {
      const filePath = join(testDataPath, 'test-algorithm.json');
      (existsSync as jest.Mock).mockImplementation((path: string) => path === filePath);
      (readFileSync as jest.Mock).mockReturnValue(JSON.stringify(mockAlgorithm));
      
      const result = manager.loadFromFile('test-algorithm');
      
      expect(result).toEqual(mockAlgorithm);
      expect(readFileSync).toHaveBeenCalledWith(filePath, 'utf-8');
    });

    it('should return null if file does not exist', () => {
      (existsSync as jest.Mock).mockReturnValue(false);
      
      const result = manager.loadFromFile('non-existent');
      
      expect(result).toBeNull();
    });

    it('should return null on JSON parse error', () => {
      const filePath = join(testDataPath, 'test-algorithm.json');
      (existsSync as jest.Mock).mockImplementation((path: string) => path === filePath);
      (readFileSync as jest.Mock).mockReturnValue('invalid json');
      
      const result = manager.loadFromFile('test-algorithm');
      
      expect(result).toBeNull();
    });
  });

  describe('saveToFile', () => {
    it('should save algorithm to file successfully', () => {
      const result = manager.saveToFile(mockAlgorithm);
      
      expect(result).toBe(true);
      expect(writeFileSync).toHaveBeenCalledWith(
        join(testDataPath, 'test-algorithm.json'),
        JSON.stringify(mockAlgorithm, null, 2),
        'utf-8'
      );
    });

    it('should return false on write error', () => {
      (writeFileSync as jest.Mock).mockImplementation(() => {
        throw new Error('Write error');
      });
      
      const result = manager.saveToFile(mockAlgorithm);
      
      expect(result).toBe(false);
    });
  });

  describe('validate', () => {
    it('should validate correct algorithm successfully', () => {
      const result = manager.validate(mockAlgorithm);
      
      expect(result.isValid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    it('should detect missing required fields', () => {
      const invalidAlgorithm = { ...mockAlgorithm };
      delete (invalidAlgorithm as any).id;
      delete (invalidAlgorithm as any).name;
      
      const result = manager.validate(invalidAlgorithm);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Algorithm ID is required');
      expect(result.errors).toContain('Algorithm name is required');
    });

    it('should detect missing task definition', () => {
      const invalidAlgorithm = { ...mockAlgorithm };
      delete (invalidAlgorithm as any).taskDefinition;
      
      const result = manager.validate(invalidAlgorithm);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Task definition is required');
    });

    it('should detect missing input/output specification', () => {
      const invalidAlgorithm = { ...mockAlgorithm };
      delete (invalidAlgorithm as any).inputOutputSpecification;
      
      const result = manager.validate(invalidAlgorithm);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Input/Output specification is required');
    });

    it('should provide warnings for missing optional sections', () => {
      const minimalAlgorithm = { ...mockAlgorithm };
      delete (minimalAlgorithm as any).performance;
      delete (minimalAlgorithm as any).implementation;
      delete (minimalAlgorithm as any).testing;
      delete (minimalAlgorithm as any).documentation;
      
      const result = manager.validate(minimalAlgorithm);
      
      expect(result.isValid).toBe(true);
      expect(result.warnings).toContain('Performance metrics are not specified');
      expect(result.warnings).toContain('Implementation details are not specified');
      expect(result.warnings).toContain('Testing information is not specified');
      expect(result.warnings).toContain('Documentation resources are not specified');
    });
  });

  describe('createFromTemplate', () => {
    it('should create algorithm from valid template', () => {
      const template: AlgorithmDocumentationTemplate = {
        taskDefinition: {
          title: 'Template Algorithm',
          description: 'Algorithm created from template',
          problemStatement: 'Template problem',
        },
        inputOutputSpecification: {
          inputs: [],
          outputs: [],
        },
        parameters: {
          configurable: [],
        },
        dependencies: {
          rosPackages: [],
        },
        metadata: {
          status: 'experimental',
          tags: ['template'],
        },
      };
      
      const result = manager.createFromTemplate('template-algorithm', template);
      
      expect(result).not.toBeNull();
      expect(result?.id).toBe('template-algorithm');
      expect(result?.name).toBe('template-algorithm');
      expect(result?.taskDefinition.title).toBe('Template Algorithm');
      expect(result?.metadata.status).toBe('experimental');
    });

    it('should return null for invalid template', () => {
      const invalidTemplate: AlgorithmDocumentationTemplate = {
        taskDefinition: {
          title: '',
          description: '',
          problemStatement: '',
        },
        inputOutputSpecification: {
          inputs: [],
          outputs: [],
        },
        parameters: {
          configurable: [],
        },
        dependencies: {
          rosPackages: [],
        },
        metadata: {
          status: 'experimental',
        },
      };
      
      const result = manager.createFromTemplate('invalid-template', invalidTemplate);
      
      expect(result).toBeNull();
    });
  });

  describe('search', () => {
    beforeEach(() => {
      // Add test algorithms to manager
      manager.saveToFile(mockAlgorithm);
      
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
      manager.saveToFile(algorithm2);
    });

    it('should search by name', () => {
      const results = manager.search({ name: 'Test' });
      
      expect(results).toHaveLength(2);
      expect(results[0].algorithm.name).toContain('Test');
    });

    it('should search by tags', () => {
      const results = manager.search({ tags: ['test'] });
      
      expect(results).toHaveLength(2);
      expect(results[0].matchedCriteria).toContain('tags');
    });

    it('should search by categories', () => {
      const results = manager.search({ categories: ['navigation'] });
      
      expect(results).toHaveLength(1);
      expect(results[0].algorithm.metadata.categories).toContain('navigation');
    });

    it('should search by status', () => {
      const results = manager.search({ status: ['stable'] });
      
      expect(results).toHaveLength(2);
      expect(results[0].matchedCriteria).toContain('status');
    });

    it('should return empty results for no matches', () => {
      const results = manager.search({ name: 'NonExistent' });
      
      expect(results).toHaveLength(0);
    });
  });

  describe('CRUD operations', () => {
    it('should get algorithm by ID', () => {
      manager.saveToFile(mockAlgorithm);
      
      const result = manager.getById('test-algorithm');
      
      expect(result).toEqual(mockAlgorithm);
    });

    it('should return null for non-existent algorithm', () => {
      const result = manager.getById('non-existent');
      
      expect(result).toBeNull();
    });

    it('should get all algorithms', () => {
      manager.saveToFile(mockAlgorithm);
      
      const algorithm2 = { ...mockAlgorithm, id: 'test-algorithm-2' };
      manager.saveToFile(algorithm2);
      
      const results = manager.getAll();
      
      expect(results).toHaveLength(2);
      expect(results.map(a => a.id)).toContain('test-algorithm');
      expect(results.map(a => a.id)).toContain('test-algorithm-2');
    });

    it('should update algorithm', () => {
      manager.saveToFile(mockAlgorithm);
      
      const updates = {
        name: 'Updated Algorithm',
        metadata: {
          status: 'beta' as const,
        },
      } as Partial<AlgorithmDocumentation>;
      
      const result = manager.update('test-algorithm', updates);
      
      expect(result).toBe(true);
      
      const updated = manager.getById('test-algorithm');
      expect(updated?.name).toBe('Updated Algorithm');
      expect(updated?.metadata.status).toBe('beta');
      expect(updated?.metadata.updatedAt).not.toBe(mockAlgorithm.metadata.updatedAt);
    });

    it('should return false for update of non-existent algorithm', () => {
      const result = manager.update('non-existent', { name: 'Updated' });
      
      expect(result).toBe(false);
    });

    it('should delete algorithm', () => {
      manager.saveToFile(mockAlgorithm);
      (existsSync as jest.Mock).mockReturnValue(true);
      
      const result = manager.delete('test-algorithm');
      
      expect(result).toBe(true);
      expect(unlinkSync).toHaveBeenCalled();
    });

    it('should return false for deletion of non-existent algorithm', () => {
      (existsSync as jest.Mock).mockReturnValue(false);
      
      const result = manager.delete('non-existent');
      
      expect(result).toBe(false);
    });
  });

  describe('filtering methods', () => {
    beforeEach(() => {
      const algorithm1 = { ...mockAlgorithm, metadata: { ...mockAlgorithm.metadata, status: 'stable' } };
      const algorithm2 = { ...mockAlgorithm, id: 'test-2', metadata: { ...mockAlgorithm.metadata, status: 'beta' } };
      const algorithm3 = { ...mockAlgorithm, id: 'test-3', implementation: { language: 'Python' } };
      
      manager.saveToFile(algorithm1);
      manager.saveToFile(algorithm2);
      manager.saveToFile(algorithm3);
    });

    it('should filter by status', () => {
      const results = manager.getByStatus('stable');
      
      expect(results).toHaveLength(1);
      expect(results[0].metadata.status).toBe('stable');
    });

    it('should filter by language', () => {
      const results = manager.getByLanguage('Python');
      
      expect(results).toHaveLength(1);
      expect(results[0].implementation?.language).toBe('Python');
    });

    it('should filter by complexity', () => {
      const algorithm = { ...mockAlgorithm, performance: { complexity: 'O(n)' } } as AlgorithmDocumentation;
      manager.saveToFile(algorithm);
      
      const results = manager.getByComplexity('O(n)');
      
      expect(results).toHaveLength(1);
      expect(results[0].performance?.complexity).toBe('O(n)');
    });
  });

  describe('statistics', () => {
    it('should generate statistics correctly', () => {
      const algorithm1 = { ...mockAlgorithm, metadata: { ...mockAlgorithm.metadata, status: 'stable' } };
      const algorithm2 = { ...mockAlgorithm, id: 'test-2', metadata: { ...mockAlgorithm.metadata, status: 'beta' } };
      const algorithm3 = { ...mockAlgorithm, id: 'test-3', implementation: { language: 'Python' } };
      
      manager.saveToFile(algorithm1);
      manager.saveToFile(algorithm2);
      manager.saveToFile(algorithm3);
      
      const stats = manager.getStatistics();
      
      expect(stats.total).toBe(3);
      expect(stats.statusCounts).toEqual({ stable: 1, beta: 1 });
      expect(stats.languageCounts).toEqual({ Python: 1 });
    });
  });

  describe('import/export', () => {
    it('should export algorithms to file', () => {
      manager.saveToFile(mockAlgorithm);
      
      const result = manager.exportToFile('./test-export.json');
      
      expect(result).toBe(true);
      expect(writeFileSync).toHaveBeenCalledWith(
        './test-export.json',
        expect.stringContaining('test-algorithm'),
        'utf-8'
      );
    });

    it('should import algorithms from file', () => {
      const exportData = {
        algorithms: [mockAlgorithm],
        exportDate: '2024-01-01T00:00:00.000Z',
        total: 1,
      };
      
      (existsSync as jest.Mock).mockReturnValue(true);
      (readFileSync as jest.Mock).mockReturnValue(JSON.stringify(exportData));
      
      const result = manager.importFromFile('./test-import.json');
      
      expect(result).toBe(true);
    });

    it('should handle import errors gracefully', () => {
      (existsSync as jest.Mock).mockReturnValue(false);
      
      const result = manager.importFromFile('./non-existent.json');
      
      expect(result).toBe(false);
    });
  });
}); 