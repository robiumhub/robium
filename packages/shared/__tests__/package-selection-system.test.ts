import { PackageSelectionSystem } from '../utils/package-selection-system';
import { MetadataStorageSystem } from '../utils/metadata-storage-system';
import { RosPackage, PackageSelectionCriteria, CompatibilityCriteria } from '../types/package-selection';
import { AlgorithmDocumentation } from '../types/algorithm-documentation';
import { readFileSync, writeFileSync, existsSync, mkdirSync, readdirSync, copyFileSync, statSync } from 'fs';
import { join } from 'path';

// Mock file system operations for testing
jest.mock('fs', () => ({
  readFileSync: jest.fn(),
  writeFileSync: jest.fn(),
  existsSync: jest.fn(),
  mkdirSync: jest.fn(),
  readdirSync: jest.fn(),
  copyFileSync: jest.fn(),
  statSync: jest.fn(),
}));

describe('PackageSelectionSystem', () => {
  let packageSelection: PackageSelectionSystem;
  let metadataStorage: MetadataStorageSystem;
  let mockRegistryPath: string;

  const mockAlgorithm: AlgorithmDocumentation = {
    id: 'test-algorithm',
    name: 'Test Algorithm',
    version: '1.0.0',
    taskDefinition: {
      title: 'Test Task',
      description: 'Test algorithm for unit testing',
      problemStatement: 'Test problem statement',
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
          version: '1.0.0',
          purpose: 'Core functionality',
          optional: false,
        },
        {
          name: 'optional_package',
          version: '2.0.0',
          purpose: 'Optional features',
          optional: true,
        },
      ],
    },
    metadata: {
      createdAt: '2024-01-01T00:00:00Z',
      updatedAt: '2024-01-01T00:00:00Z',
      status: 'stable' as const,
    },
  };

  const mockPackage: RosPackage = {
    name: 'test_package',
    version: '1.0.0',
    description: 'Test ROS package',
    maintainer: 'test@example.com',
    license: 'MIT',
    dependencies: [
      {
        name: 'dependency_package',
        version: '1.0.0',
        type: 'ros',
      },
    ],
    conflicts: [],
    provides: [],
    category: 'navigation',
    tags: ['test', 'navigation'],
    size: 1024000, // 1MB
    buildType: 'ament_cmake',
    rosVersion: 'humble',
    platform: ['linux', 'ubuntu'],
  };

  beforeEach(() => {
    mockRegistryPath = './test-registry';
    
    // Mock metadata storage
    metadataStorage = {
      getById: jest.fn().mockReturnValue(mockAlgorithm),
    } as any;

    // Mock file system operations
    (existsSync as jest.Mock).mockReturnValue(true);
    (readdirSync as jest.Mock).mockReturnValue(['test_package.json']);
    (readFileSync as jest.Mock).mockReturnValue(JSON.stringify(mockPackage));
    (statSync as jest.Mock).mockReturnValue({ isDirectory: () => true, size: 1024 });

    packageSelection = new PackageSelectionSystem(metadataStorage, {
      packageRegistryPath: mockRegistryPath,
    });
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  describe('constructor', () => {
    it('should initialize with default configuration', () => {
      expect(packageSelection).toBeDefined();
    });

    it('should load package registry on initialization', () => {
      expect(readdirSync).toHaveBeenCalledWith(mockRegistryPath);
      expect(readFileSync).toHaveBeenCalledWith(join(mockRegistryPath, 'test_package.json'), 'utf-8');
    });
  });

  describe('selectPackages', () => {
    const mockCriteria: PackageSelectionCriteria = {
      algorithms: ['test-algorithm'],
      targetRosVersion: 'humble',
      targetPlatform: 'ubuntu',
      optimizationLevel: 'balanced',
    };

    it('should select packages based on algorithm requirements', () => {
      const result = packageSelection.selectPackages(mockCriteria);

      expect(result.selectedPackages).toBeDefined();
      expect(result.selectedPackages.length).toBeGreaterThan(0);
      expect(result.statistics).toBeDefined();
      expect(result.optimizationScore).toBeGreaterThan(0);
    });

    it('should handle missing algorithms gracefully', () => {
      (metadataStorage.getById as jest.Mock).mockReturnValue(null);

      const result = packageSelection.selectPackages(mockCriteria);

      expect(result.warnings).toContain('Algorithm test-algorithm not found');
      expect(result.selectedPackages.length).toBe(0);
    });

    it('should resolve dependencies correctly', () => {
      const result = packageSelection.selectPackages(mockCriteria);

      expect(result.resolvedDependencies).toBeDefined();
      expect(result.resolvedDependencies.size).toBeGreaterThan(0);
    });

    it('should detect and report conflicts', () => {
      // Mock a package with conflicts
      const conflictingPackage: RosPackage = {
        ...mockPackage,
        conflicts: ['conflicting_package'],
      };
      (readFileSync as jest.Mock).mockReturnValue(JSON.stringify(conflictingPackage));

      const result = packageSelection.selectPackages(mockCriteria);

      expect(result.conflicts.length).toBeGreaterThan(0);
    });

    it('should apply optimization strategy', () => {
      const result = packageSelection.selectPackages({
        ...mockCriteria,
        optimizationLevel: 'minimal',
      });

      expect(result.selectedPackages).toBeDefined();
      expect(result.optimizationScore).toBeGreaterThan(0);
    });
  });

  describe('copyPackages', () => {
    const mockCopyConfig = {
      sourcePath: './source',
      targetPath: './target',
      includeTests: false,
      includeDocs: true,
      includeExamples: false,
      preserveStructure: true,
      overwrite: false,
      validateAfterCopy: true,
      createBackup: false,
    };

    it('should copy packages successfully', () => {
      const result = packageSelection.copyPackages([mockPackage], mockCopyConfig);

      expect(result.success).toBe(true);
      expect(result.copiedPackages).toContain('test_package');
      expect(result.copyTime).toBeGreaterThan(0);
    });

    it('should handle missing source packages', () => {
      (existsSync as jest.Mock).mockReturnValueOnce(false); // Source package doesn't exist

      const result = packageSelection.copyPackages([mockPackage], mockCopyConfig);

      expect(result.success).toBe(false);
      expect(result.failedPackages).toContain('test_package');
      expect(result.errors).toContain('Source package not found: test_package');
    });

    it('should create backup when requested', () => {
      const configWithBackup = { ...mockCopyConfig, createBackup: true };
      (existsSync as jest.Mock).mockReturnValue(true); // Target exists

      packageSelection.copyPackages([mockPackage], configWithBackup);

      expect(mkdirSync).toHaveBeenCalled();
    });

    it('should validate copied packages when requested', () => {
      const result = packageSelection.copyPackages([mockPackage], mockCopyConfig);

      expect(result.warnings).toBeDefined();
      expect(result.errors).toBeDefined();
    });

    it('should calculate total size correctly', () => {
      const result = packageSelection.copyPackages([mockPackage], mockCopyConfig);

      expect(result.totalSize).toBeGreaterThan(0);
    });
  });

  describe('validateCompatibility', () => {
    const mockCompatibilityCriteria: CompatibilityCriteria = {
      rosVersion: 'humble',
      platform: 'ubuntu',
      architecture: 'x86_64',
      pythonVersion: '3.8',
      systemLibraries: ['libopencv-dev', 'libpcl-dev'],
      hardwareRequirements: ['camera', 'lidar'],
    };

    it('should validate package compatibility', () => {
      const result = packageSelection.validateCompatibility([mockPackage], mockCompatibilityCriteria);

      expect(result.compatible).toBeDefined();
      expect(result.score).toBeGreaterThan(0);
      expect(result.issues).toBeDefined();
      expect(result.recommendations).toBeDefined();
    });

    it('should detect version incompatibilities', () => {
      const incompatiblePackage: RosPackage = {
        ...mockPackage,
        rosVersion: 'foxy', // Different ROS version
      };

      const result = packageSelection.validateCompatibility([incompatiblePackage], mockCompatibilityCriteria);

      expect(result.compatible).toBe(false);
      expect(result.issues.some(issue => issue.issueType === 'version')).toBe(true);
      expect(result.score).toBeLessThan(100);
    });

    it('should detect platform incompatibilities', () => {
      const platformSpecificPackage: RosPackage = {
        ...mockPackage,
        platform: ['windows'], // Different platform
      };

      const result = packageSelection.validateCompatibility([platformSpecificPackage], mockCompatibilityCriteria);

      expect(result.issues.some(issue => issue.issueType === 'platform')).toBe(true);
      expect(result.score).toBeLessThan(100);
    });

    it('should provide resolution recommendations', () => {
      const incompatiblePackage: RosPackage = {
        ...mockPackage,
        rosVersion: 'foxy',
      };

      const result = packageSelection.validateCompatibility([incompatiblePackage], mockCompatibilityCriteria);

      expect(result.recommendations.length).toBeGreaterThan(0);
      expect(result.recommendations.some(rec => rec.includes('Upgrade to ROS'))).toBe(true);
    });

    it('should cache compatibility results', () => {
      const result1 = packageSelection.validateCompatibility([mockPackage], mockCompatibilityCriteria);
      const result2 = packageSelection.validateCompatibility([mockPackage], mockCompatibilityCriteria);

      expect(result1).toEqual(result2);
    });
  });

  describe('optimization strategies', () => {
    const mockCriteria: PackageSelectionCriteria = {
      algorithms: ['test-algorithm'],
      optimizationLevel: 'balanced',
    };

    it('should apply minimal optimization', () => {
      const result = packageSelection.selectPackages({
        ...mockCriteria,
        optimizationLevel: 'minimal',
      });

      expect(result.selectedPackages).toBeDefined();
      expect(result.optimizationScore).toBeGreaterThan(0);
    });

    it('should apply balanced optimization', () => {
      const result = packageSelection.selectPackages({
        ...mockCriteria,
        optimizationLevel: 'balanced',
      });

      expect(result.selectedPackages).toBeDefined();
      expect(result.optimizationScore).toBeGreaterThan(0);
    });

    it('should apply complete optimization', () => {
      const result = packageSelection.selectPackages({
        ...mockCriteria,
        optimizationLevel: 'complete',
      });

      expect(result.selectedPackages).toBeDefined();
      expect(result.optimizationScore).toBeGreaterThan(0);
    });
  });

  describe('statistics and monitoring', () => {
    it('should provide system statistics', () => {
      const stats = packageSelection.getStatistics();

      expect(stats.totalPackages).toBeDefined();
      expect(stats.cacheSize).toBeDefined();
      expect(stats.config).toBeDefined();
    });

    it('should clear caches', () => {
      packageSelection.clearCaches();
      // No assertions needed - just ensure no errors are thrown
    });

    it('should reload registry', () => {
      packageSelection.reloadRegistry();
      expect(readdirSync).toHaveBeenCalledWith(mockRegistryPath);
    });
  });

  describe('error handling', () => {
    it('should handle file system errors gracefully', () => {
      (readdirSync as jest.Mock).mockImplementation(() => {
        throw new Error('File system error');
      });

      // Should not throw error during construction
      expect(() => {
        new PackageSelectionSystem(metadataStorage, { packageRegistryPath: mockRegistryPath });
      }).not.toThrow();
    });

    it('should handle invalid package data gracefully', () => {
      (readFileSync as jest.Mock).mockReturnValue('invalid json');

      // Should not throw error during construction
      expect(() => {
        new PackageSelectionSystem(metadataStorage, { packageRegistryPath: mockRegistryPath });
      }).not.toThrow();
    });
  });

  describe('dependency resolution', () => {
    it('should handle circular dependencies', () => {
      const circularPackage: RosPackage = {
        ...mockPackage,
        dependencies: [
          {
            name: 'circular_dependency',
            type: 'ros',
          },
        ],
      };
      (readFileSync as jest.Mock).mockReturnValue(JSON.stringify(circularPackage));

      const result = packageSelection.selectPackages({
        algorithms: ['test-algorithm'],
        optimizationLevel: 'minimal',
      });

      expect(result.conflicts.length).toBeGreaterThan(0);
    });

    it('should handle missing dependencies', () => {
      const packageWithMissingDep: RosPackage = {
        ...mockPackage,
        dependencies: [
          {
            name: 'missing_package',
            type: 'ros',
          },
        ],
      };
      (readFileSync as jest.Mock).mockReturnValue(JSON.stringify(packageWithMissingDep));

      const result = packageSelection.selectPackages({
        algorithms: ['test-algorithm'],
        optimizationLevel: 'minimal',
      });

      expect(result.conflicts.length).toBeGreaterThan(0);
    });
  });
}); 