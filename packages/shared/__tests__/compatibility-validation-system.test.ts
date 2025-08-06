import { CompatibilityValidationSystem } from '../utils/compatibility-validation-system';
import { MetadataStorageSystem } from '../utils/metadata-storage-system';
import { AlgorithmDocumentation } from '../types/algorithm-documentation';
import {
  EnvironmentSpec,
  CompatibilityCriteria,
  CompatibilityTestConfig,
  TestScenario,
} from '../types/compatibility-validation';

describe('CompatibilityValidationSystem', () => {
  let validationSystem: CompatibilityValidationSystem;
  let metadataStorage: MetadataStorageSystem;

  const mockAlgorithm: AlgorithmDocumentation = {
    id: 'test-algorithm-1',
    name: 'Test Algorithm',
    version: '1.0.0',
    taskDefinition: {
      title: 'Test Algorithm',
      description: 'A test algorithm for validation',
      problemStatement: 'Test problem statement',
    },
    inputOutputSpecification: {
      inputs: [
        {
          name: 'input_data',
          type: 'std_msgs/String',
          description: 'Input data',
          required: true,
        },
      ],
      outputs: [
        {
          name: 'output_data',
          type: 'std_msgs/String',
          description: 'Output data',
        },
      ],
    },
    parameters: {
      configurable: [
        {
          name: 'test_param',
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
          purpose: 'Test package',
          optional: false,
        },
      ],
    },
    performance: {
      complexity: 'O(n)',
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
      tags: ['test', 'validation'],
      categories: ['test'],
    },
  };

  const mockEnvironment: EnvironmentSpec = {
    rosVersion: '2.0.0',
    platform: {
      os: 'ubuntu',
      architecture: 'x86_64',
      distribution: '20.04',
      version: '20.04',
      kernel: '5.4.0',
      containerized: false,
    },
    hardware: {
      cpu: {
        architecture: 'x86_64',
        cores: 4,
        frequency: 2.4,
        instructionSet: ['SSE4.2', 'AVX2'],
        features: ['hyperthreading'],
      },
      memory: {
        total: 8192,
        available: 6144,
        type: 'DDR4',
        speed: 3200,
      },
      storage: {
        total: 512,
        available: 256,
        type: 'ssd',
        readSpeed: 500,
        writeSpeed: 400,
      },
      gpu: {
        model: 'NVIDIA GTX 1060',
        memory: 6144,
        computeCapability: '6.1',
        driverVersion: '470.82.01',
        cudaSupport: true,
        openclSupport: true,
      },
      sensors: [
        {
          type: 'camera',
          model: 'USB Camera',
          resolution: '1920x1080',
          frequency: 30,
          range: '0.1-10m',
          accuracy: '±1cm',
        },
      ],
      actuators: [
        {
          type: 'motor',
          model: 'DC Motor',
          torque: '10Nm',
          speed: '1000rpm',
          precision: '±0.1°',
        },
      ],
      peripherals: [
        {
          type: 'lidar',
          interface: 'USB3.0',
          model: 'Hokuyo UTM-30LX',
          capabilities: ['2D scanning', '360° coverage'],
        },
      ],
    },
    software: {
      pythonVersion: '3.8.10',
      cppVersion: '17',
      libraries: [
        {
          name: 'libopencv',
          version: '4.5.0',
          type: 'system',
          source: 'apt',
          license: 'BSD',
        },
        {
          name: 'numpy',
          version: '1.21.0',
          type: 'python',
          source: 'pip',
          license: 'BSD',
        },
      ],
      frameworks: [
        {
          name: 'OpenCV',
          version: '4.5.0',
          type: 'cv',
          capabilities: ['image_processing', 'computer_vision'],
        },
      ],
      dependencies: [
        {
          name: 'cmake',
          version: '3.16.0',
          type: 'build',
          purpose: 'Build system',
        },
      ],
    },
    network: {
      bandwidth: 1000,
      latency: 5,
      protocol: ['TCP', 'UDP'],
      security: ['TLS', 'SSH'],
      topology: 'star',
    },
    security: {
      encryption: ['AES-256', 'RSA-2048'],
      authentication: ['password', 'key-based'],
      authorization: ['role-based', 'attribute-based'],
      compliance: ['GDPR', 'ISO27001'],
    },
  };

  beforeEach(() => {
    // Mock metadata storage
    metadataStorage = {
      getById: jest.fn().mockReturnValue(mockAlgorithm),
      getAll: jest.fn().mockReturnValue([mockAlgorithm]),
    } as any;

    validationSystem = new CompatibilityValidationSystem(metadataStorage, {
      cacheEnabled: true,
      cacheSize: 100,
      strictMode: false,
      includeWarnings: true,
      validateDependencies: true,
      validatePerformance: true,
      validateSecurity: true,
    });
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  describe('constructor', () => {
    it('should initialize with default configuration', () => {
      expect(validationSystem).toBeDefined();
    });

    it('should initialize with custom configuration', () => {
      const customSystem = new CompatibilityValidationSystem(metadataStorage, {
        strictMode: true,
        cacheEnabled: false,
        timeout: 60000,
      });

      expect(customSystem).toBeDefined();
    });
  });

  describe('validateCompatibility', () => {
    const mockCriteria: CompatibilityCriteria = {
      algorithmId: 'test-algorithm-1',
      environment: mockEnvironment,
      strictMode: false,
      includeWarnings: true,
      validateDependencies: true,
      validatePerformance: true,
      validateSecurity: true,
    };

    it('should validate algorithm compatibility successfully', () => {
      const result = validationSystem.validateCompatibility(mockCriteria);

      expect(result).toBeDefined();
      expect(result.compatible).toBeDefined();
      expect(result.score).toBeGreaterThanOrEqual(0);
      expect(result.score).toBeLessThanOrEqual(100);
      expect(result.issues).toBeDefined();
      expect(result.warnings).toBeDefined();
      expect(result.recommendations).toBeDefined();
      expect(result.details).toBeDefined();
      expect(result.metadata).toBeDefined();
    });

    it('should return cached result for same criteria', () => {
      const result1 = validationSystem.validateCompatibility(mockCriteria);
      const result2 = validationSystem.validateCompatibility(mockCriteria);

      expect(result1).toEqual(result2);
    });

    it('should handle algorithm not found', () => {
      (metadataStorage.getById as jest.Mock).mockReturnValue(null);

      const result = validationSystem.validateCompatibility(mockCriteria);

      expect(result.compatible).toBe(false);
      expect(result.score).toBe(0);
      expect(result.issues.length).toBeGreaterThan(0);
      expect(result.issues[0].type).toBe('critical');
    });

    it('should validate with strict mode', () => {
      const strictCriteria: CompatibilityCriteria = {
        ...mockCriteria,
        strictMode: true,
      };

      const result = validationSystem.validateCompatibility(strictCriteria);

      expect(result).toBeDefined();
      // Strict mode should be more restrictive
      expect(result.compatible).toBeDefined();
    });

    it('should include validation details', () => {
      const result = validationSystem.validateCompatibility(mockCriteria);

      expect(result.details.environment).toBeDefined();
      expect(result.details.dependencies).toBeDefined();
      expect(result.details.performance).toBeDefined();
      expect(result.details.security).toBeDefined();
      expect(result.details.hardware).toBeDefined();
      expect(result.details.software).toBeDefined();
    });

    it('should provide validation metadata', () => {
      const result = validationSystem.validateCompatibility(mockCriteria);

      expect(result.metadata.timestamp).toBeDefined();
      expect(result.metadata.validator).toBe('CompatibilityValidationSystem');
      expect(result.metadata.version).toBe('1.0.0');
      expect(result.metadata.duration).toBeGreaterThan(0);
      expect(result.metadata.checks).toBeGreaterThan(0);
      expect(result.metadata.passed).toBeGreaterThanOrEqual(0);
      expect(result.metadata.failed).toBeGreaterThanOrEqual(0);
      expect(result.metadata.warnings).toBeGreaterThanOrEqual(0);
    });
  });

  describe('runCompatibilityTests', () => {
    const mockTestConfig: CompatibilityTestConfig = {
      testEnvironment: mockEnvironment,
      testAlgorithms: ['test-algorithm-1'],
      testScenarios: [
        {
          name: 'basic_functionality',
          description: 'Test basic algorithm functionality',
          setup: ['Initialize environment'],
          execution: ['Run algorithm'],
          cleanup: ['Clean up resources'],
          expectedResults: { success: true },
        },
      ],
      timeout: 30,
      retries: 3,
      parallel: false,
      reporting: {
        format: 'json',
        includeDetails: true,
        includeWarnings: true,
        includeRecommendations: true,
      },
    };

    it('should run compatibility tests successfully', async () => {
      const result = await validationSystem.runCompatibilityTests(mockTestConfig);

      expect(result).toBeDefined();
      expect(result.testId).toBeDefined();
      expect(result.timestamp).toBeDefined();
      expect(result.environment).toEqual(mockEnvironment);
      expect(result.results).toBeDefined();
      expect(result.results.length).toBe(1);
      expect(result.summary).toBeDefined();
      expect(result.metadata).toBeDefined();
    });

    it('should handle multiple algorithms', async () => {
      const multiAlgorithmConfig: CompatibilityTestConfig = {
        ...mockTestConfig,
        testAlgorithms: ['test-algorithm-1', 'test-algorithm-2'],
      };

      // Mock second algorithm
      (metadataStorage.getById as jest.Mock).mockImplementation((id: string) => {
        if (id === 'test-algorithm-1') return mockAlgorithm;
        if (id === 'test-algorithm-2') return { ...mockAlgorithm, id: 'test-algorithm-2' };
        return null;
      });

      const result = await validationSystem.runCompatibilityTests(multiAlgorithmConfig);

      expect(result.results.length).toBe(2);
      expect(result.summary.totalAlgorithms).toBe(2);
    });

    it('should handle algorithm not found in tests', async () => {
      const configWithMissingAlgorithm: CompatibilityTestConfig = {
        ...mockTestConfig,
        testAlgorithms: ['nonexistent-algorithm'],
      };

      (metadataStorage.getById as jest.Mock).mockReturnValue(null);

      const result = await validationSystem.runCompatibilityTests(configWithMissingAlgorithm);

      expect(result.results.length).toBe(1);
      expect(result.results[0].compatible).toBe(false);
      expect(result.results[0].score).toBe(0);
      expect(result.results[0].issues.length).toBeGreaterThan(0);
    });

    it('should provide test summary', async () => {
      const result = await validationSystem.runCompatibilityTests(mockTestConfig);

      expect(result.summary.totalTests).toBeGreaterThan(0);
      expect(result.summary.passedTests).toBeGreaterThanOrEqual(0);
      expect(result.summary.failedTests).toBeGreaterThanOrEqual(0);
      expect(result.summary.totalAlgorithms).toBe(1);
      expect(result.summary.compatibleAlgorithms).toBeGreaterThanOrEqual(0);
      expect(result.summary.averageScore).toBeGreaterThanOrEqual(0);
      expect(result.summary.criticalIssues).toBeGreaterThanOrEqual(0);
      expect(result.summary.warnings).toBeGreaterThanOrEqual(0);
    });

    it('should provide test metadata', async () => {
      const result = await validationSystem.runCompatibilityTests(mockTestConfig);

      expect(result.metadata.testDuration).toBeGreaterThan(0);
      expect(result.metadata.parallelExecution).toBe(false);
      expect(result.metadata.retries).toBe(3);
      expect(result.metadata.timeout).toBe(30);
      expect(result.metadata.validator).toBe('CompatibilityValidationSystem');
      expect(result.metadata.version).toBe('1.0.0');
    });
  });

  describe('environment validation', () => {
    it('should validate ROS version compatibility', () => {
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      const result = validationSystem.validateCompatibility(criteria);

      expect(result.details.environment.rosVersion).toBeDefined();
      expect(result.details.environment.rosVersion.passed).toBeDefined();
      expect(result.details.environment.rosVersion.score).toBeGreaterThanOrEqual(0);
      expect(result.details.environment.rosVersion.message).toBeDefined();
    });

    it('should validate platform compatibility', () => {
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      const result = validationSystem.validateCompatibility(criteria);

      expect(result.details.environment.platform).toBeDefined();
      expect(result.details.environment.platform.passed).toBeDefined();
      expect(result.details.environment.platform.score).toBeGreaterThanOrEqual(0);
      expect(result.details.environment.platform.message).toBeDefined();
    });

    it('should validate architecture compatibility', () => {
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      const result = validationSystem.validateCompatibility(criteria);

      expect(result.details.environment.architecture).toBeDefined();
      expect(result.details.environment.architecture.passed).toBeDefined();
      expect(result.details.environment.architecture.score).toBeGreaterThanOrEqual(0);
      expect(result.details.environment.architecture.message).toBeDefined();
    });
  });

  describe('dependency validation', () => {
    it('should validate ROS packages', () => {
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      const result = validationSystem.validateCompatibility(criteria);

      expect(result.details.dependencies.rosPackages).toBeDefined();
      expect(Array.isArray(result.details.dependencies.rosPackages)).toBe(true);
    });

    it('should validate system libraries', () => {
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      const result = validationSystem.validateCompatibility(criteria);

      expect(result.details.dependencies.systemLibraries).toBeDefined();
      expect(Array.isArray(result.details.dependencies.systemLibraries)).toBe(true);
    });

    it('should validate Python packages', () => {
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      const result = validationSystem.validateCompatibility(criteria);

      expect(result.details.dependencies.pythonPackages).toBeDefined();
      expect(Array.isArray(result.details.dependencies.pythonPackages)).toBe(true);
    });

    it('should identify missing dependencies', () => {
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      const result = validationSystem.validateCompatibility(criteria);

      expect(result.details.dependencies.missing).toBeDefined();
      expect(Array.isArray(result.details.dependencies.missing)).toBe(true);
    });
  });

  describe('performance validation', () => {
    it('should validate CPU requirements', () => {
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      const result = validationSystem.validateCompatibility(criteria);

      expect(result.details.performance.cpuRequirements).toBeDefined();
      expect(result.details.performance.cpuRequirements.required).toBeGreaterThan(0);
      expect(result.details.performance.cpuRequirements.available).toBeGreaterThan(0);
      expect(result.details.performance.cpuRequirements.unit).toBe('cores');
      expect(typeof result.details.performance.cpuRequirements.satisfied).toBe('boolean');
      expect(result.details.performance.cpuRequirements.margin).toBeGreaterThanOrEqual(0);
    });

    it('should validate memory requirements', () => {
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      const result = validationSystem.validateCompatibility(criteria);

      expect(result.details.performance.memoryRequirements).toBeDefined();
      expect(result.details.performance.memoryRequirements.required).toBeGreaterThan(0);
      expect(result.details.performance.memoryRequirements.available).toBeGreaterThan(0);
      expect(result.details.performance.memoryRequirements.unit).toBe('MB');
      expect(typeof result.details.performance.memoryRequirements.satisfied).toBe('boolean');
      expect(result.details.performance.memoryRequirements.margin).toBeGreaterThanOrEqual(0);
    });

    it('should validate storage requirements', () => {
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      const result = validationSystem.validateCompatibility(criteria);

      expect(result.details.performance.storageRequirements).toBeDefined();
      expect(result.details.performance.storageRequirements.required).toBeGreaterThan(0);
      expect(result.details.performance.storageRequirements.available).toBeGreaterThan(0);
      expect(result.details.performance.storageRequirements.unit).toBe('MB');
      expect(typeof result.details.performance.storageRequirements.satisfied).toBe('boolean');
      expect(result.details.performance.storageRequirements.margin).toBeGreaterThanOrEqual(0);
    });

    it('should provide performance estimates', () => {
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      const result = validationSystem.validateCompatibility(criteria);

      expect(result.details.performance.estimatedPerformance).toBeDefined();
      expect(result.details.performance.estimatedPerformance.executionTime).toBeDefined();
      expect(result.details.performance.estimatedPerformance.memoryUsage).toBeDefined();
      expect(result.details.performance.estimatedPerformance.cpuUsage).toBeDefined();
      expect(result.details.performance.estimatedPerformance.throughput).toBeDefined();
      expect(result.details.performance.estimatedPerformance.scalability).toBeDefined();
    });
  });

  describe('security validation', () => {
    it('should validate security requirements', () => {
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      const result = validationSystem.validateCompatibility(criteria);

      expect(result.details.security).toBeDefined();
      expect(result.details.security.permissions).toBeDefined();
      expect(result.details.security.networkAccess).toBeDefined();
      expect(result.details.security.dataAccess).toBeDefined();
      expect(result.details.security.compliance).toBeDefined();
      expect(result.details.security.vulnerabilities).toBeDefined();
    });
  });

  describe('hardware validation', () => {
    it('should validate hardware requirements', () => {
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      const result = validationSystem.validateCompatibility(criteria);

      expect(result.details.hardware).toBeDefined();
      expect(result.details.hardware.sensors).toBeDefined();
      expect(result.details.hardware.actuators).toBeDefined();
      expect(result.details.hardware.peripherals).toBeDefined();
      expect(result.details.hardware.connectivity).toBeDefined();
    });
  });

  describe('software validation', () => {
    it('should validate software requirements', () => {
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      const result = validationSystem.validateCompatibility(criteria);

      expect(result.details.software).toBeDefined();
      expect(result.details.software.languageSupport).toBeDefined();
      expect(result.details.software.frameworkSupport).toBeDefined();
      expect(result.details.software.librarySupport).toBeDefined();
      expect(result.details.software.buildSupport).toBeDefined();
    });
  });

  describe('error handling', () => {
    it('should handle validation errors gracefully', () => {
      // Mock a validation error
      jest.spyOn(validationSystem as any, 'validateEnvironment').mockImplementation(() => {
        throw new Error('Validation error');
      });

      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      const result = validationSystem.validateCompatibility(criteria);

      expect(result.compatible).toBe(false);
      expect(result.score).toBe(0);
      expect(result.issues.length).toBeGreaterThan(0);
      expect(result.issues[0].type).toBe('critical');
    });

    it('should handle test execution errors', async () => {
      const testConfig: CompatibilityTestConfig = {
        testEnvironment: mockEnvironment,
        testAlgorithms: ['test-algorithm-1'],
        testScenarios: [
          {
            name: 'error_test',
            description: 'Test that throws error',
            setup: ['Initialize'],
            execution: ['Throw error'],
            cleanup: ['Cleanup'],
            expectedResults: { success: false },
          },
        ],
        timeout: 30,
        retries: 3,
        parallel: false,
        reporting: {
          format: 'json',
          includeDetails: true,
          includeWarnings: true,
          includeRecommendations: true,
        },
      };

      const result = await validationSystem.runCompatibilityTests(testConfig);

      expect(result).toBeDefined();
      expect(result.results.length).toBe(1);
    });
  });

  describe('getStatistics', () => {
    it('should provide system statistics', () => {
      const stats = validationSystem.getStatistics();

      expect(stats.cacheSize).toBeDefined();
      expect(stats.cacheHits).toBeDefined();
      expect(stats.cacheMisses).toBeDefined();
      expect(stats.cacheHitRate).toBeDefined();
      expect(stats.validationRules).toBeDefined();
      expect(stats.testScenarios).toBeDefined();
      expect(stats.config).toBeDefined();
    });
  });

  describe('clearCaches', () => {
    it('should clear caches', () => {
      // First, populate cache
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      validationSystem.validateCompatibility(criteria);

      // Clear caches
      expect(() => validationSystem.clearCaches()).not.toThrow();

      // Verify cache is cleared
      const stats = validationSystem.getStatistics();
      expect(stats.cacheSize).toBe(0);
      expect(stats.cacheHits).toBe(0);
      expect(stats.cacheMisses).toBe(0);
    });
  });

  describe('performance', () => {
    it('should complete validation within reasonable time', () => {
      const startTime = Date.now();
      
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      validationSystem.validateCompatibility(criteria);
      
      const endTime = Date.now();
      const duration = endTime - startTime;
      
      // Should complete within 1 second
      expect(duration).toBeLessThan(1000);
    });

    it('should handle multiple validations efficiently', () => {
      const startTime = Date.now();
      
      const criteria: CompatibilityCriteria = {
        algorithmId: 'test-algorithm-1',
        environment: mockEnvironment,
      };

      // Run multiple validations
      for (let i = 0; i < 10; i++) {
        validationSystem.validateCompatibility(criteria);
      }
      
      const endTime = Date.now();
      const duration = endTime - startTime;
      
      // Should complete within 2 seconds
      expect(duration).toBeLessThan(2000);
    });
  });
});

// Export test function for integration with test runner
export async function testCompatibilityValidationSystem() {
  console.log('Testing CompatibilityValidationSystem...');
  
  // This is a placeholder - in a real test framework, the describe blocks would run automatically
  // For now, we'll just verify the system can be instantiated
  const metadataStorage = new MetadataStorageSystem();
  const validationSystem = new CompatibilityValidationSystem(metadataStorage);
  
  console.log('✓ CompatibilityValidationSystem instantiated successfully');
  console.log('✓ System statistics:', validationSystem.getStatistics());
  
  return true;
}