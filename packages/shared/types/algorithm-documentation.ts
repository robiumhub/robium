// Algorithm Documentation Types
// Comprehensive type definitions for ROS algorithm documentation

export interface TaskDefinition {
  title: string;
  description: string;
  problemStatement: string;
  useCases?: string[];
  assumptions?: string[];
  limitations?: string[];
}

export interface InputOutputSpec {
  name: string;
  type: string;
  description: string;
  required: boolean;
  default?: unknown;
  constraints?: Record<string, unknown>;
}

export interface OutputSpec {
  name: string;
  type: string;
  description: string;
  format?: string;
}

export interface InputOutputSpecification {
  inputs: InputOutputSpec[];
  outputs: OutputSpec[];
  sideEffects?: string[];
}

export interface ConfigurableParameter {
  name: string;
  type: 'int' | 'float' | 'double' | 'bool' | 'string' | 'array' | 'object';
  description: string;
  default?: unknown;
  min?: number;
  max?: number;
  options?: unknown[];
  unit?: string;
  category?: string;
}

export interface AdvancedParameter {
  name: string;
  type: string;
  description: string;
  default?: unknown;
  warning?: string;
}

export interface AlgorithmParameters {
  configurable: ConfigurableParameter[];
  advanced?: AdvancedParameter[];
}

export interface RosPackageDependency {
  name: string;
  version?: string;
  purpose: string;
  optional?: boolean;
}

export interface PythonPackageDependency {
  name: string;
  version?: string;
}

export interface AlgorithmDependencies {
  rosPackages: RosPackageDependency[];
  systemLibraries?: string[];
  pythonPackages?: PythonPackageDependency[];
  algorithms?: string[];
}

export interface ExecutionTime {
  typical?: string;
  worstCase?: string;
  bestCase?: string;
}

export interface MemoryUsage {
  typical?: string;
  peak?: string;
}

export interface PerformanceMetrics {
  complexity?: 'O(1)' | 'O(log n)' | 'O(n)' | 'O(n log n)' | 'O(n²)' | 'O(n³)' | 'O(2ⁿ)' | 'O(n!)';
  spaceComplexity?: string;
  executionTime?: ExecutionTime;
  memoryUsage?: MemoryUsage;
  throughput?: string;
}

export interface ImplementationDetails {
  language: 'C++' | 'Python' | 'C' | 'Java';
  architecture?: string;
  threading?: 'single-threaded' | 'multi-threaded' | 'asynchronous';
  realTime?: boolean;
  optimizations?: string[];
}

export interface Benchmark {
  name: string;
  description: string;
  results: Record<string, unknown>;
}

export interface TestingInfo {
  testCoverage?: number;
  testTypes?: ('unit' | 'integration' | 'performance' | 'regression' | 'stress')[];
  testData?: string[];
  benchmarks?: Benchmark[];
}

export interface Tutorial {
  title: string;
  url: string;
  difficulty: 'beginner' | 'intermediate' | 'advanced';
}

export interface Example {
  name: string;
  description: string;
  codeUrl?: string;
  demoUrl?: string;
}

export interface ResearchPaper {
  title: string;
  authors: string[];
  url?: string;
  year?: number;
}

export interface DocumentationResources {
  apiReference?: string;
  tutorials?: Tutorial[];
  examples?: Example[];
  papers?: ResearchPaper[];
}

export interface AlgorithmMetadata {
  createdAt: string;
  updatedAt: string;
  version?: string;
  maintainer?: string;
  license?: string;
  status: 'stable' | 'beta' | 'experimental' | 'deprecated';
  tags?: string[];
  categories?: string[];
}

export interface AlgorithmDocumentation {
  id: string;
  name: string;
  version: string;
  taskDefinition: TaskDefinition;
  inputOutputSpecification: InputOutputSpecification;
  parameters: AlgorithmParameters;
  dependencies: AlgorithmDependencies;
  performance?: PerformanceMetrics;
  implementation?: ImplementationDetails;
  testing?: TestingInfo;
  documentation?: DocumentationResources;
  metadata: AlgorithmMetadata;
}

// Type aliases for common patterns
export type AlgorithmId = string;
export type AlgorithmStatus = AlgorithmMetadata['status'];
export type AlgorithmLanguage = ImplementationDetails['language'];
export type ParameterType = ConfigurableParameter['type'];
export type ComplexityLevel = PerformanceMetrics['complexity'];

// Validation types
export interface ValidationResult {
  isValid: boolean;
  errors: string[];
  warnings: string[];
}

// Search and filtering types
export interface AlgorithmSearchCriteria {
  name?: string;
  tags?: string[];
  categories?: string[];
  status?: AlgorithmStatus[];
  language?: AlgorithmLanguage[];
  complexity?: ComplexityLevel[];
  metaCategory?: string;
  taskCategory?: string;
}

export interface AlgorithmSearchResult {
  algorithm: AlgorithmDocumentation;
  relevanceScore: number;
  matchedCriteria: string[];
}

// Template types for creating new algorithm documentation
export interface AlgorithmDocumentationTemplate {
  taskDefinition: Partial<TaskDefinition>;
  inputOutputSpecification: Partial<InputOutputSpecification>;
  parameters: Partial<AlgorithmParameters>;
  dependencies: Partial<AlgorithmDependencies>;
  performance?: Partial<PerformanceMetrics>;
  implementation?: Partial<ImplementationDetails>;
  testing?: Partial<TestingInfo>;
  documentation?: Partial<DocumentationResources>;
  metadata: Partial<AlgorithmMetadata>;
}

// All types are exported above as interfaces 