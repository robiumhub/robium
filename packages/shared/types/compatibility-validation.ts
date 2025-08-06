// Compatibility Validation Types
// Comprehensive type definitions for algorithm compatibility validation

import { AlgorithmDocumentation, AlgorithmId } from './algorithm-documentation';

/**
 * Environment specification for compatibility validation
 */
export interface EnvironmentSpec {
  rosVersion: string;
  platform: PlatformSpec;
  hardware: HardwareSpec;
  software: SoftwareSpec;
  network?: NetworkSpec;
  security?: SecuritySpec;
}

/**
 * Platform specification
 */
export interface PlatformSpec {
  os: string;
  architecture: string;
  distribution?: string;
  version?: string;
  kernel?: string;
  containerized?: boolean;
  virtualization?: string;
}

/**
 * Hardware specification
 */
export interface HardwareSpec {
  cpu: CpuSpec;
  memory: MemorySpec;
  storage: StorageSpec;
  gpu?: GpuSpec;
  sensors?: SensorSpec[];
  actuators?: ActuatorSpec[];
  peripherals?: PeripheralSpec[];
}

/**
 * CPU specification
 */
export interface CpuSpec {
  architecture: string;
  cores: number;
  frequency: number; // GHz
  instructionSet?: string[];
  features?: string[];
}

/**
 * Memory specification
 */
export interface MemorySpec {
  total: number; // MB
  available: number; // MB
  type?: string;
  speed?: number; // MHz
}

/**
 * Storage specification
 */
export interface StorageSpec {
  total: number; // GB
  available: number; // GB
  type: 'ssd' | 'hdd' | 'nvme' | 'flash';
  readSpeed?: number; // MB/s
  writeSpeed?: number; // MB/s
}

/**
 * GPU specification
 */
export interface GpuSpec {
  model: string;
  memory: number; // MB
  computeCapability?: string;
  driverVersion?: string;
  cudaSupport?: boolean;
  openclSupport?: boolean;
}

/**
 * Sensor specification
 */
export interface SensorSpec {
  type: string;
  model?: string;
  resolution?: string;
  frequency?: number; // Hz
  range?: string;
  accuracy?: string;
}

/**
 * Actuator specification
 */
export interface ActuatorSpec {
  type: string;
  model?: string;
  torque?: string;
  speed?: string;
  precision?: string;
}

/**
 * Peripheral specification
 */
export interface PeripheralSpec {
  type: string;
  interface: string;
  model?: string;
  capabilities?: string[];
}

/**
 * Software specification
 */
export interface SoftwareSpec {
  pythonVersion?: string;
  cppVersion?: string;
  libraries: LibrarySpec[];
  frameworks: FrameworkSpec[];
  dependencies: DependencySpec[];
}

/**
 * Library specification
 */
export interface LibrarySpec {
  name: string;
  version: string;
  type: 'system' | 'python' | 'cpp' | 'ros';
  source?: string;
  license?: string;
}

/**
 * Framework specification
 */
export interface FrameworkSpec {
  name: string;
  version: string;
  type: 'ml' | 'cv' | 'planning' | 'control' | 'simulation';
  capabilities?: string[];
}

/**
 * Dependency specification
 */
export interface DependencySpec {
  name: string;
  version: string;
  type: 'runtime' | 'build' | 'test' | 'optional';
  purpose: string;
}

/**
 * Network specification
 */
export interface NetworkSpec {
  bandwidth: number; // Mbps
  latency: number; // ms
  protocol: string[];
  security: string[];
  topology?: string;
}

/**
 * Security specification
 */
export interface SecuritySpec {
  encryption: string[];
  authentication: string[];
  authorization: string[];
  compliance: string[];
}

/**
 * Compatibility validation criteria
 */
export interface CompatibilityCriteria {
  algorithmId: AlgorithmId;
  environment: EnvironmentSpec;
  strictMode?: boolean;
  includeWarnings?: boolean;
  validateDependencies?: boolean;
  validatePerformance?: boolean;
  validateSecurity?: boolean;
}

/**
 * Compatibility validation result
 */
export interface CompatibilityResult {
  compatible: boolean;
  score: number; // 0-100
  issues: CompatibilityIssue[];
  warnings: CompatibilityWarning[];
  recommendations: string[];
  details: ValidationDetails;
  metadata: ValidationMetadata;
}

/**
 * Compatibility issue
 */
export interface CompatibilityIssue {
  type: 'critical' | 'error' | 'warning' | 'info';
  category: 'environment' | 'dependency' | 'performance' | 'security' | 'hardware' | 'software';
  code: string;
  message: string;
  description: string;
  impact: 'blocking' | 'high' | 'medium' | 'low';
  resolution?: string;
  affectedComponents?: string[];
  references?: string[];
}

/**
 * Compatibility warning
 */
export interface CompatibilityWarning {
  type: 'performance' | 'security' | 'maintenance' | 'future';
  message: string;
  description: string;
  severity: 'high' | 'medium' | 'low';
  recommendation?: string;
}

/**
 * Validation details
 */
export interface ValidationDetails {
  environment: EnvironmentValidation;
  dependencies: DependencyValidation;
  performance: PerformanceValidation;
  security: SecurityValidation;
  hardware: HardwareValidation;
  software: SoftwareValidation;
}

/**
 * Environment validation
 */
export interface EnvironmentValidation {
  rosVersion: ValidationCheck;
  platform: ValidationCheck;
  architecture: ValidationCheck;
  containerization: ValidationCheck;
}

/**
 * Dependency validation
 */
export interface DependencyValidation {
  rosPackages: PackageValidation[];
  systemLibraries: LibraryValidation[];
  pythonPackages: PackageValidation[];
  missing: string[];
  conflicts: string[];
  versionMismatches: string[];
}

/**
 * Performance validation
 */
export interface PerformanceValidation {
  cpuRequirements: RequirementCheck;
  memoryRequirements: RequirementCheck;
  storageRequirements: RequirementCheck;
  gpuRequirements?: RequirementCheck;
  networkRequirements?: RequirementCheck;
  estimatedPerformance: PerformanceEstimate;
}

/**
 * Security validation
 */
export interface SecurityValidation {
  permissions: SecurityCheck[];
  networkAccess: SecurityCheck[];
  dataAccess: SecurityCheck[];
  compliance: ComplianceCheck[];
  vulnerabilities: VulnerabilityCheck[];
}

/**
 * Hardware validation
 */
export interface HardwareValidation {
  sensors: SensorValidation[];
  actuators: ActuatorValidation[];
  peripherals: PeripheralValidation[];
  connectivity: ConnectivityValidation;
}

/**
 * Software validation
 */
export interface SoftwareValidation {
  languageSupport: LanguageValidation;
  frameworkSupport: FrameworkValidation;
  librarySupport: LibraryValidation;
  buildSupport: BuildValidation;
}

/**
 * Validation check result
 */
export interface ValidationCheck {
  passed: boolean;
  score: number; // 0-100
  message: string;
  details?: Record<string, unknown>;
}

/**
 * Requirement check
 */
export interface RequirementCheck {
  required: number;
  available: number;
  unit: string;
  satisfied: boolean;
  margin: number; // percentage
}

/**
 * Package validation
 */
export interface PackageValidation {
  name: string;
  requiredVersion: string;
  availableVersion?: string;
  compatible: boolean;
  installed: boolean;
  source?: string;
  issues?: string[];
}

/**
 * Library validation
 */
export interface LibraryValidation {
  name: string;
  required: boolean;
  available: boolean;
  version?: string;
  path?: string;
  issues?: string[];
}

/**
 * Performance estimate
 */
export interface PerformanceEstimate {
  executionTime: string;
  memoryUsage: string;
  cpuUsage: string;
  throughput: string;
  scalability: 'low' | 'medium' | 'high';
  bottlenecks?: string[];
}

/**
 * Security check
 */
export interface SecurityCheck {
  type: string;
  required: boolean;
  available: boolean;
  level: 'none' | 'basic' | 'advanced';
  description: string;
}

/**
 * Compliance check
 */
export interface ComplianceCheck {
  standard: string;
  compliant: boolean;
  requirements: string[];
  gaps?: string[];
  recommendations?: string[];
}

/**
 * Vulnerability check
 */
export interface VulnerabilityCheck {
  severity: 'critical' | 'high' | 'medium' | 'low';
  cve?: string;
  description: string;
  affected: string[];
  mitigation?: string;
}

/**
 * Sensor validation
 */
export interface SensorValidation {
  type: string;
  required: boolean;
  available: boolean;
  specifications: Record<string, unknown>;
  compatibility: number; // 0-100
}

/**
 * Actuator validation
 */
export interface ActuatorValidation {
  type: string;
  required: boolean;
  available: boolean;
  specifications: Record<string, unknown>;
  compatibility: number; // 0-100
}

/**
 * Peripheral validation
 */
export interface PeripheralValidation {
  type: string;
  required: boolean;
  available: boolean;
  interface: string;
  compatibility: number; // 0-100
}

/**
 * Connectivity validation
 */
export interface ConnectivityValidation {
  protocols: ProtocolValidation[];
  bandwidth: RequirementCheck;
  latency: RequirementCheck;
  reliability: number; // 0-100
}

/**
 * Protocol validation
 */
export interface ProtocolValidation {
  name: string;
  required: boolean;
  available: boolean;
  version?: string;
  security?: string;
}

/**
 * Language validation
 */
export interface LanguageValidation {
  language: string;
  requiredVersion: string;
  availableVersion?: string;
  compatible: boolean;
  features: string[];
  missingFeatures?: string[];
}

/**
 * Framework validation
 */
export interface FrameworkValidation {
  name: string;
  requiredVersion: string;
  availableVersion?: string;
  compatible: boolean;
  capabilities: string[];
  missingCapabilities?: string[];
}

/**
 * Build validation
 */
export interface BuildValidation {
  buildSystem: string;
  supported: boolean;
  dependencies: string[];
  missingDependencies?: string[];
  buildFlags?: string[];
}

/**
 * Validation metadata
 */
export interface ValidationMetadata {
  timestamp: string;
  validator: string;
  version: string;
  duration: number; // ms
  checks: number;
  passed: number;
  failed: number;
  warnings: number;
}

/**
 * Compatibility test configuration
 */
export interface CompatibilityTestConfig {
  testEnvironment: EnvironmentSpec;
  testAlgorithms: AlgorithmId[];
  testScenarios: TestScenario[];
  timeout: number; // seconds
  retries: number;
  parallel: boolean;
  reporting: ReportingConfig;
}

/**
 * Test scenario
 */
export interface TestScenario {
  name: string;
  description: string;
  setup: string[];
  execution: string[];
  cleanup: string[];
  expectedResults: Record<string, unknown>;
  timeout?: number;
}

/**
 * Reporting configuration
 */
export interface ReportingConfig {
  format: 'json' | 'xml' | 'html' | 'markdown';
  includeDetails: boolean;
  includeWarnings: boolean;
  includeRecommendations: boolean;
  outputPath?: string;
}

/**
 * Compatibility test result
 */
export interface CompatibilityTestResult {
  testId: string;
  timestamp: string;
  environment: EnvironmentSpec;
  results: AlgorithmTestResult[];
  summary: TestSummary;
  metadata: TestMetadata;
}

/**
 * Algorithm test result
 */
export interface AlgorithmTestResult {
  algorithmId: AlgorithmId;
  compatible: boolean;
  score: number;
  issues: CompatibilityIssue[];
  warnings: CompatibilityWarning[];
  testResults: TestResult[];
  duration: number; // ms
}

/**
 * Test result
 */
export interface TestResult {
  scenario: string;
  passed: boolean;
  duration: number; // ms
  output?: string;
  error?: string;
  metrics?: Record<string, unknown>;
}

/**
 * Test summary
 */
export interface TestSummary {
  totalTests: number;
  passedTests: number;
  failedTests: number;
  totalAlgorithms: number;
  compatibleAlgorithms: number;
  averageScore: number;
  criticalIssues: number;
  warnings: number;
}

/**
 * Test metadata
 */
export interface TestMetadata {
  testDuration: number; // ms
  parallelExecution: boolean;
  retries: number;
  timeout: number;
  validator: string;
  version: string;
}

// Type aliases for convenience
export type ValidationStatus = 'pass' | 'fail' | 'warning' | 'info';
export type IssueSeverity = 'critical' | 'high' | 'medium' | 'low';
export type CompatibilityLevel = 'fully_compatible' | 'mostly_compatible' | 'partially_compatible' | 'incompatible'; 