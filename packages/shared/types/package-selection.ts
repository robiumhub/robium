// Package Selection Types
// Comprehensive type definitions for ROS package selection and dependency management

import { AlgorithmDocumentation, AlgorithmId } from './algorithm-documentation';

/**
 * Represents a ROS package with its metadata and dependencies
 */
export interface RosPackage {
  name: string;
  version: string;
  description?: string;
  maintainer?: string;
  license?: string;
  dependencies: PackageDependency[];
  conflicts?: string[];
  provides?: string[];
  category?: string;
  tags?: string[];
  size?: number; // Size in bytes
  buildType?: 'ament_cmake' | 'ament_python' | 'cmake' | 'python';
  rosVersion?: string;
  platform?: string[];
}

/**
 * Package dependency specification
 */
export interface PackageDependency {
  name: string;
  version?: string;
  versionConstraint?: '==' | '>=' | '<=' | '>' | '<' | '~=';
  optional?: boolean;
  purpose?: string;
  type?: 'ros' | 'system' | 'python' | 'build' | 'test';
}

/**
 * Dependency resolution result
 */
export interface DependencyResolution {
  package: RosPackage;
  resolved: boolean;
  conflicts: string[];
  missing: PackageDependency[];
  alternatives: RosPackage[];
  resolutionPath: string[];
}

/**
 * Package selection criteria
 */
export interface PackageSelectionCriteria {
  algorithms: AlgorithmId[];
  targetRosVersion?: string;
  targetPlatform?: string;
  maxSize?: number;
  preferredCategories?: string[];
  excludedPackages?: string[];
  includeOptional?: boolean;
  optimizationLevel?: 'minimal' | 'balanced' | 'complete';
}

/**
 * Package selection result
 */
export interface PackageSelectionResult {
  selectedPackages: RosPackage[];
  resolvedDependencies: Map<string, DependencyResolution>;
  conflicts: PackageConflict[];
  warnings: string[];
  statistics: SelectionStatistics;
  optimizationScore: number;
}

/**
 * Package conflict information
 */
export interface PackageConflict {
  packages: string[];
  conflictType: 'version' | 'dependency' | 'platform' | 'ros_version';
  description: string;
  resolution?: string;
  severity: 'error' | 'warning' | 'info';
}

/**
 * Selection statistics
 */
export interface SelectionStatistics {
  totalPackages: number;
  totalSize: number;
  dependencyDepth: number;
  resolutionTime: number;
  cacheHits: number;
  cacheMisses: number;
  conflictsResolved: number;
  alternativesConsidered: number;
}

/**
 * Package copying configuration
 */
export interface PackageCopyConfig {
  sourcePath: string;
  targetPath: string;
  includeTests?: boolean;
  includeDocs?: boolean;
  includeExamples?: boolean;
  preserveStructure?: boolean;
  overwrite?: boolean;
  validateAfterCopy?: boolean;
  createBackup?: boolean;
}

/**
 * Package copying result
 */
export interface PackageCopyResult {
  success: boolean;
  copiedPackages: string[];
  failedPackages: string[];
  errors: string[];
  warnings: string[];
  totalSize: number;
  copyTime: number;
}

/**
 * Compatibility validation criteria
 */
export interface CompatibilityCriteria {
  rosVersion: string;
  platform: string;
  architecture?: string;
  pythonVersion?: string;
  systemLibraries?: string[];
  hardwareRequirements?: string[];
}

/**
 * Compatibility validation result
 */
export interface CompatibilityResult {
  compatible: boolean;
  issues: CompatibilityIssue[];
  recommendations: string[];
  score: number; // 0-100 compatibility score
}

/**
 * Compatibility issue
 */
export interface CompatibilityIssue {
  package: string;
  issueType: 'version' | 'dependency' | 'platform' | 'hardware' | 'configuration';
  severity: 'critical' | 'high' | 'medium' | 'low';
  description: string;
  resolution?: string;
}

/**
 * Package optimization strategy
 */
export interface OptimizationStrategy {
  type: 'minimal' | 'balanced' | 'complete';
  criteria: {
    minimizeSize?: boolean;
    minimizeDependencies?: boolean;
    maximizeCompatibility?: boolean;
    preferStable?: boolean;
    includeAlternatives?: boolean;
  };
  weights: {
    size: number;
    dependencies: number;
    compatibility: number;
    stability: number;
  };
}

/**
 * Package selection engine configuration
 */
export interface PackageSelectionConfig {
  packageRegistryPath: string;
  cacheEnabled: boolean;
  cacheSize: number;
  maxResolutionDepth: number;
  timeout: number;
  optimizationStrategy: OptimizationStrategy;
  validationEnabled: boolean;
  backupEnabled: boolean;
}

// Type aliases for convenience
export type PackageName = string;
export type PackageVersion = string;
export type ResolutionPath = string[]; 