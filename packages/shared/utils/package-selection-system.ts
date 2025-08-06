import { readFileSync, writeFileSync, existsSync, mkdirSync, readdirSync, copyFileSync, statSync } from 'fs';
import { join, dirname, basename, extname } from 'path';
import { MetadataStorageSystem } from './metadata-storage-system';
import { AlgorithmDocumentation, AlgorithmId } from '../types/algorithm-documentation';
import {
  RosPackage,
  PackageDependency,
  DependencyResolution,
  PackageSelectionCriteria,
  PackageSelectionResult,
  PackageConflict,
  SelectionStatistics,
  PackageCopyConfig,
  PackageCopyResult,
  CompatibilityCriteria,
  CompatibilityResult,
  CompatibilityIssue,
  OptimizationStrategy,
  PackageSelectionConfig,
  PackageName,
  PackageVersion,
} from '../types/package-selection';

/**
 * Package Selection System
 * Provides intelligent package selection, dependency resolution, and copying capabilities
 */
export class PackageSelectionSystem {
  private config: PackageSelectionConfig;
  private metadataStorage: MetadataStorageSystem;
  private packageRegistry: Map<PackageName, RosPackage>;
  private dependencyCache: Map<string, DependencyResolution>;
  private compatibilityCache: Map<string, CompatibilityResult>;

  constructor(
    metadataStorage: MetadataStorageSystem,
    config: Partial<PackageSelectionConfig> = {}
  ) {
    this.metadataStorage = metadataStorage;
    this.config = {
      packageRegistryPath: './packages/registry',
      cacheEnabled: true,
      cacheSize: 1000,
      maxResolutionDepth: 10,
      timeout: 30000, // 30 seconds
      optimizationStrategy: {
        type: 'balanced',
        criteria: {
          minimizeSize: true,
          minimizeDependencies: true,
          maximizeCompatibility: true,
          preferStable: true,
          includeAlternatives: false,
        },
        weights: {
          size: 0.3,
          dependencies: 0.3,
          compatibility: 0.3,
          stability: 0.1,
        },
      },
      validationEnabled: true,
      backupEnabled: true,
      ...config,
    };

    this.packageRegistry = new Map();
    this.dependencyCache = new Map();
    this.compatibilityCache = new Map();

    this.loadPackageRegistry();
  }

  /**
   * Load package registry from filesystem
   */
  private loadPackageRegistry(): void {
    try {
      if (!existsSync(this.config.packageRegistryPath)) {
        mkdirSync(this.config.packageRegistryPath, { recursive: true });
        return;
      }

      const files = readdirSync(this.config.packageRegistryPath);
      for (const file of files) {
        if (extname(file) === '.json') {
          try {
            const packagePath = join(this.config.packageRegistryPath, file);
            const packageData = JSON.parse(readFileSync(packagePath, 'utf-8'));
            this.packageRegistry.set(packageData.name, packageData);
          } catch (error) {
            console.warn(`Failed to load package from ${file}:`, error);
          }
        }
      }
    } catch (error) {
      console.error('Failed to load package registry:', error);
    }
  }

  /**
   * Select packages based on algorithm requirements
   */
  selectPackages(criteria: PackageSelectionCriteria): PackageSelectionResult {
    const startTime = Date.now();
    const selectedPackages: RosPackage[] = [];
    const resolvedDependencies = new Map<string, DependencyResolution>();
    const conflicts: PackageConflict[] = [];
    const warnings: string[] = [];

    try {
      // Get algorithm documentation for each algorithm ID
      const algorithms: AlgorithmDocumentation[] = [];
      for (const algorithmId of criteria.algorithms) {
        const algorithm = this.metadataStorage.getById(algorithmId);
        if (algorithm) {
          algorithms.push(algorithm);
        } else {
          warnings.push(`Algorithm ${algorithmId} not found`);
        }
      }

      if (algorithms.length === 0) {
        throw new Error('No valid algorithms found');
      }

      // Extract required packages from algorithms
      const requiredPackages = this.extractRequiredPackages(algorithms);
      
      // Resolve dependencies for each package
      for (const packageName of requiredPackages) {
        const resolution = this.resolveDependencies(packageName, criteria);
        resolvedDependencies.set(packageName, resolution);

        if (resolution.resolved) {
          selectedPackages.push(resolution.package);
        } else {
          // Handle conflicts
          if (resolution.conflicts.length > 0) {
            conflicts.push({
              packages: [packageName, ...resolution.conflicts],
              conflictType: 'dependency',
              description: `Failed to resolve dependencies for ${packageName}`,
              severity: 'error',
            });
          }
        }
      }

      // Apply optimization strategy
      const optimizedPackages = this.optimizeSelection(selectedPackages, criteria);
      
      // Calculate statistics
      const statistics: SelectionStatistics = {
        totalPackages: optimizedPackages.length,
        totalSize: optimizedPackages.reduce((sum, pkg) => sum + (pkg.size || 0), 0),
        dependencyDepth: this.calculateDependencyDepth(resolvedDependencies),
        resolutionTime: Date.now() - startTime,
        cacheHits: 0, // TODO: Implement cache hit tracking
        cacheMisses: 0, // TODO: Implement cache miss tracking
        conflictsResolved: conflicts.filter(c => c.severity !== 'error').length,
        alternativesConsidered: 0, // TODO: Implement alternatives tracking
      };

      // Calculate optimization score
      const optimizationScore = this.calculateOptimizationScore(optimizedPackages, criteria);

      return {
        selectedPackages: optimizedPackages,
        resolvedDependencies,
        conflicts,
        warnings,
        statistics,
        optimizationScore,
      };
    } catch (error) {
      return {
        selectedPackages: [],
        resolvedDependencies: new Map(),
        conflicts: [{
          packages: [],
          conflictType: 'dependency',
          description: `Selection failed: ${error instanceof Error ? error.message : 'Unknown error'}`,
          severity: 'error',
        }],
        warnings: [...warnings, error instanceof Error ? error.message : 'Unknown error'],
        statistics: {
          totalPackages: 0,
          totalSize: 0,
          dependencyDepth: 0,
          resolutionTime: Date.now() - startTime,
          cacheHits: 0,
          cacheMisses: 0,
          conflictsResolved: 0,
          alternativesConsidered: 0,
        },
        optimizationScore: 0,
      };
    }
  }

  /**
   * Extract required packages from algorithm documentation
   */
  private extractRequiredPackages(algorithms: AlgorithmDocumentation[]): PackageName[] {
    const packages = new Set<PackageName>();
    
    for (const algorithm of algorithms) {
      for (const rosPackage of algorithm.dependencies.rosPackages) {
        if (!rosPackage.optional) {
          packages.add(rosPackage.name);
        }
      }
    }

    return Array.from(packages);
  }

  /**
   * Resolve dependencies for a package
   */
  private resolveDependencies(
    packageName: PackageName,
    criteria: PackageSelectionCriteria,
    depth: number = 0,
    path: string[] = []
  ): DependencyResolution {
    // Check cache first
    const cacheKey = `${packageName}-${criteria.targetRosVersion}-${criteria.targetPlatform}`;
    if (this.config.cacheEnabled && this.dependencyCache.has(cacheKey)) {
      return this.dependencyCache.get(cacheKey)!;
    }

    // Prevent infinite recursion
    if (depth > this.config.maxResolutionDepth || path.includes(packageName)) {
      return {
        package: this.createPlaceholderPackage(packageName),
        resolved: false,
        conflicts: [packageName],
        missing: [],
        alternatives: [],
        resolutionPath: [...path, packageName],
      };
    }

    const packageData = this.packageRegistry.get(packageName);
    if (!packageData) {
      return {
        package: this.createPlaceholderPackage(packageName),
        resolved: false,
        conflicts: [],
        missing: [{ name: packageName, type: 'ros' }],
        alternatives: [],
        resolutionPath: [...path, packageName],
      };
    }

    // Check version compatibility
    if (criteria.targetRosVersion && packageData.rosVersion) {
      if (!this.isVersionCompatible(packageData.rosVersion, criteria.targetRosVersion)) {
        return {
          package: packageData,
          resolved: false,
          conflicts: [packageName],
          missing: [],
          alternatives: this.findAlternativePackages(packageName, criteria),
          resolutionPath: [...path, packageName],
        };
      }
    }

    // Resolve dependencies recursively
    const missing: PackageDependency[] = [];
    const conflicts: string[] = [];

    for (const dependency of packageData.dependencies) {
      if (!dependency.optional || criteria.includeOptional) {
        const depResolution = this.resolveDependencies(
          dependency.name,
          criteria,
          depth + 1,
          [...path, packageName]
        );

        if (!depResolution.resolved) {
          missing.push(dependency);
          conflicts.push(...depResolution.conflicts);
        }
      }
    }

    const resolution: DependencyResolution = {
      package: packageData,
      resolved: missing.length === 0,
      conflicts,
      missing,
      alternatives: [],
      resolutionPath: [...path, packageName],
    };

    // Cache the result
    if (this.config.cacheEnabled) {
      this.dependencyCache.set(cacheKey, resolution);
    }

    return resolution;
  }

  /**
   * Create a placeholder package for missing packages
   */
  private createPlaceholderPackage(name: PackageName): RosPackage {
    return {
      name,
      version: '0.0.0',
      description: `Placeholder package for ${name}`,
      dependencies: [],
      conflicts: [],
      provides: [],
    };
  }

  /**
   * Check version compatibility
   */
  private isVersionCompatible(packageVersion: string, targetVersion: string): boolean {
    // Simple version compatibility check
    // TODO: Implement more sophisticated version comparison
    const packageMajor = packageVersion.split('.')[0];
    const targetMajor = targetVersion.split('.')[0];
    return packageMajor === targetMajor;
  }

  /**
   * Find alternative packages
   */
  private findAlternativePackages(packageName: PackageName, criteria: PackageSelectionCriteria): RosPackage[] {
    // TODO: Implement alternative package finding logic
    return [];
  }

  /**
   * Optimize package selection based on criteria
   */
  private optimizeSelection(packages: RosPackage[], criteria: PackageSelectionCriteria): RosPackage[] {
    const strategy = this.config.optimizationStrategy;

    switch (strategy.type) {
      case 'minimal':
        return this.optimizeForMinimal(packages, criteria);
      case 'balanced':
        return this.optimizeForBalanced(packages, criteria);
      case 'complete':
        return this.optimizeForComplete(packages, criteria);
      default:
        return packages;
    }
  }

  /**
   * Optimize for minimal package selection
   */
  private optimizeForMinimal(packages: RosPackage[], criteria: PackageSelectionCriteria): RosPackage[] {
    // Sort by size and remove duplicates
    const uniquePackages = this.removeDuplicatePackages(packages);
    return uniquePackages.sort((a, b) => (a.size || 0) - (b.size || 0));
  }

  /**
   * Optimize for balanced selection
   */
  private optimizeForBalanced(packages: RosPackage[], criteria: PackageSelectionCriteria): RosPackage[] {
    const uniquePackages = this.removeDuplicatePackages(packages);
    
    // Score packages based on multiple criteria
    const scoredPackages = uniquePackages.map(pkg => ({
      package: pkg,
      score: this.calculatePackageScore(pkg, criteria),
    }));

    return scoredPackages
      .sort((a, b) => b.score - a.score)
      .map(item => item.package);
  }

  /**
   * Optimize for complete selection
   */
  private optimizeForComplete(packages: RosPackage[], criteria: PackageSelectionCriteria): RosPackage[] {
    // Include all packages and alternatives
    const allPackages = [...packages];
    
    // Add alternative packages for each selected package
    for (const pkg of packages) {
      const alternatives = this.findAlternativePackages(pkg.name, criteria);
      allPackages.push(...alternatives);
    }

    return this.removeDuplicatePackages(allPackages);
  }

  /**
   * Remove duplicate packages
   */
  private removeDuplicatePackages(packages: RosPackage[]): RosPackage[] {
    const seen = new Set<string>();
    return packages.filter(pkg => {
      if (seen.has(pkg.name)) {
        return false;
      }
      seen.add(pkg.name);
      return true;
    });
  }

  /**
   * Calculate package score for optimization
   */
  private calculatePackageScore(pkg: RosPackage, criteria: PackageSelectionCriteria): number {
    const weights = this.config.optimizationStrategy.weights;
    let score = 0;

    // Size score (smaller is better)
    if (weights.size > 0) {
      const sizeScore = pkg.size ? Math.max(0, 1 - (pkg.size / 1000000)) : 0.5;
      score += weights.size * sizeScore;
    }

    // Dependency score (fewer dependencies is better)
    if (weights.dependencies > 0) {
      const depScore = Math.max(0, 1 - (pkg.dependencies.length / 10));
      score += weights.dependencies * depScore;
    }

    // Compatibility score
    if (weights.compatibility > 0) {
      const compatScore = this.calculateCompatibilityScore(pkg, criteria);
      score += weights.compatibility * compatScore;
    }

    // Stability score
    if (weights.stability > 0) {
      const stabilityScore = this.calculateStabilityScore(pkg);
      score += weights.stability * stabilityScore;
    }

    return score;
  }

  /**
   * Calculate compatibility score
   */
  private calculateCompatibilityScore(pkg: RosPackage, criteria: PackageSelectionCriteria): number {
    let score = 1.0;

    // Check ROS version compatibility
    if (criteria.targetRosVersion && pkg.rosVersion) {
      if (!this.isVersionCompatible(pkg.rosVersion, criteria.targetRosVersion)) {
        score *= 0.5;
      }
    }

    // Check platform compatibility
    if (criteria.targetPlatform && pkg.platform) {
      if (!pkg.platform.includes(criteria.targetPlatform)) {
        score *= 0.7;
      }
    }

    return score;
  }

  /**
   * Calculate stability score
   */
  private calculateStabilityScore(pkg: RosPackage): number {
    // TODO: Implement stability scoring based on version, maintainer, etc.
    return 0.8; // Default stability score
  }

  /**
   * Calculate dependency depth
   */
  private calculateDependencyDepth(resolvedDependencies: Map<string, DependencyResolution>): number {
    let maxDepth = 0;
    for (const resolution of resolvedDependencies.values()) {
      maxDepth = Math.max(maxDepth, resolution.resolutionPath.length);
    }
    return maxDepth;
  }

  /**
   * Calculate optimization score
   */
  private calculateOptimizationScore(packages: RosPackage[], criteria: PackageSelectionCriteria): number {
    if (packages.length === 0) return 0;

    const totalSize = packages.reduce((sum, pkg) => sum + (pkg.size || 0), 0);
    const totalDependencies = packages.reduce((sum, pkg) => sum + pkg.dependencies.length, 0);
    
    // Normalize scores
    const sizeScore = Math.max(0, 1 - (totalSize / 10000000)); // 10MB baseline
    const dependencyScore = Math.max(0, 1 - (totalDependencies / 100)); // 100 deps baseline
    
    return (sizeScore + dependencyScore) / 2;
  }

  /**
   * Copy selected packages to target workspace
   */
  copyPackages(
    packages: RosPackage[],
    config: PackageCopyConfig
  ): PackageCopyResult {
    const startTime = Date.now();
    const copiedPackages: string[] = [];
    const failedPackages: string[] = [];
    const errors: string[] = [];
    const warnings: string[] = [];

    try {
      // Create target directory
      if (!existsSync(config.targetPath)) {
        mkdirSync(config.targetPath, { recursive: true });
      }

      // Create backup if requested
      if (config.createBackup && existsSync(config.targetPath)) {
        const backupPath = `${config.targetPath}_backup_${Date.now()}`;
        this.copyDirectory(config.targetPath, backupPath);
      }

      // Copy each package
      for (const pkg of packages) {
        try {
          const sourcePackagePath = join(config.sourcePath, pkg.name);
          const targetPackagePath = join(config.targetPath, pkg.name);

          if (!existsSync(sourcePackagePath)) {
            failedPackages.push(pkg.name);
            errors.push(`Source package not found: ${pkg.name}`);
            continue;
          }

          // Copy package directory
          this.copyPackageDirectory(sourcePackagePath, targetPackagePath, config);
          copiedPackages.push(pkg.name);

        } catch (error) {
          failedPackages.push(pkg.name);
          errors.push(`Failed to copy ${pkg.name}: ${error instanceof Error ? error.message : 'Unknown error'}`);
        }
      }

      // Validate after copy if requested
      if (config.validateAfterCopy) {
        const validationResult = this.validateCopiedPackages(copiedPackages, config.targetPath);
        warnings.push(...validationResult.warnings);
        errors.push(...validationResult.errors);
      }

      const totalSize = this.calculateTotalSize(copiedPackages, config.targetPath);

      return {
        success: failedPackages.length === 0,
        copiedPackages,
        failedPackages,
        errors,
        warnings,
        totalSize,
        copyTime: Date.now() - startTime,
      };

    } catch (error) {
      return {
        success: false,
        copiedPackages: [],
        failedPackages: packages.map(p => p.name),
        errors: [error instanceof Error ? error.message : 'Unknown error'],
        warnings: [],
        totalSize: 0,
        copyTime: Date.now() - startTime,
      };
    }
  }

  /**
   * Copy package directory with filtering
   */
  private copyPackageDirectory(
    sourcePath: string,
    targetPath: string,
    config: PackageCopyConfig
  ): void {
    if (existsSync(targetPath) && !config.overwrite) {
      throw new Error(`Target directory already exists: ${targetPath}`);
    }

    // Create target directory
    mkdirSync(targetPath, { recursive: true });

    // Copy files and directories
    const items = readdirSync(sourcePath);
    for (const item of items) {
      const sourceItemPath = join(sourcePath, item);
      const targetItemPath = join(targetPath, item);
      const stats = statSync(sourceItemPath);

      if (stats.isDirectory()) {
        // Skip certain directories based on config
        if (item === 'test' && !config.includeTests) continue;
        if (item === 'doc' && !config.includeDocs) continue;
        if (item === 'example' && !config.includeExamples) continue;

        this.copyDirectory(sourceItemPath, targetItemPath);
      } else {
        copyFileSync(sourceItemPath, targetItemPath);
      }
    }
  }

  /**
   * Copy directory recursively
   */
  private copyDirectory(sourcePath: string, targetPath: string): void {
    mkdirSync(targetPath, { recursive: true });
    const items = readdirSync(sourcePath);

    for (const item of items) {
      const sourceItemPath = join(sourcePath, item);
      const targetItemPath = join(targetPath, item);
      const stats = statSync(sourceItemPath);

      if (stats.isDirectory()) {
        this.copyDirectory(sourceItemPath, targetItemPath);
      } else {
        copyFileSync(sourceItemPath, targetItemPath);
      }
    }
  }

  /**
   * Validate copied packages
   */
  private validateCopiedPackages(packages: string[], targetPath: string): { warnings: string[]; errors: string[] } {
    const warnings: string[] = [];
    const errors: string[] = [];

    for (const packageName of packages) {
      const packagePath = join(targetPath, packageName);
      
      if (!existsSync(packagePath)) {
        errors.push(`Package directory missing: ${packageName}`);
        continue;
      }

      // Check for required files
      const requiredFiles = ['package.xml', 'CMakeLists.txt'];
      for (const file of requiredFiles) {
        if (!existsSync(join(packagePath, file))) {
          warnings.push(`Required file missing in ${packageName}: ${file}`);
        }
      }
    }

    return { warnings, errors };
  }

  /**
   * Calculate total size of copied packages
   */
  private calculateTotalSize(packages: string[], targetPath: string): number {
    let totalSize = 0;
    
    for (const packageName of packages) {
      const packagePath = join(targetPath, packageName);
      if (existsSync(packagePath)) {
        totalSize += this.calculateDirectorySize(packagePath);
      }
    }

    return totalSize;
  }

  /**
   * Calculate directory size recursively
   */
  private calculateDirectorySize(dirPath: string): number {
    let size = 0;
    const items = readdirSync(dirPath);

    for (const item of items) {
      const itemPath = join(dirPath, item);
      const stats = statSync(itemPath);

      if (stats.isDirectory()) {
        size += this.calculateDirectorySize(itemPath);
      } else {
        size += stats.size;
      }
    }

    return size;
  }

  /**
   * Validate package compatibility
   */
  validateCompatibility(
    packages: RosPackage[],
    criteria: CompatibilityCriteria
  ): CompatibilityResult {
    const cacheKey = `${packages.map(p => p.name).join(',')}-${criteria.rosVersion}-${criteria.platform}`;
    
    if (this.config.cacheEnabled && this.compatibilityCache.has(cacheKey)) {
      return this.compatibilityCache.get(cacheKey)!;
    }

    const issues: CompatibilityIssue[] = [];
    const recommendations: string[] = [];
    let compatible = true;
    let score = 100;

    for (const pkg of packages) {
      // Check ROS version compatibility
      if (pkg.rosVersion && !this.isVersionCompatible(pkg.rosVersion, criteria.rosVersion)) {
        issues.push({
          package: pkg.name,
          issueType: 'version',
          severity: 'high',
          description: `Package ${pkg.name} requires ROS ${pkg.rosVersion}, but target is ${criteria.rosVersion}`,
          resolution: `Upgrade to ROS ${pkg.rosVersion} or find compatible alternative`,
        });
        compatible = false;
        score -= 20;
      }

      // Check platform compatibility
      if (pkg.platform && !pkg.platform.includes(criteria.platform)) {
        issues.push({
          package: pkg.name,
          issueType: 'platform',
          severity: 'medium',
          description: `Package ${pkg.name} is not tested on ${criteria.platform}`,
          resolution: 'Test package compatibility or find platform-specific alternative',
        });
        score -= 10;
      }

      // Check system library dependencies
      if (criteria.systemLibraries && pkg.dependencies) {
        for (const dep of pkg.dependencies) {
          if (dep.type === 'system' && !criteria.systemLibraries.includes(dep.name)) {
            issues.push({
              package: pkg.name,
              issueType: 'dependency',
              severity: 'medium',
              description: `Package ${pkg.name} requires system library ${dep.name}`,
              resolution: `Install system library: ${dep.name}`,
            });
            score -= 5;
          }
        }
      }
    }

    // Generate recommendations
    if (issues.length > 0) {
      recommendations.push('Consider the following actions:');
      for (const issue of issues) {
        if (issue.resolution) {
          recommendations.push(`- ${issue.resolution}`);
        }
      }
    }

    const result: CompatibilityResult = {
      compatible,
      issues,
      recommendations,
      score: Math.max(0, score),
    };

    // Cache the result
    if (this.config.cacheEnabled) {
      this.compatibilityCache.set(cacheKey, result);
    }

    return result;
  }

  /**
   * Get package statistics
   */
  getStatistics(): Record<string, unknown> {
    return {
      totalPackages: this.packageRegistry.size,
      cacheSize: this.dependencyCache.size,
      compatibilityCacheSize: this.compatibilityCache.size,
      config: this.config,
    };
  }

  /**
   * Clear caches
   */
  clearCaches(): void {
    this.dependencyCache.clear();
    this.compatibilityCache.clear();
  }

  /**
   * Reload package registry
   */
  reloadRegistry(): void {
    this.packageRegistry.clear();
    this.loadPackageRegistry();
  }
} 