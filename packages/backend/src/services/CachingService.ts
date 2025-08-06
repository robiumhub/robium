import { logger } from '../utils/logger';
import fs from 'fs';
import path from 'path';
import crypto from 'crypto';

export interface CacheEntry<T = any> {
  key: string;
  value: T;
  timestamp: number;
  ttl: number;
  metadata: CacheMetadata;
}

export interface CacheMetadata {
  type: 'template' | 'generated-file' | 'docker-layer' | 'configuration';
  size: number;
  checksum: string;
  dependencies: string[];
  version: string;
}

export interface CacheConfig {
  maxSize: number; // Maximum cache size in bytes
  defaultTtl: number; // Default TTL in milliseconds
  cleanupInterval: number; // Cleanup interval in milliseconds
  enableCompression: boolean;
  enablePersistence: boolean;
  cacheDirectory: string;
}

export interface TemplateCacheEntry {
  templateName: string;
  templateContent: string;
  compiledTemplate: any;
  lastModified: number;
  checksum: string;
}

export interface GeneratedFileCacheEntry {
  filePath: string;
  content: string;
  configHash: string;
  generatedAt: number;
  dependencies: string[];
}

export interface DockerLayerCacheEntry {
  layerId: string;
  layerHash: string;
  baseImage: string;
  instructions: string[];
  size: number;
  createdAt: number;
}

export class CachingService {
  private cache: Map<string, CacheEntry> = new Map();
  private config: CacheConfig;
  private cleanupTimer?: NodeJS.Timeout;
  private cacheStats = {
    hits: 0,
    misses: 0,
    evictions: 0,
    totalSize: 0
  };

  constructor(config: Partial<CacheConfig> = {}) {
    this.config = {
      maxSize: 100 * 1024 * 1024, // 100MB default
      defaultTtl: 24 * 60 * 60 * 1000, // 24 hours default
      cleanupInterval: 60 * 60 * 1000, // 1 hour default
      enableCompression: true,
      enablePersistence: true,
      cacheDirectory: './generated/cache',
      ...config
    };

    this.initializeCache();
    this.startCleanupTimer();
  }

  /**
   * Initialize cache directory and load persisted cache
   */
  private async initializeCache(): Promise<void> {
    try {
      // Create cache directory if it doesn't exist
      if (!fs.existsSync(this.config.cacheDirectory)) {
        fs.mkdirSync(this.config.cacheDirectory, { recursive: true });
      }

      // Load persisted cache if enabled
      if (this.config.enablePersistence) {
        await this.loadPersistedCache();
      }

      logger.info('Cache service initialized successfully');
    } catch (error) {
      logger.error('Failed to initialize cache:', error as Record<string, unknown>);
    }
  }

  /**
   * Generate cache key from components
   */
  private generateCacheKey(components: string[]): string {
    const keyString = components.join('|');
    return crypto.createHash('sha256').update(keyString).digest('hex');
  }

  /**
   * Calculate checksum for content
   */
  private calculateChecksum(content: string): string {
    return crypto.createHash('md5').update(content).digest('hex');
  }

  /**
   * Get cache entry
   */
  get<T>(key: string): T | null {
    const entry = this.cache.get(key);
    
    if (!entry) {
      this.cacheStats.misses++;
      return null;
    }

    // Check if entry has expired
    if (Date.now() > entry.timestamp + entry.ttl) {
      this.cache.delete(key);
      this.cacheStats.misses++;
      return null;
    }

    this.cacheStats.hits++;
    return entry.value as T;
  }

  /**
   * Set cache entry
   */
  set<T>(key: string, value: T, ttl?: number, metadata?: Partial<CacheMetadata>): void {
    const entry: CacheEntry<T> = {
      key,
      value,
      timestamp: Date.now(),
      ttl: ttl || this.config.defaultTtl,
      metadata: {
        type: 'configuration',
        size: JSON.stringify(value).length,
        checksum: this.calculateChecksum(JSON.stringify(value)),
        dependencies: [],
        version: '1.0.0',
        ...metadata
      }
    };

    // Check cache size limit
    if (this.cacheStats.totalSize + entry.metadata.size > this.config.maxSize) {
      this.evictOldestEntries(entry.metadata.size);
    }

    this.cache.set(key, entry);
    this.cacheStats.totalSize += entry.metadata.size;

    // Persist to disk if enabled
    if (this.config.enablePersistence) {
      this.persistEntry(key, entry);
    }
  }

  /**
   * Template caching methods
   */
  cacheTemplate(templateName: string, templateContent: string, compiledTemplate: any): void {
    const key = this.generateCacheKey(['template', templateName]);
    const checksum = this.calculateChecksum(templateContent);
    
    const entry: TemplateCacheEntry = {
      templateName,
      templateContent,
      compiledTemplate,
      lastModified: Date.now(),
      checksum
    };

    this.set(key, entry, this.config.defaultTtl, {
      type: 'template',
      size: templateContent.length,
      checksum,
      dependencies: [templateName],
      version: '1.0.0'
    });
  }

  getCachedTemplate(templateName: string): TemplateCacheEntry | null {
    const key = this.generateCacheKey(['template', templateName]);
    return this.get<TemplateCacheEntry>(key);
  }

  /**
   * Generated file caching methods
   */
  cacheGeneratedFile(filePath: string, content: string, configHash: string, dependencies: string[] = []): void {
    const key = this.generateCacheKey(['generated-file', filePath, configHash]);
    
    const entry: GeneratedFileCacheEntry = {
      filePath,
      content,
      configHash,
      generatedAt: Date.now(),
      dependencies
    };

    this.set(key, entry, this.config.defaultTtl, {
      type: 'generated-file',
      size: content.length,
      checksum: this.calculateChecksum(content),
      dependencies,
      version: '1.0.0'
    });
  }

  getCachedGeneratedFile(filePath: string, configHash: string): GeneratedFileCacheEntry | null {
    const key = this.generateCacheKey(['generated-file', filePath, configHash]);
    return this.get<GeneratedFileCacheEntry>(key);
  }

  /**
   * Docker layer caching methods
   */
  cacheDockerLayer(layerId: string, layerHash: string, baseImage: string, instructions: string[], size: number): void {
    const key = this.generateCacheKey(['docker-layer', layerId, layerHash]);
    
    const entry: DockerLayerCacheEntry = {
      layerId,
      layerHash,
      baseImage,
      instructions,
      size,
      createdAt: Date.now()
    };

    this.set(key, entry, this.config.defaultTtl, {
      type: 'docker-layer',
      size,
      checksum: layerHash,
      dependencies: [baseImage, ...instructions],
      version: '1.0.0'
    });
  }

  getCachedDockerLayer(layerId: string, layerHash: string): DockerLayerCacheEntry | null {
    const key = this.generateCacheKey(['docker-layer', layerId, layerHash]);
    return this.get<DockerLayerCacheEntry>(key);
  }

  /**
   * Configuration caching methods
   */
  cacheConfiguration(configType: string, configId: string, config: any): void {
    const key = this.generateCacheKey(['configuration', configType, configId]);
    this.set(key, config, this.config.defaultTtl, {
      type: 'configuration',
      size: JSON.stringify(config).length,
      checksum: this.calculateChecksum(JSON.stringify(config)),
      dependencies: [configType, configId],
      version: '1.0.0'
    });
  }

  getCachedConfiguration(configType: string, configId: string): any {
    const key = this.generateCacheKey(['configuration', configType, configId]);
    return this.get(key);
  }

  /**
   * Cache invalidation methods
   */
  invalidateByPattern(pattern: string): void {
    const regex = new RegExp(pattern);
    for (const [key] of this.cache) {
      if (regex.test(key)) {
        this.cache.delete(key);
      }
    }
  }

  invalidateByType(type: string): void {
    for (const [key, entry] of this.cache) {
      if (entry.metadata.type === type) {
        this.cache.delete(key);
        this.cacheStats.totalSize -= entry.metadata.size;
      }
    }
  }

  invalidateByDependency(dependency: string): void {
    for (const [key, entry] of this.cache) {
      if (entry.metadata.dependencies.includes(dependency)) {
        this.cache.delete(key);
        this.cacheStats.totalSize -= entry.metadata.size;
      }
    }
  }

  /**
   * Clear all cache
   */
  clear(): void {
    this.cache.clear();
    this.cacheStats.totalSize = 0;
    this.cacheStats.evictions = 0;
    
    // Clear persisted cache files
    if (this.config.enablePersistence) {
      this.clearPersistedCache();
    }
  }

  /**
   * Get cache statistics
   */
  getStats() {
    const hitRate = this.cacheStats.hits + this.cacheStats.misses > 0 
      ? (this.cacheStats.hits / (this.cacheStats.hits + this.cacheStats.misses)) * 100 
      : 0;

    return {
      ...this.cacheStats,
      hitRate: `${hitRate.toFixed(2)}%`,
      entryCount: this.cache.size,
      maxSize: this.config.maxSize,
      usedSize: this.cacheStats.totalSize,
      usagePercentage: ((this.cacheStats.totalSize / this.config.maxSize) * 100).toFixed(2) + '%'
    };
  }

  /**
   * Evict oldest entries when cache is full
   */
  private evictOldestEntries(requiredSpace: number): void {
    const entries = Array.from(this.cache.entries())
      .sort(([, a], [, b]) => a.timestamp - b.timestamp);

    let freedSpace = 0;
    for (const [key, entry] of entries) {
      if (freedSpace >= requiredSpace) break;
      
      this.cache.delete(key);
      freedSpace += entry.metadata.size;
      this.cacheStats.evictions++;
      this.cacheStats.totalSize -= entry.metadata.size;
    }
  }

  /**
   * Start cleanup timer
   */
  private startCleanupTimer(): void {
    this.cleanupTimer = setInterval(() => {
      this.cleanupExpiredEntries();
    }, this.config.cleanupInterval);
  }

  /**
   * Cleanup expired entries
   */
  private cleanupExpiredEntries(): void {
    const now = Date.now();
    for (const [key, entry] of this.cache) {
      if (now > entry.timestamp + entry.ttl) {
        this.cache.delete(key);
        this.cacheStats.totalSize -= entry.metadata.size;
      }
    }
  }

  /**
   * Persist cache entry to disk
   */
  private async persistEntry(key: string, entry: CacheEntry): Promise<void> {
    try {
      const filePath = path.join(this.config.cacheDirectory, `${key}.json`);
      await fs.promises.writeFile(filePath, JSON.stringify(entry, null, 2));
    } catch (error) {
      logger.error('Failed to persist cache entry:', error as Record<string, unknown>);
    }
  }

  /**
   * Load persisted cache from disk
   */
  private async loadPersistedCache(): Promise<void> {
    try {
      const files = await fs.promises.readdir(this.config.cacheDirectory);
      
      for (const file of files) {
        if (file.endsWith('.json')) {
          const filePath = path.join(this.config.cacheDirectory, file);
          const content = await fs.promises.readFile(filePath, 'utf8');
          const entry: CacheEntry = JSON.parse(content);
          
          // Only load if not expired
          if (Date.now() <= entry.timestamp + entry.ttl) {
            this.cache.set(entry.key, entry);
            this.cacheStats.totalSize += entry.metadata.size;
          } else {
            // Remove expired file
            await fs.promises.unlink(filePath);
          }
        }
      }
    } catch (error) {
      logger.error('Failed to load persisted cache:', error as Record<string, unknown>);
    }
  }

  /**
   * Clear persisted cache files
   */
  private async clearPersistedCache(): Promise<void> {
    try {
      const files = await fs.promises.readdir(this.config.cacheDirectory);
      
      for (const file of files) {
        if (file.endsWith('.json')) {
          const filePath = path.join(this.config.cacheDirectory, file);
          await fs.promises.unlink(filePath);
        }
      }
    } catch (error) {
      logger.error('Failed to clear persisted cache:', error as Record<string, unknown>);
    }
  }

  /**
   * Cleanup resources
   */
  destroy(): void {
    if (this.cleanupTimer) {
      clearInterval(this.cleanupTimer);
    }
  }
}

// Export singleton instance
export const cachingService = new CachingService(); 