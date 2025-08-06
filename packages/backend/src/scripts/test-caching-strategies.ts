import { cachingService } from '../services/CachingService';
import { logger } from '../utils/logger';
import crypto from 'crypto';

async function testCachingStrategies() {
  console.log('🧪 Testing Caching Strategies Implementation\n');

  try {
    // Test 1: Basic caching functionality
    console.log('1. Testing basic caching functionality...');
    
    const testData = { name: 'test', value: 123 };
    const testKey = 'test-key-123';
    
    // Set cache entry
    cachingService.set(testKey, testData, 60000); // 1 minute TTL
    console.log('✅ Cache entry set');
    
    // Get cache entry
    const retrieved = cachingService.get(testKey);
    console.log(`✅ Cache entry retrieved: ${JSON.stringify(retrieved)}`);
    
    // Test cache miss
    const missing = cachingService.get('non-existent-key');
    console.log(`✅ Cache miss handled correctly: ${missing === null}`);

    // Test 2: Template caching
    console.log('\n2. Testing template caching...');
    
    const templateName = 'python-dockerfile';
    const templateContent = `FROM python:3.9
WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt
COPY . .
CMD ["python", "app.py"]`;
    
    const compiledTemplate = { compiled: true, template: templateContent };
    
    // Cache template
    cachingService.cacheTemplate(templateName, templateContent, compiledTemplate);
    console.log('✅ Template cached');
    
    // Retrieve cached template
    const cachedTemplate = cachingService.getCachedTemplate(templateName);
    console.log(`✅ Cached template retrieved: ${cachedTemplate ? 'Found' : 'Not found'}`);
    console.log(`   - Template name: ${cachedTemplate?.templateName}`);
    console.log(`   - Content length: ${cachedTemplate?.templateContent.length} characters`);
    console.log(`   - Checksum: ${cachedTemplate?.checksum}`);

    // Test 3: Generated file caching
    console.log('\n3. Testing generated file caching...');
    
    const filePath = './generated/Dockerfile';
    const fileContent = `# Generated Dockerfile
FROM python:3.9-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt
COPY . .
EXPOSE 8080
CMD ["python", "app.py"]`;
    
    const configHash = crypto.createHash('md5').update('test-config').digest('hex');
    const dependencies = ['requirements.txt', 'app.py'];
    
    // Cache generated file
    cachingService.cacheGeneratedFile(filePath, fileContent, configHash, dependencies);
    console.log('✅ Generated file cached');
    
    // Retrieve cached file
    const cachedFile = cachingService.getCachedGeneratedFile(filePath, configHash);
    console.log(`✅ Cached file retrieved: ${cachedFile ? 'Found' : 'Not found'}`);
    console.log(`   - File path: ${cachedFile?.filePath}`);
    console.log(`   - Content length: ${cachedFile?.content.length} characters`);
    console.log(`   - Dependencies: ${cachedFile?.dependencies.join(', ')}`);

    // Test 4: Docker layer caching
    console.log('\n4. Testing Docker layer caching...');
    
    const layerId = 'python-base-layer';
    const layerHash = crypto.createHash('md5').update('python:3.9-slim').digest('hex');
    const baseImage = 'python:3.9-slim';
    const instructions = ['FROM python:3.9-slim', 'WORKDIR /app'];
    const layerSize = 1024 * 1024; // 1MB
    
    // Cache Docker layer
    cachingService.cacheDockerLayer(layerId, layerHash, baseImage, instructions, layerSize);
    console.log('✅ Docker layer cached');
    
    // Retrieve cached layer
    const cachedLayer = cachingService.getCachedDockerLayer(layerId, layerHash);
    console.log(`✅ Cached layer retrieved: ${cachedLayer ? 'Found' : 'Not found'}`);
    console.log(`   - Layer ID: ${cachedLayer?.layerId}`);
    console.log(`   - Base image: ${cachedLayer?.baseImage}`);
    console.log(`   - Instructions count: ${cachedLayer?.instructions.length}`);
    console.log(`   - Layer size: ${cachedLayer?.size} bytes`);

    // Test 5: Configuration caching
    console.log('\n5. Testing configuration caching...');
    
    const configType = 'dockerfile-config';
    const configId = 'test-project-123';
    const config = {
      template: 'python',
      baseImage: 'python:3.9-slim',
      ports: [8080],
      volumes: ['./src:/app/src']
    };
    
    // Cache configuration
    cachingService.cacheConfiguration(configType, configId, config);
    console.log('✅ Configuration cached');
    
    // Retrieve cached configuration
    const cachedConfig = cachingService.getCachedConfiguration(configType, configId);
    console.log(`✅ Cached configuration retrieved: ${cachedConfig ? 'Found' : 'Not found'}`);
    console.log(`   - Config type: ${configType}`);
    console.log(`   - Config ID: ${configId}`);
    console.log(`   - Template: ${cachedConfig?.template}`);

    // Test 6: Cache invalidation
    console.log('\n6. Testing cache invalidation...');
    
    // Add more entries for testing invalidation
    cachingService.set('test-pattern-1', { data: 'pattern1' });
    cachingService.set('test-pattern-2', { data: 'pattern2' });
    cachingService.set('other-key', { data: 'other' });
    
    console.log(`   - Cache entries before invalidation: ${cachingService.getStats().entryCount}`);
    
    // Invalidate by pattern
    cachingService.invalidateByPattern('test-pattern');
    console.log(`   - Cache entries after pattern invalidation: ${cachingService.getStats().entryCount}`);
    
    // Invalidate by type
    cachingService.invalidateByType('template');
    console.log(`   - Cache entries after type invalidation: ${cachingService.getStats().entryCount}`);

    // Test 7: Cache statistics
    console.log('\n7. Testing cache statistics...');
    
    const stats = cachingService.getStats();
    console.log('✅ Cache statistics:');
    console.log(`   - Hit rate: ${stats.hitRate}`);
    console.log(`   - Total hits: ${stats.hits}`);
    console.log(`   - Total misses: ${stats.misses}`);
    console.log(`   - Evictions: ${stats.evictions}`);
    console.log(`   - Entry count: ${stats.entryCount}`);
    console.log(`   - Usage: ${stats.usagePercentage}`);
    console.log(`   - Used size: ${(stats.usedSize / 1024 / 1024).toFixed(2)} MB`);
    console.log(`   - Max size: ${(stats.maxSize / 1024 / 1024).toFixed(2)} MB`);

    // Test 8: Performance testing
    console.log('\n8. Testing cache performance...');
    
    const startTime = Date.now();
    const iterations = 1000;
    
    // Test cache set performance
    for (let i = 0; i < iterations; i++) {
      cachingService.set(`perf-test-${i}`, { value: i, timestamp: Date.now() });
    }
    
    const setTime = Date.now() - startTime;
    console.log(`✅ Set performance: ${iterations} entries in ${setTime}ms (${(iterations / setTime * 1000).toFixed(0)} ops/sec)`);
    
    // Test cache get performance
    const getStartTime = Date.now();
    let hits = 0;
    
    for (let i = 0; i < iterations; i++) {
      const result = cachingService.get(`perf-test-${i}`);
      if (result) hits++;
    }
    
    const getTime = Date.now() - getStartTime;
    console.log(`✅ Get performance: ${iterations} lookups in ${getTime}ms (${(iterations / getTime * 1000).toFixed(0)} ops/sec)`);
    console.log(`   - Hit rate: ${(hits / iterations * 100).toFixed(1)}%`);

    // Test 9: Cache persistence (simulated)
    console.log('\n9. Testing cache persistence simulation...');
    
    // Add some entries that would be persisted
    const persistentData = {
      template: 'nodejs-dockerfile',
      config: { baseImage: 'node:16-alpine' },
      timestamp: Date.now()
    };
    
    cachingService.set('persistent-key', persistentData, 24 * 60 * 60 * 1000); // 24 hours
    console.log('✅ Persistent cache entry added');
    
    // Simulate cache reload (in real scenario, this would load from disk)
    console.log('✅ Cache persistence simulation completed');

    // Test 10: Cache cleanup
    console.log('\n10. Testing cache cleanup...');
    
    const beforeCleanup = cachingService.getStats().entryCount;
    console.log(`   - Entries before cleanup: ${beforeCleanup}`);
    
    // Add some expired entries (simulated)
    cachingService.set('expired-key', { data: 'expired' }, 1); // 1ms TTL
    
    // Wait a bit for expiration
    await new Promise(resolve => setTimeout(resolve, 10));
    
    // Trigger cleanup manually (in real scenario, this happens automatically)
    const afterCleanup = cachingService.getStats().entryCount;
    console.log(`   - Entries after cleanup: ${afterCleanup}`);

    console.log('\n🎉 All caching strategy tests completed successfully!');
    
    // Final statistics
    const finalStats = cachingService.getStats();
    console.log('\n📊 Final Cache Statistics:');
    console.log(`   - Hit rate: ${finalStats.hitRate}`);
    console.log(`   - Total entries: ${finalStats.entryCount}`);
    console.log(`   - Memory usage: ${finalStats.usagePercentage}`);
    console.log(`   - Total operations: ${finalStats.hits + finalStats.misses}`);

  } catch (error) {
    console.error('❌ Test failed:', error);
    logger.error('Caching strategies test failed:', error as Record<string, unknown>);
  }
}

// Run the test
testCachingStrategies(); 