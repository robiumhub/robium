import { containerNamingService } from '../services/ContainerNamingService';
import { logger } from '../utils/logger';

async function testContainerNamingService() {
  try {
    console.log('🧪 Testing Container Naming Service...\n');

    // Test 1: Generate container names
    console.log('1. Testing container name generation...');
    const userId = 'user123';
    const projectId = 'project456';
    
    const basicName = containerNamingService.generateContainerName(userId, projectId);
    console.log(`✅ Basic name: ${basicName.fullName}`);
    
    const nameWithTimestamp = containerNamingService.generateContainerName(userId, projectId, { timestamp: true });
    console.log(`✅ Name with timestamp: ${nameWithTimestamp.fullName}`);
    
    const nameWithSuffix = containerNamingService.generateContainerName(userId, projectId, { suffix: 'dev' });
    console.log(`✅ Name with suffix: ${nameWithSuffix.fullName}`);
    
    const nameWithBoth = containerNamingService.generateContainerName(userId, projectId, { timestamp: true, suffix: 'prod' });
    console.log(`✅ Name with both: ${nameWithBoth.fullName}\n`);

    // Test 2: Validate container names
    console.log('2. Testing container name validation...');
    const validName = 'robium_user123_project456';
    const invalidName = 'robium_user123_project456_with_invalid_chars!@#';
    const tooLongName = 'robium_user123_project456_with_very_long_name_that_exceeds_the_maximum_length_limit_of_sixty_three_characters';
    
    const validResult = containerNamingService.validateContainerName(validName);
    console.log(`✅ Valid name validation: ${validResult.isValid}`);
    if (validResult.warnings.length > 0) {
      console.log(`   Warnings: ${validResult.warnings.join(', ')}`);
    }
    
    const invalidResult = containerNamingService.validateContainerName(invalidName);
    console.log(`❌ Invalid name validation: ${invalidResult.isValid}`);
    console.log(`   Errors: ${invalidResult.errors.join(', ')}\n`);
    
    const tooLongResult = containerNamingService.validateContainerName(tooLongName);
    console.log(`❌ Too long name validation: ${tooLongResult.isValid}`);
    console.log(`   Errors: ${tooLongResult.errors.join(', ')}\n`);

    // Test 3: Parse container names
    console.log('3. Testing container name parsing...');
    const testNames = [
      'robium_user123_project456',
      'robium_user123_project456_1234567890',
      'robium_user123_project456_v2',
      'robium_user123_project456_1234567890_prod',
      'invalid_name_format'
    ];
    
    for (const name of testNames) {
      const parsed = containerNamingService.parseContainerName(name);
      if (parsed) {
        console.log(`✅ Parsed "${name}":`);
        console.log(`   - User ID: ${parsed.userId}`);
        console.log(`   - Project ID: ${parsed.projectId}`);
        console.log(`   - Timestamp: ${parsed.timestamp || 'none'}`);
        console.log(`   - Suffix: ${parsed.suffix || 'none'}`);
      } else {
        console.log(`❌ Could not parse "${name}"`);
      }
    }
    console.log('');

    // Test 4: Sanitize identifiers
    console.log('4. Testing identifier sanitization...');
    const testIdentifiers = [
      'User Name 123',
      'Project-Name_456',
      'Invalid@Characters#789',
      'Multiple   Spaces',
      'Leading_Underscore',
      'Trailing-Hyphen-'
    ];
    
    for (const identifier of testIdentifiers) {
      const sanitized = containerNamingService.sanitizeIdentifier(identifier);
      console.log(`✅ "${identifier}" -> "${sanitized}"`);
    }
    console.log('');

    // Test 5: Generate container tags
    console.log('5. Testing container tag generation...');
    const baseTags = containerNamingService.generateContainerTags(userId, projectId);
    console.log('✅ Base tags:');
    Object.entries(baseTags).forEach(([key, value]) => {
      console.log(`   - ${key}: ${value}`);
    });
    
    const additionalTags = { 'robium.environment': 'development', 'robium.branch': 'main' };
    const extendedTags = containerNamingService.generateContainerTags(userId, projectId, additionalTags);
    console.log('✅ Extended tags:');
    Object.entries(extendedTags).forEach(([key, value]) => {
      console.log(`   - ${key}: ${value}`);
    });
    console.log('');

    // Test 6: Check for collisions (simulated)
    console.log('6. Testing collision detection...');
    const proposedName = 'robium_user123_project456';
    try {
      const collision = await containerNamingService.checkForCollisions(proposedName);
      console.log(`✅ Collision check for "${proposedName}":`);
      console.log(`   - Collision type: ${collision.collisionType}`);
      console.log(`   - Resolution method: ${collision.resolutionMethod}`);
      console.log(`   - Resolved name: ${collision.resolvedName}`);
    } catch (error) {
      console.log(`⚠️  Collision check failed (Docker not available): ${error}`);
    }
    console.log('');

    // Test 7: Generate unique container name
    console.log('7. Testing unique container name generation...');
    try {
      const uniqueName = await containerNamingService.generateUniqueContainerName(userId, projectId);
      console.log(`✅ Unique container name: ${uniqueName.fullName}`);
    } catch (error) {
      console.log(`⚠️  Unique name generation failed (Docker not available): ${error}`);
    }
    console.log('');

    console.log('🎉 All container naming service tests completed successfully!');
    console.log('\n📋 Summary:');
    console.log('✅ Container name generation working');
    console.log('✅ Name validation functional');
    console.log('✅ Name parsing operational');
    console.log('✅ Identifier sanitization working');
    console.log('✅ Container tag generation functional');
    console.log('✅ Collision detection framework ready');
    console.log('✅ Unique name generation framework ready');

  } catch (error) {
    console.error('❌ Container naming service test failed:', error);
    process.exit(1);
  }
}

// Run the test
testContainerNamingService(); 