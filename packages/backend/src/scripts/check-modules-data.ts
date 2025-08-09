import path from 'path';
import dotenv from 'dotenv';
import { Database } from '../utils/database';

// Load backend .env explicitly
dotenv.config({ path: path.resolve(__dirname, '../../.env') });

async function main() {
  try {
    await Database.connect();
    console.log('🔍 Checking modules data in database...\n');

    // 1. Get all modules
    console.log('📦 ALL MODULES:');
    console.log('================');
    const allModules = await Database.query(`
      SELECT 
        id,
        name,
        type,
        category,
        supported_robots,
        is_active,
        created_at,
        updated_at
      FROM modules 
      ORDER BY name
    `);

    if ((allModules as any).rows.length === 0) {
      console.log('❌ No modules found in database');
    } else {
      console.log(`Found ${(allModules as any).rows.length} modules:`);
      (allModules as any).rows.forEach((module: any, index: number) => {
        console.log(`${index + 1}. ${module.name} (${module.type})`);
        console.log(`   Category: ${module.category || 'N/A'}`);
        console.log(
          `   Supported Robots: ${
            module.supported_robots
              ? module.supported_robots.join(', ')
              : 'None'
          }`
        );
        console.log(`   Active: ${module.is_active ? 'Yes' : 'No'}`);
        console.log(`   Created: ${module.created_at}`);
        console.log(`   Updated: ${module.updated_at}`);
        console.log('');
      });
    }

    // 2. Check if modules table exists and has data
    console.log('📊 MODULES TABLE INFO:');
    console.log('=======================');
    const tableInfo = await Database.query(`
      SELECT 
        COUNT(*) as total_modules,
        COUNT(CASE WHEN supported_robots IS NOT NULL THEN 1 END) as modules_with_robots,
        COUNT(CASE WHEN supported_robots IS NOT NULL AND array_length(supported_robots, 1) > 0 THEN 1 END) as modules_with_robot_data,
        COUNT(CASE WHEN is_active = true THEN 1 END) as active_modules
      FROM modules
    `);

    const info = (tableInfo as any).rows[0];
    console.log(`Total Modules: ${info.total_modules}`);
    console.log(
      `Modules with supported_robots field: ${info.modules_with_robots}`
    );
    console.log(
      `Modules with actual robot data: ${info.modules_with_robot_data}`
    );
    console.log(`Active Modules: ${info.active_modules}`);

    // 3. Check if there are any sample modules that should have robot data
    console.log('\n🔍 CHECKING FOR SAMPLE DATA:');
    console.log('============================');
    const sampleModules = await Database.query(`
      SELECT 
        name,
        type,
        category,
        supported_robots
      FROM modules 
      WHERE name ILIKE '%robot%' 
      OR name ILIKE '%navigation%' 
      OR name ILIKE '%perception%' 
      OR name ILIKE '%manipulation%'
      OR type ILIKE '%robot%'
      LIMIT 10
    `);

    if ((sampleModules as any).rows.length === 0) {
      console.log('❌ No sample modules found');
    } else {
      console.log('Found potential robot-related modules:');
      (sampleModules as any).rows.forEach((module: any, index: number) => {
        console.log(`${index + 1}. ${module.name} (${module.type})`);
        console.log(`   Category: ${module.category || 'N/A'}`);
        console.log(
          `   Supported Robots: ${
            module.supported_robots
              ? module.supported_robots.join(', ')
              : 'None'
          }`
        );
        console.log('');
      });
    }

    process.exit(0);
  } catch (err) {
    console.error('❌ Failed to check modules data:', err);
    process.exit(1);
  }
}

main();
