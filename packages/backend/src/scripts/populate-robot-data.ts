import path from 'path';
import dotenv from 'dotenv';
import { Database } from '../utils/database';

// Load backend .env explicitly
dotenv.config({ path: path.resolve(__dirname, '../../.env') });

async function main() {
  try {
    await Database.connect();
    console.log('🤖 Populating modules with robot data...\n');

    // Define robot data for each module
    const robotData = [
      {
        name: 'localization',
        supported_robots: [
          'turtlebot3',
          'pioneer3at',
          'kobuki',
          'create3',
          'jackal',
        ],
      },
      {
        name: 'navigation',
        supported_robots: [
          'turtlebot3',
          'pioneer3at',
          'kobuki',
          'create3',
          'jackal',
          'husky',
          'clearpath_robots',
        ],
      },
      {
        name: 'person_tracking',
        supported_robots: [
          'turtlebot3',
          'pioneer3at',
          'kobuki',
          'create3',
          'jackal',
          'pr2',
          'fetch',
          'baxter',
        ],
      },
    ];

    // Update each module with robot data
    for (const moduleData of robotData) {
      console.log(
        `📝 Updating ${
          moduleData.name
        } with supported robots: ${moduleData.supported_robots.join(', ')}`
      );

      const result = await Database.query(
        `UPDATE modules 
         SET supported_robots = $1, updated_at = NOW() 
         WHERE name = $2 
         RETURNING id, name, supported_robots`,
        [moduleData.supported_robots, moduleData.name]
      );

      if ((result as any).rows.length === 0) {
        console.log(`❌ Module '${moduleData.name}' not found`);
      } else {
        const updatedModule = (result as any).rows[0];
        console.log(
          `✅ Updated: ${
            updatedModule.name
          } - Robots: ${updatedModule.supported_robots.join(', ')}`
        );
      }
    }

    console.log('\n🎉 Robot data population completed!');
    console.log('\n📊 Updated modules:');

    // Show the updated modules
    const updatedModules = await Database.query(`
      SELECT name, type, category, supported_robots, is_active
      FROM modules 
      ORDER BY name
    `);

    (updatedModules as any).rows.forEach((module: any) => {
      console.log(`- ${module.name} (${module.type})`);
      console.log(`  Category: ${module.category}`);
      console.log(
        `  Supported Robots: ${
          module.supported_robots ? module.supported_robots.join(', ') : 'None'
        }`
      );
      console.log(`  Active: ${module.is_active ? 'Yes' : 'No'}`);
      console.log('');
    });

    // Show robot statistics
    console.log('📈 ROBOT STATISTICS:');
    console.log('=====================');
    const robotStats = await Database.query(`
      SELECT 
        robot_type,
        COUNT(m.id) as module_count
      FROM (
        SELECT DISTINCT unnest(supported_robots) AS robot_type
        FROM modules 
        WHERE is_active = true 
        AND supported_robots IS NOT NULL 
        AND array_length(supported_robots, 1) > 0
      ) AS robot_types
      LEFT JOIN modules m ON robot_type = ANY(m.supported_robots) AND m.is_active = true
      GROUP BY robot_type
      ORDER BY module_count DESC
    `);

    if ((robotStats as any).rows.length === 0) {
      console.log('❌ No robot statistics available');
    } else {
      (robotStats as any).rows.forEach((stat: any) => {
        console.log(`${stat.robot_type}: ${stat.module_count} modules`);
      });
    }

    process.exit(0);
  } catch (err) {
    console.error('❌ Failed to populate robot data:', err);
    process.exit(1);
  }
}

main();
