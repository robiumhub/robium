import path from 'path';
import dotenv from 'dotenv';
import { Database } from '../utils/database';

// Load backend .env explicitly
dotenv.config({ path: path.resolve(__dirname, '../../.env') });

async function main() {
  try {
    await Database.connect();
    console.log('🔍 Querying robot data from database...\n');

    // 1. Get all modules with supported robots
    console.log('📦 MODULES WITH SUPPORTED ROBOTS:');
    console.log('=====================================');
    const modulesWithRobots = await Database.query(`
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
      WHERE supported_robots IS NOT NULL 
      AND array_length(supported_robots, 1) > 0
      ORDER BY name
    `);

    if ((modulesWithRobots as any).rows.length === 0) {
      console.log('❌ No modules with supported robots found');
    } else {
      (modulesWithRobots as any).rows.forEach((module: any, index: number) => {
        console.log(`${index + 1}. ${module.name} (${module.type})`);
        console.log(`   Category: ${module.category || 'N/A'}`);
        console.log(
          `   Supported Robots: ${module.supported_robots.join(', ')}`
        );
        console.log(`   Active: ${module.is_active ? 'Yes' : 'No'}`);
        console.log(`   Updated: ${module.updated_at}`);
        console.log('');
      });
    }

    // 2. Get all unique robot types
    console.log('🤖 UNIQUE ROBOT TYPES:');
    console.log('=======================');
    const uniqueRobots = await Database.query(`
      SELECT DISTINCT unnest(supported_robots) AS robot_type
      FROM modules 
      WHERE supported_robots IS NOT NULL 
      AND array_length(supported_robots, 1) > 0
      AND is_active = true
      ORDER BY robot_type
    `);

    if ((uniqueRobots as any).rows.length === 0) {
      console.log('❌ No robot types found');
    } else {
      (uniqueRobots as any).rows.forEach((robot: any, index: number) => {
        console.log(`${index + 1}. ${robot.robot_type}`);
      });
    }
    console.log('');

    // 3. Get robot statistics
    console.log('📊 ROBOT STATISTICS:');
    console.log('=====================');
    const robotStats = await Database.query(`
      SELECT 
        robot_type,
        COUNT(m.id) as module_count,
        COUNT(DISTINCT p.id) as project_count
      FROM (
        SELECT DISTINCT unnest(supported_robots) AS robot_type
        FROM modules 
        WHERE is_active = true 
        AND supported_robots IS NOT NULL 
        AND array_length(supported_robots, 1) > 0
      ) AS robot_types
      LEFT JOIN modules m ON robot_type = ANY(m.supported_robots) AND m.is_active = true
      LEFT JOIN project_module_dependencies pmd ON m.id = pmd.module_id
      LEFT JOIN projects p ON pmd.project_id = p.id AND p.is_active = true
      GROUP BY robot_type
      ORDER BY module_count DESC, project_count DESC
    `);

    if ((robotStats as any).rows.length === 0) {
      console.log('❌ No robot statistics available');
    } else {
      (robotStats as any).rows.forEach((stat: any) => {
        console.log(`${stat.robot_type}:`);
        console.log(`  - Modules: ${stat.module_count}`);
        console.log(`  - Projects: ${stat.project_count}`);
      });
    }
    console.log('');

    // 4. Get robot category distribution
    console.log('🏷️ ROBOT CATEGORY DISTRIBUTION:');
    console.log('===============================');
    const categoryStats = await Database.query(`
      SELECT 
        m.category,
        COUNT(DISTINCT robot_type) as robot_count
      FROM modules m,
      LATERAL unnest(m.supported_robots) AS robot_type
      WHERE m.is_active = true 
      AND m.supported_robots IS NOT NULL 
      AND array_length(m.supported_robots, 1) > 0
      AND m.category IS NOT NULL
      GROUP BY m.category
      ORDER BY robot_count DESC
    `);

    if ((categoryStats as any).rows.length === 0) {
      console.log('❌ No category distribution available');
    } else {
      (categoryStats as any).rows.forEach((cat: any) => {
        console.log(`${cat.category}: ${cat.robot_count} robot types`);
      });
    }
    console.log('');

    // 5. Get recent robot activity
    console.log('🕒 RECENT ROBOT ACTIVITY (Last 30 days):');
    console.log('=========================================');
    const recentActivity = await Database.query(`
      SELECT 
        m.name,
        m.type,
        m.category,
        m.updated_at,
        array_agg(DISTINCT robot_type) as supported_robots
      FROM modules m,
      LATERAL unnest(m.supported_robots) AS robot_type
      WHERE m.is_active = true 
      AND m.supported_robots IS NOT NULL 
      AND array_length(m.supported_robots, 1) > 0
      AND m.updated_at >= NOW() - INTERVAL '30 days'
      GROUP BY m.id, m.name, m.type, m.category, m.updated_at
      ORDER BY m.updated_at DESC
      LIMIT 10
    `);

    if ((recentActivity as any).rows.length === 0) {
      console.log('❌ No recent activity found');
    } else {
      (recentActivity as any).rows.forEach((activity: any, index: number) => {
        console.log(`${index + 1}. ${activity.name} (${activity.type})`);
        console.log(`   Category: ${activity.category || 'N/A'}`);
        console.log(`   Robots: ${activity.supported_robots.join(', ')}`);
        console.log(`   Updated: ${activity.updated_at}`);
        console.log('');
      });
    }

    // 6. Get total counts
    console.log('📈 SUMMARY:');
    console.log('===========');
    const totalCounts = await Database.query(`
      SELECT 
        COUNT(DISTINCT robot_type) as total_robot_types,
        COUNT(DISTINCT m.id) as total_modules_with_robots,
        COUNT(DISTINCT p.id) as total_projects_using_robots
      FROM (
        SELECT DISTINCT unnest(supported_robots) AS robot_type
        FROM modules 
        WHERE is_active = true 
        AND supported_robots IS NOT NULL 
        AND array_length(supported_robots, 1) > 0
      ) AS robot_types
      LEFT JOIN modules m ON robot_type = ANY(m.supported_robots) AND m.is_active = true
      LEFT JOIN project_module_dependencies pmd ON m.id = pmd.module_id
      LEFT JOIN projects p ON pmd.project_id = p.id AND p.is_active = true
    `);

    const counts = (totalCounts as any).rows[0];
    console.log(`Total Robot Types: ${counts.total_robot_types}`);
    console.log(
      `Total Modules with Robots: ${counts.total_modules_with_robots}`
    );
    console.log(
      `Total Projects Using Robots: ${counts.total_projects_using_robots}`
    );

    process.exit(0);
  } catch (err) {
    console.error('❌ Failed to get robot data:', err);
    process.exit(1);
  }
}

main();
