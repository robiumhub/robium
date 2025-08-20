#!/usr/bin/env ts-node

import dotenv from 'dotenv';
import { Database } from '../utils/database';
import crypto from 'crypto';

dotenv.config();

type UserRecord = { id: string; email: string };

async function getUserByEmail(email: string): Promise<UserRecord | null> {
  const result = (await Database.query(
    `SELECT id, email FROM users WHERE email = ? LIMIT 1`,
    [email]
  )) as { rows: UserRecord[] };
  return result.rows[0] || null;
}

async function seedRosPackages() {
  console.log('📦 Seeding ROS packages...');
  const packages = [
    {
      name: 'nav2-core',
      version: '1.0.0',
      description: 'Navigation stack core components',
      category: 'navigation',
      type: 'mock',
    },
    {
      name: 'rmw-cyclonedx',
      version: '0.9.0',
      description: 'Cyclone DDS middleware for ROS 2',
      category: 'communication',
      type: 'mock',
    },
    {
      name: 'depthai-oakd',
      version: '2.9.0',
      description: 'OAK-D camera integration and perception nodes',
      category: 'vision',
      type: 'mock',
    },
    {
      name: 'slam-toolbox',
      version: '2.6.0',
      description: 'SLAM mapping and localization',
      category: 'navigation',
      type: 'mock',
    },
    {
      name: 'moveit2',
      version: '2.5.0',
      description: 'Motion planning framework',
      category: 'manipulation',
      type: 'mock',
    },
    {
      name: 'robot-localization',
      version: '3.4.0',
      description: 'State estimation and sensor fusion',
      category: 'localization',
      type: 'mock',
    },
  ];

  for (const p of packages) {
    const id = crypto.randomUUID();
    await Database.query(
      `INSERT OR REPLACE INTO ros_packages (id, name, version, description, category, type, updated_at)
       VALUES (?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP)`,
      [id, p.name, p.version, p.description, p.category, p.type]
    );
  }
  console.log(`✅ Seeded ${packages.length} ROS packages`);
}

async function seedModules() {
  console.log('🔧 Seeding modules...');
  const modules = [
    {
      name: 'localization',
      version: '1.0.0',
      description: 'AMCL-based localization module',
      category: 'navigation',
      type: 'core',
    },
    {
      name: 'mapping',
      version: '1.0.0',
      description: 'SLAM mapping module',
      category: 'navigation',
      type: 'advanced',
    },
    {
      name: 'navigation',
      version: '1.0.0',
      description: 'Path planning and navigation',
      category: 'navigation',
      type: 'core',
    },
    {
      name: 'perception',
      version: '1.0.0',
      description: 'Computer vision and perception',
      category: 'vision',
      type: 'advanced',
    },
    {
      name: 'manipulation',
      version: '1.0.0',
      description: 'Robotic arm control and manipulation',
      category: 'manipulation',
      type: 'advanced',
    },
  ];

  for (const m of modules) {
    const id = crypto.randomUUID();
    await Database.query(
      `INSERT OR REPLACE INTO modules (id, name, version, description, category, type, updated_at)
       VALUES (?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP)`,
      [id, m.name, m.version, m.description, m.category, m.type]
    );
  }
  console.log(`✅ Seeded ${modules.length} modules`);
}

async function createProject(
  ownerId: string,
  name: string,
  options: {
    description?: string;
    isTemplate?: boolean;
    tags?: string[];
    type?: string;
    category?: string;
    author?: string;
    license?: string;
    config?: any;
    metadata?: any;
  } = {}
): Promise<string> {
  const projectId = crypto.randomUUID();
  await Database.query(
    `INSERT OR REPLACE INTO projects (
      id, name, description, owner_id, is_template, type, category, author, license, tags, 
      config, metadata, created_at, updated_at
    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)`,
    [
      projectId,
      name,
      options.description || null,
      ownerId,
      options.isTemplate ? 1 : 0,
      options.type || 'custom',
      options.category || 'general',
      options.author || 'Robium Team',
      options.license || 'Apache-2.0',
      JSON.stringify(options.tags || []),
      JSON.stringify(options.config || {}),
      JSON.stringify(options.metadata || {}),
    ]
  );
  return projectId;
}

async function attachProjectConfig(projectId: string, configOptions: any = {}) {
  await Database.query(
    `INSERT OR REPLACE INTO project_configurations (
      project_id, type, base_image, workdir, environment_variables, ports, created_at, updated_at
    ) VALUES (?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)`,
    [
      projectId,
      configOptions.type || 'python',
      configOptions.baseImage || 'python:3.11-slim',
      configOptions.workdir || '/app',
      JSON.stringify(configOptions.env || { NODE_ENV: 'development' }),
      JSON.stringify(configOptions.ports || [8000]),
    ]
  );
}

async function seedProjects(adminId: string) {
  console.log('🚀 Seeding sample projects and templates...');

  // Project Templates for Admin User
  const templates = [
    {
      name: 'Autonomous Navigation Robot',
      description:
        'Complete autonomous navigation robot template with localization, mapping, and path planning',
      category: 'navigation',
      tags: ['autonomous', 'navigation', 'mobile-robot', 'template'],
      isTemplate: true,
    },
    {
      name: 'Vision-Based Perception Robot',
      description:
        'Computer vision and perception robot template with object detection and tracking',
      category: 'perception',
      tags: ['vision', 'perception', 'computer-vision', 'template'],
      isTemplate: true,
    },
    {
      name: 'Warehouse Logistics Robot',
      description:
        'Warehouse automation robot template for material handling and logistics',
      category: 'logistics',
      tags: ['warehouse', 'logistics', 'automation', 'template'],
      isTemplate: true,
    },
    {
      name: 'Robotic Arm Manipulation',
      description:
        'Robotic arm manipulation template with motion planning and object manipulation',
      category: 'manipulation',
      tags: ['manipulation', 'robotic-arm', 'motion-planning', 'template'],
      isTemplate: true,
    },
    {
      name: 'Smart Home Assistant Robot',
      description:
        'Smart home assistant robot template with navigation and interaction capabilities',
      category: 'assistant',
      tags: ['smart-home', 'assistant', 'interaction', 'template'],
      isTemplate: true,
    },
  ];

  // Sample Projects for Admin User
  const projects = [
    {
      name: 'Demo Navigation Project',
      description: 'Demonstration project showcasing navigation capabilities',
      category: 'navigation',
      tags: ['demo', 'navigation', 'showcase'],
      isTemplate: false,
    },
    {
      name: 'Research Vision Project',
      description: 'Research project for advanced computer vision algorithms',
      category: 'research',
      tags: ['research', 'vision', 'algorithms'],
      isTemplate: false,
    },
    {
      name: 'Prototype Manipulation System',
      description: 'Prototype robotic manipulation system for testing',
      category: 'manipulation',
      tags: ['prototype', 'manipulation', 'testing'],
      isTemplate: false,
    },
  ];

  // Create templates
  console.log('  📋 Creating project templates...');
  for (const template of templates) {
    const projectId = await createProject(adminId, template.name, {
      description: template.description,
      isTemplate: true,
      category: template.category,
      tags: template.tags,
      type: 'template',
      config: {
        robot_type: 'mobile_robot',
        sensors: ['lidar', 'camera', 'imu'],
        actuators: ['wheels'],
        environment: 'indoor',
      },
      metadata: {
        version: '1.0.0',
        difficulty: 'intermediate',
        estimated_time: '2-4 weeks',
        supported_robots: ['turtlebot', 'kobuki', 'custom'],
      },
    });

    await attachProjectConfig(projectId, {
      type: 'python',
      baseImage: 'ros:humble-desktop',
      workdir: '/workspace',
      env: {
        ROS_DOMAIN_ID: '0',
        RMW_IMPLEMENTATION: 'rmw_cyclonedx_cpp',
      },
      ports: [8080, 8000],
    });

    console.log(`    ✅ Created template: ${template.name}`);
  }

  // Create sample projects
  console.log('  📁 Creating sample projects...');
  for (const project of projects) {
    const projectId = await createProject(adminId, project.name, {
      description: project.description,
      isTemplate: false,
      category: project.category,
      tags: project.tags,
      type: 'custom',
      config: {
        robot_type: 'custom_robot',
        sensors: ['camera'],
        actuators: ['wheels'],
        environment: 'lab',
      },
      metadata: {
        version: '0.1.0',
        status: 'development',
        last_modified: new Date().toISOString(),
      },
    });

    await attachProjectConfig(projectId);
    console.log(`    ✅ Created project: ${project.name}`);
  }
}

async function seedDatasets(adminId: string) {
  console.log('📊 Seeding sample datasets...');
  const datasets = [
    {
      name: 'Indoor Navigation Dataset',
      description: 'LiDAR and camera data for indoor navigation',
      owner: adminId,
    },
    {
      name: 'Object Detection Training Data',
      description: 'Annotated images for object detection training',
      owner: adminId,
    },
    {
      name: 'Manipulation Task Demonstrations',
      description: 'Recorded demonstrations of manipulation tasks',
      owner: adminId,
    },
    {
      name: 'Robot Sensor Calibration Data',
      description: 'Calibration data for various robot sensors',
      owner: adminId,
    },
  ];

  for (const d of datasets) {
    await Database.query(
      `INSERT OR REPLACE INTO datasets (id, name, description, owner_id, created_at, updated_at)
       VALUES (?, ?, ?, ?, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)`,
      [crypto.randomUUID(), d.name, d.description, d.owner]
    );
  }
  console.log(`✅ Seeded ${datasets.length} datasets`);
}

async function main() {
  try {
    console.log('🚀 Starting comprehensive sample data seeding...');
    await Database.connect();

    // Get admin user
    const admin = await getUserByEmail('admin@robium.dev');
    if (!admin) {
      throw new Error('Admin user not found. Please run seed-users.ts first.');
    }
    console.log(`👤 Found admin user: ${admin.email}`);

    // Seed all data
    await seedRosPackages();
    await seedModules();
    await seedProjects(admin.id);
    await seedDatasets(admin.id);

    // Summary
    const projectCount = (await Database.query(
      'SELECT COUNT(*) as count FROM projects'
    )) as { rows: { count: number }[] };
    const templateCount = (await Database.query(
      'SELECT COUNT(*) as count FROM projects WHERE is_template = 1'
    )) as { rows: { count: number }[] };
    const packageCount = (await Database.query(
      'SELECT COUNT(*) as count FROM ros_packages'
    )) as { rows: { count: number }[] };
    const moduleCount = (await Database.query(
      'SELECT COUNT(*) as count FROM modules'
    )) as { rows: { count: number }[] };
    const datasetCount = (await Database.query(
      'SELECT COUNT(*) as count FROM datasets'
    )) as { rows: { count: number }[] };

    console.log('\n📈 Summary:');
    console.log(`  🚀 Projects: ${projectCount.rows[0].count} total`);
    console.log(`  📋 Templates: ${templateCount.rows[0].count} templates`);
    console.log(`  📦 ROS Packages: ${packageCount.rows[0].count} packages`);
    console.log(`  🔧 Modules: ${moduleCount.rows[0].count} modules`);
    console.log(`  📊 Datasets: ${datasetCount.rows[0].count} datasets`);

    console.log('\n✅ All sample data seeded successfully for admin user!');
    console.log('🎉 Ready to explore the Robium platform with sample content.');
  } catch (error) {
    console.error('❌ Failed to seed sample data:', error);
    process.exit(1);
  } finally {
    await Database.disconnect();
  }
}

if (require.main === module) {
  main();
}

export { main as seedAllSampleData };
