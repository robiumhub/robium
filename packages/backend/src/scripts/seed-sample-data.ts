#!/usr/bin/env ts-node

import dotenv from 'dotenv';
import { Database } from '../utils/database';

dotenv.config();

type UserRecord = { id: string; email: string };

async function getUserByEmail(email: string): Promise<UserRecord | null> {
  const result = (await Database.query(
    `SELECT id, email FROM users WHERE email = $1 LIMIT 1`,
    [email]
  )) as { rows: UserRecord[] };
  return result.rows[0] || null;
}

async function upsertRosPackages() {
  const packages = [
    {
      name: 'nav2-core',
      version: '1.0.0',
      description: 'Navigation stack core components',
      category: 'navigation',
      type: 'mock',
      tags: ['nav', 'planning'],
      supported_robots: ['turtlebot4', 'jackal'],
    },
    {
      name: 'rmw-cyclonedds',
      version: '0.9.0',
      description: 'Cyclone DDS middleware for ROS 2',
      category: 'communication',
      type: 'mock',
      tags: ['dds', 'rmw'],
      supported_robots: ['turtlebot4', 'husky'],
    },
    {
      name: 'depthai-oakd',
      version: '2.9.0',
      description: 'OAK-D camera integration and perception nodes',
      category: 'vision',
      type: 'mock',
      tags: ['vision', 'stereo'],
      supported_robots: ['turtlebot4'],
    },
  ];

  for (const p of packages) {
    await Database.query(
      `INSERT INTO ros_packages (name, version, description, category, type, tags, supported_robots)
       VALUES ($1, $2, $3, $4, $5, $6::jsonb, $7::text[])
       ON CONFLICT (name) DO UPDATE SET
         version = EXCLUDED.version,
         description = EXCLUDED.description,
         category = EXCLUDED.category,
         type = EXCLUDED.type,
         tags = EXCLUDED.tags,
         supported_robots = EXCLUDED.supported_robots,
         updated_at = NOW()`,
      [
        p.name,
        p.version,
        p.description,
        p.category,
        p.type,
        JSON.stringify(p.tags),
        p.supported_robots,
      ]
    );
  }
}

async function upsertModules() {
  const modules = [
    {
      name: 'localization',
      version: '1.0.0',
      description: 'AMCL-based localization module',
      category: 'navigation',
      type: 'core',
      supported_robots: ['turtlebot4', 'jackal'],
    },
    {
      name: 'mapping',
      version: '1.0.0',
      description: 'SLAM mapping module',
      category: 'navigation',
      type: 'advanced',
      supported_robots: ['husky'],
    },
  ];

  for (const m of modules) {
    await Database.query(
      `INSERT INTO modules (name, version, description, category, type, supported_robots)
       VALUES ($1, $2, $3, $4, $5, $6::text[])
       ON CONFLICT (name) DO UPDATE SET
         version = EXCLUDED.version,
         description = EXCLUDED.description,
         category = EXCLUDED.category,
         type = EXCLUDED.type,
         supported_robots = EXCLUDED.supported_robots,
         updated_at = NOW()`,
      [m.name, m.version, m.description, m.category, m.type, m.supported_robots]
    );
  }
}

async function createProject(
  ownerId: string,
  name: string,
  options: {
    description?: string;
    isTemplate?: boolean;
    tags?: string[];
    type?: string;
  } = {}
): Promise<string> {
  const result = (await Database.query(
    `INSERT INTO projects (name, description, owner_id, is_template, type, tags, created_by, updated_by)
     VALUES ($1, $2, $3, $4, $5, $6::text[], $3, $3)
     RETURNING id`,
    [
      name,
      options.description || null,
      ownerId,
      options.isTemplate || false,
      options.type || 'custom',
      options.tags || [],
    ]
  )) as { rows: { id: string }[] };
  return result.rows[0].id;
}

async function attachProjectConfig(projectId: string) {
  await Database.query(
    `INSERT INTO project_configurations (project_id, type, base_image, workdir, environment_variables, ports)
     VALUES ($1, 'python', 'python:3.11-slim', '/app', $2::jsonb, $3::jsonb)
     ON CONFLICT (project_id) DO NOTHING`,
    [
      projectId,
      JSON.stringify({ NODE_ENV: 'development' }),
      JSON.stringify([8000]),
    ]
  );
}

async function attachProjectFiles(projectId: string) {
  await Database.query(
    `INSERT INTO project_files (project_id, file_path, file_type, content, is_generated)
     VALUES ($1, 'README.md', 'doc', '# Sample Project', false)
     ON CONFLICT (project_id, file_path) DO UPDATE SET updated_at = NOW()`,
    [projectId]
  );
}

async function linkPackagesToProject(projectId: string) {
  const pkgIds = (await Database.query(
    `SELECT id FROM ros_packages WHERE name IN ('nav2-core','rmw-cyclonedds','depthai-oakd')`
  )) as { rows: { id: string }[] };
  for (const row of pkgIds.rows) {
    await Database.query(
      `INSERT INTO project_packages (project_id, package_id, is_required)
       VALUES ($1, $2, true)
       ON CONFLICT (project_id, package_id) DO NOTHING`,
      [projectId, row.id]
    );
  }
}

async function seedSampleData() {
  const admin = await getUserByEmail('admin@robium.dev');
  const test = await getUserByEmail('test@robium.dev');

  if (!admin || !test) {
    throw new Error(
      'Required users not found. Please seed users first (admin@test) before seeding sample data.'
    );
  }

  await upsertRosPackages();
  await upsertModules();

  // Admin projects (distinct content)
  const adminProject1 = await createProject(admin.id, 'Admin Control Center', {
    description: 'Admin-only project with full access features',
    tags: ['admin', 'ops'],
    type: 'custom',
  });
  await attachProjectConfig(adminProject1);
  await attachProjectFiles(adminProject1);
  await linkPackagesToProject(adminProject1);

  const adminTemplate = await createProject(admin.id, 'Admin Base Template', {
    description: 'Base template for administrative tooling projects',
    isTemplate: true,
    tags: ['template', 'admin'],
    type: 'custom',
  });
  await attachProjectConfig(adminTemplate);

  // Test user projects (different content)
  const testProject1 = await createProject(test.id, 'Demo Navigation', {
    description: 'Navigation project for demo robot',
    tags: ['demo', 'navigation'],
    type: 'custom',
  });
  await attachProjectConfig(testProject1);
  await linkPackagesToProject(testProject1);

  const testTemplate = await createProject(test.id, 'Starter Vision Template', {
    description: 'Template for camera-based perception projects',
    isTemplate: true,
    tags: ['template', 'vision'],
    type: 'custom',
  });
  await attachProjectConfig(testTemplate);
}

async function main() {
  try {
    await Database.connect();
    await seedSampleData();
    console.log('✅ Sample data seeded successfully');
  } catch (err) {
    console.error('❌ Failed to seed sample data:', err);
    process.exit(1);
  } finally {
    await Database.disconnect();
  }
}

if (require.main === module) {
  main();
}
