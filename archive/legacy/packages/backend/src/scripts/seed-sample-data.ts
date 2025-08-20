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

async function upsertRosPackages() {
  const packages = [
    {
      name: 'nav2-core',
      version: '1.0.0',
      description: 'Navigation stack core components',
      category: 'navigation',
      type: 'mock',
    },
    {
      name: 'rmw-cyclonedds',
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
  ];

  for (const p of packages) {
    const id = crypto.randomUUID();
    await Database.query(
      `INSERT OR REPLACE INTO ros_packages (id, name, version, description, category, type, updated_at)
       VALUES (?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP)`,
      [id, p.name, p.version, p.description, p.category, p.type]
    );
  }
}

async function upsertModules() {
  // Optional: define minimal modules table data only if table exists
  await Database.query(
    `CREATE TABLE IF NOT EXISTS modules (
      id TEXT PRIMARY KEY,
      name TEXT NOT NULL UNIQUE,
      version TEXT,
      description TEXT,
      category TEXT,
      type TEXT,
      supported_robots TEXT,
      is_active INTEGER DEFAULT 1,
      is_default INTEGER DEFAULT 0,
      created_at TEXT DEFAULT CURRENT_TIMESTAMP,
      updated_at TEXT DEFAULT CURRENT_TIMESTAMP
    )`
  );

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
  ];

  for (const m of modules) {
    const id = crypto.randomUUID();
    await Database.query(
      `INSERT OR REPLACE INTO modules (id, name, version, description, category, type, updated_at)
       VALUES (?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP)`,
      [id, m.name, m.version, m.description, m.category, m.type]
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
  const projectId = cryptoRandomId();
  await Database.query(
    `INSERT INTO projects (id, name, description, owner_id, is_template, type, tags, created_at, updated_at)
     VALUES (?, ?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)`,
    [
      projectId,
      name,
      options.description || null,
      ownerId,
      options.isTemplate ? 1 : 0,
      options.type || 'custom',
      JSON.stringify(options.tags || []),
    ]
  );
  return projectId;
}

async function attachProjectConfig(projectId: string) {
  await Database.query(
    `CREATE TABLE IF NOT EXISTS project_configurations (
      project_id TEXT PRIMARY KEY,
      type TEXT,
      base_image TEXT,
      workdir TEXT,
      environment_variables TEXT,
      ports TEXT,
      created_at TEXT DEFAULT CURRENT_TIMESTAMP,
      updated_at TEXT DEFAULT CURRENT_TIMESTAMP
    )`
  );

  await Database.query(
    `INSERT OR IGNORE INTO project_configurations (project_id, type, base_image, workdir, environment_variables, ports)
     VALUES (?, 'python', 'python:3.11-slim', '/app', ?, ?)`,
    [
      projectId,
      JSON.stringify({ NODE_ENV: 'development' }),
      JSON.stringify([8000]),
    ]
  );
}

async function attachProjectFiles(projectId: string) {
  await Database.query(
    `INSERT OR IGNORE INTO project_files (project_id, file_path, file_type, content, is_generated)
     VALUES (?, 'README.md', 'doc', '# Sample Project', 0)`,
    [projectId]
  );
}

async function linkPackagesToProject(projectId: string) {
  const pkgIds = (await Database.query(
    `SELECT id FROM ros_packages WHERE name IN ('nav2-core','rmw-cyclonedds','depthai-oakd')`
  )) as { rows: { id: string }[] };
  for (const row of pkgIds.rows) {
    await Database.query(
      `INSERT OR IGNORE INTO project_packages (project_id, package_id, is_required)
       VALUES (?, ?, 1)`,
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

  // Datasets (minimal)
  await Database.query(
    `CREATE TABLE IF NOT EXISTS datasets (
      id TEXT PRIMARY KEY,
      name TEXT NOT NULL,
      description TEXT,
      owner_id TEXT,
      is_public INTEGER DEFAULT 1,
      created_at TEXT DEFAULT CURRENT_TIMESTAMP,
      updated_at TEXT DEFAULT CURRENT_TIMESTAMP
    )`
  );
  const ds = [
    {
      name: 'City Blocks Logs',
      description: 'Urban navigation logs',
      owner: admin.id,
    },
    {
      name: 'Warehouse Scans',
      description: 'LiDAR scans of warehouse aisles',
      owner: test.id,
    },
    {
      name: 'Camera Samples',
      description: 'Sample RGB frames for vision',
      owner: test.id,
    },
  ];
  for (const d of ds) {
    await Database.query(
      `INSERT OR IGNORE INTO datasets (id, name, description, owner_id)
       VALUES (?, ?, ?, ?)`,
      [cryptoRandomId(), d.name, d.description, d.owner]
    );
  }
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

function cryptoRandomId(): string {
  return crypto.randomUUID();
}
