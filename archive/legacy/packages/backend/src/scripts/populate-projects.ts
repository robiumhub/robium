#!/usr/bin/env ts-node

import { Database } from '../utils/database';
import crypto from 'crypto';

interface Project {
  name: string;
  description: string;
  version: string;
  author: string;
  maintainer_email: string;
  license: string;
  category: string;
  type: 'custom' | 'template' | 'example';
  is_template: boolean;
  config: Record<string, any>;
  metadata: Record<string, any>;
  workspace_path: string;
  source_path: string;
  config_path: string;
  status: 'draft' | 'active' | 'archived' | 'deleted';
  module_dependencies: string[]; // Module names
  direct_packages?: string[]; // Direct package dependencies (optional)
}

const projects: Project[] = [
  {
    name: 'autonomous-navigation-robot',
    description:
      'A complete autonomous navigation robot project using localization and navigation modules',
    version: '1.0.0',
    author: 'Robium Team',
    maintainer_email: 'ros@robium.dev',
    license: 'Apache-2.0',
    category: 'navigation',
    type: 'template',
    is_template: true,
    config: {
      robot_type: 'mobile_robot',
      sensors: ['lidar', 'camera', 'imu'],
      actuators: ['wheels', 'servos'],
      environment: 'indoor',
    },
    metadata: {
      tags: ['autonomous', 'navigation', 'mobile-robot', 'indoor'],
      difficulty: 'intermediate',
      estimated_time: '2-4 weeks',
    },
    workspace_path: 'workspaces/autonomous-navigation-robot',
    source_path: 'workspaces/autonomous-navigation-robot/src',
    config_path: 'workspaces/autonomous-navigation-robot/config',
    status: 'active',
    module_dependencies: ['localization', 'navigation'],
  },
  {
    name: 'person-tracking-robot',
    description:
      'A robot that can track and follow people using computer vision',
    version: '1.0.0',
    author: 'Robium Team',
    maintainer_email: 'ros@robium.dev',
    license: 'Apache-2.0',
    category: 'perception',
    type: 'template',
    is_template: true,
    config: {
      robot_type: 'mobile_robot',
      sensors: ['camera', 'depth_camera', 'lidar'],
      actuators: ['wheels', 'pan_tilt'],
      environment: 'indoor',
    },
    metadata: {
      tags: [
        'perception',
        'person-tracking',
        'computer-vision',
        'mobile-robot',
      ],
      difficulty: 'advanced',
      estimated_time: '3-5 weeks',
    },
    workspace_path: 'workspaces/person-tracking-robot',
    source_path: 'workspaces/person-tracking-robot/src',
    config_path: 'workspaces/person-tracking-robot/config',
    status: 'active',
    module_dependencies: ['person_tracking', 'localization'],
  },
  {
    name: 'smart-home-assistant',
    description:
      'A smart home assistant robot with navigation and person tracking capabilities',
    version: '1.0.0',
    author: 'Robium Team',
    maintainer_email: 'ros@robium.dev',
    license: 'Apache-2.0',
    category: 'assistant',
    type: 'template',
    is_template: true,
    config: {
      robot_type: 'assistant_robot',
      sensors: ['camera', 'microphone', 'lidar'],
      actuators: ['wheels', 'speaker', 'display'],
      environment: 'home',
    },
    metadata: {
      tags: ['assistant', 'smart-home', 'navigation', 'person-tracking'],
      difficulty: 'advanced',
      estimated_time: '4-6 weeks',
    },
    workspace_path: 'workspaces/smart-home-assistant',
    source_path: 'workspaces/smart-home-assistant/src',
    config_path: 'workspaces/smart-home-assistant/config',
    status: 'active',
    module_dependencies: ['localization', 'navigation', 'person_tracking'],
  },
  {
    name: 'warehouse-logistics-robot',
    description: 'A warehouse logistics robot for autonomous material handling',
    version: '1.0.0',
    author: 'Robium Team',
    maintainer_email: 'ros@robium.dev',
    license: 'Apache-2.0',
    category: 'logistics',
    type: 'template',
    is_template: true,
    config: {
      robot_type: 'logistics_robot',
      sensors: ['lidar', 'camera', 'rfid'],
      actuators: ['wheels', 'lift', 'gripper'],
      environment: 'warehouse',
    },
    metadata: {
      tags: ['logistics', 'warehouse', 'autonomous', 'material-handling'],
      difficulty: 'intermediate',
      estimated_time: '3-4 weeks',
    },
    workspace_path: 'workspaces/warehouse-logistics-robot',
    source_path: 'workspaces/warehouse-logistics-robot/src',
    config_path: 'workspaces/warehouse-logistics-robot/config',
    status: 'active',
    module_dependencies: ['localization', 'navigation'],
  },
  {
    name: 'custom-localization-project',
    description: 'A custom project focusing only on localization capabilities',
    version: '0.1.0',
    author: 'User',
    maintainer_email: 'user@example.com',
    license: 'MIT',
    category: 'localization',
    type: 'custom',
    is_template: false,
    config: {
      robot_type: 'custom_robot',
      sensors: ['lidar'],
      actuators: ['wheels'],
      environment: 'outdoor',
    },
    metadata: {
      tags: ['custom', 'localization', 'outdoor'],
      difficulty: 'beginner',
      estimated_time: '1-2 weeks',
    },
    workspace_path: 'workspaces/custom-localization-project',
    source_path: 'workspaces/custom-localization-project/src',
    config_path: 'workspaces/custom-localization-project/config',
    status: 'draft',
    module_dependencies: ['localization'],
  },
];

async function populateProjects() {
  try {
    console.log('🚀 Connecting to database...');
    await Database.connect();

    console.log('📦 Populating projects...');

    // Get admin user as owner
    const userResult = (await Database.query(
      'SELECT id FROM users WHERE email = ? LIMIT 1',
      ['admin@robium.dev']
    )) as { rows: Array<{ id: string }> };

    if (userResult.rows.length === 0) {
      throw new Error('No users found in database');
    }

    const ownerUserId = userResult.rows[0].id;
    console.log(`📋 Using user ID as owner: ${ownerUserId}`);

    // First, get all module IDs for reference
    const modulesResult = (await Database.query(
      'SELECT id, name FROM modules WHERE is_active = 1'
    )) as { rows: Array<{ id: string; name: string }> };

    const moduleMap = new Map<string, string>();
    modulesResult.rows.forEach((row) => {
      moduleMap.set(row.name, row.id);
    });

    console.log('📋 Available modules:', Array.from(moduleMap.keys()));

    // Get all ROS package IDs for reference
    const packagesResult = (await Database.query(
      'SELECT id, name FROM ros_packages WHERE is_active = 1'
    )) as { rows: Array<{ id: string; name: string }> };

    const packageMap = new Map<string, string>();
    packagesResult.rows.forEach((row) => {
      packageMap.set(row.name, row.id);
    });

    console.log('📋 Available ROS packages:', Array.from(packageMap.keys()));

    // Insert projects
    const projectIds = new Map<string, string>();

    for (const project of projects) {
      console.log(`  - Adding project: ${project.name}`);

      const projectId = crypto.randomUUID();
      await Database.query(
        `
        INSERT OR REPLACE INTO projects (
          id, name, description, version, author, maintainer_email, license, category, type,
          is_template, config, metadata, workspace_path, source_path, config_path, status, owner_id, updated_at
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP)
      `,
        [
          projectId,
          project.name,
          project.description,
          project.version,
          project.author,
          project.maintainer_email,
          project.license,
          project.category,
          project.type,
          project.is_template,
          JSON.stringify(project.config),
          JSON.stringify(project.metadata),
          project.workspace_path,
          project.source_path,
          project.config_path,
          project.status,
          ownerUserId,
        ]
      );

      projectIds.set(project.name, projectId);
      console.log(
        `    ✅ Project ${project.name} added/updated with ID: ${projectId}`
      );
    }

    // Clear existing project-module relationships
    await Database.query('DELETE FROM project_module_dependencies');
    console.log('    🧹 Cleared existing project-module relationships');

    // Create project-module relationships
    for (const project of projects) {
      const projectId = projectIds.get(project.name);
      if (!projectId) continue;

      console.log(`  - Linking modules to project: ${project.name}`);

      for (let i = 0; i < project.module_dependencies.length; i++) {
        const moduleName = project.module_dependencies[i];
        const moduleId = moduleMap.get(moduleName);

        if (moduleId) {
          await Database.query(
            `
            INSERT OR REPLACE INTO project_module_dependencies (project_id, module_id, dependency_type, order_index)
            VALUES (?, ?, ?, ?)
          `,
            [projectId, moduleId, 'required', i]
          );

          console.log(
            `    ✅ Linked module ${moduleName} to project ${project.name}`
          );
        } else {
          console.log(
            `    ⚠️  Module ${moduleName} not found for project ${project.name}`
          );
        }
      }
    }

    // Clear existing project-package relationships
    await Database.query('DELETE FROM project_packages');
    console.log('    🧹 Cleared existing project-package relationships');

    // Create project-package relationships (through modules)
    for (const project of projects) {
      const projectId = projectIds.get(project.name);
      if (!projectId) continue;

      console.log(`  - Linking packages to project: ${project.name}`);

      // Get all packages from the project's modules
      const moduleNames = project.module_dependencies;
      let packageIndex = 0;

      for (const moduleName of moduleNames) {
        const moduleId = moduleMap.get(moduleName);
        if (!moduleId) continue;

        // Get packages for this module
        const modulePackagesResult = (await Database.query(
          `
          SELECT rp.id, rp.name, mp.order_index
          FROM module_packages mp
          JOIN ros_packages rp ON mp.package_id = rp.id
          WHERE mp.module_id = ?
          ORDER BY mp.order_index
        `,
          [moduleId]
        )) as {
          rows: Array<{ id: string; name: string; order_index: number }>;
        };

        for (const pkg of modulePackagesResult.rows) {
          await Database.query(
            `
            INSERT OR REPLACE INTO project_packages (project_id, package_id, is_required, order_index)
            VALUES (?, ?, ?, ?)
          `,
            [projectId, pkg.id, 1, packageIndex]
          );

          console.log(
            `    ✅ Linked package ${pkg.name} (from module ${moduleName}) to project ${project.name}`
          );
          packageIndex++;
        }
      }
    }

    // Verify the projects were added
    const countResult = (await Database.query(
      'SELECT COUNT(*) as count FROM projects'
    )) as { rows: Array<{ count: string }> };
    console.log(
      `\n📊 Total projects in database: ${countResult.rows[0].count}`
    );

    const projectsResult = (await Database.query(
      'SELECT name, category, type, status FROM projects ORDER BY name'
    )) as {
      rows: Array<{
        name: string;
        category: string;
        type: string;
        status: string;
      }>;
    };
    console.log('\n📋 Projects in database:');
    projectsResult.rows.forEach((row) => {
      console.log(
        `  - ${row.name} (${row.category}, ${row.type}, ${row.status})`
      );
    });

    // Show project-module relationships
    const relationshipsResult = (await Database.query(`
      SELECT p.name as project_name, m.name as module_name, pmd.order_index, pmd.dependency_type
      FROM project_module_dependencies pmd
      JOIN projects p ON pmd.project_id = p.id
      JOIN modules m ON pmd.module_id = m.id
      ORDER BY p.name, pmd.order_index
    `)) as {
      rows: Array<{
        project_name: string;
        module_name: string;
        order_index: number;
        dependency_type: string;
      }>;
    };

    console.log('\n🔗 Project-Module Relationships:');
    relationshipsResult.rows.forEach((row) => {
      console.log(
        `  - ${row.project_name} -> ${row.module_name} (order: ${row.order_index}, type: ${row.dependency_type})`
      );
    });

    // Show project-package relationships
    const packageRelationshipsResult = (await Database.query(`
      SELECT p.name as project_name, rp.name as package_name, pp.order_index, pp.is_required
      FROM project_packages pp
      JOIN projects p ON pp.project_id = p.id
      JOIN ros_packages rp ON pp.package_id = rp.id
      ORDER BY p.name, pp.order_index
    `)) as {
      rows: Array<{
        project_name: string;
        package_name: string;
        order_index: number;
        is_required: boolean;
      }>;
    };

    console.log('\n🔗 Project-Package Relationships:');
    packageRelationshipsResult.rows.forEach((row) => {
      console.log(
        `  - ${row.project_name} -> ${row.package_name} (order: ${row.order_index}, required: ${row.is_required})`
      );
    });

    console.log('\n✅ Projects population completed successfully!');
  } catch (error) {
    console.error('❌ Error populating projects:', error);
    process.exit(1);
  } finally {
    await Database.disconnect();
  }
}

// Run the script
if (require.main === module) {
  populateProjects();
}

export { populateProjects };
