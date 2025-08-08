#!/usr/bin/env ts-node

import { Database } from '../utils/database';

interface Module {
  name: string;
  version: string;
  description: string;
  category: string;
  type: 'core' | 'advanced' | 'custom';
  maintainer_email: string;
  license: string;
  packages: string[];
  dependencies: string[];
  tags: string[];
  algorithms: string[];
  source_path: string;
  config_path: string;
}

const modules: Module[] = [
  {
    name: 'localization',
    version: '1.0.0',
    description:
      'Localization module for robot positioning and state estimation',
    category: 'localization',
    type: 'core',
    maintainer_email: 'ros@robium.dev',
    license: 'Apache-2.0',
    packages: ['amcl_localization', 'kalman_filter'],
    dependencies: [],
    tags: [
      'localization',
      'positioning',
      'state-estimation',
      'amcl',
      'kalman-filter',
    ],
    algorithms: ['amcl', 'kalman-filter', 'tf2', 'rviz'],
    source_path: 'ros/src/localization',
    config_path: 'packages/shared/modules/localization.json',
  },
  {
    name: 'navigation',
    version: '1.0.0',
    description:
      'Navigation module for autonomous robot navigation and path planning',
    category: 'navigation',
    type: 'core',
    maintainer_email: 'ros@robium.dev',
    license: 'Apache-2.0',
    packages: ['amcl_localization', 'path_planning'],
    dependencies: ['localization'],
    tags: ['navigation', 'autonomous', 'path-planning', 'motion-planning'],
    algorithms: ['nav2', 'amcl', 'tf2', 'rviz'],
    source_path: 'ros/src/navigation',
    config_path: 'packages/shared/modules/navigation.json',
  },
  {
    name: 'person_tracking',
    version: '1.0.0',
    description: 'Person tracking module for human detection and tracking',
    category: 'perception',
    type: 'advanced',
    maintainer_email: 'ros@robium.dev',
    license: 'Apache-2.0',
    packages: ['person_tracking'],
    dependencies: [],
    tags: ['perception', 'person-detection', 'tracking', 'computer-vision'],
    algorithms: ['opencv', 'pcl', 'rviz'],
    source_path: 'ros/src/person_tracking',
    config_path: 'packages/shared/modules/person_tracking.json',
  },
];

async function populateModules() {
  try {
    console.log('🚀 Connecting to database...');
    await Database.connect();

    console.log('📦 Populating modules...');

    // First, get all ROS package IDs for reference
    const packagesResult = (await Database.query(
      'SELECT id, name FROM ros_packages WHERE is_active = true'
    )) as { rows: Array<{ id: string; name: string }> };

    const packageMap = new Map<string, string>();
    packagesResult.rows.forEach((row) => {
      packageMap.set(row.name, row.id);
    });

    console.log('📋 Available ROS packages:', Array.from(packageMap.keys()));

    // Insert modules
    const moduleIds = new Map<string, string>();

    for (const module of modules) {
      console.log(`  - Adding module: ${module.name}`);

      const result = (await Database.query(
        `
        INSERT INTO modules (
          name, version, description, category, type, maintainer_email, license,
          packages, dependencies, tags, algorithms, source_path, config_path
        ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13)
        ON CONFLICT (name) DO UPDATE SET
          version = EXCLUDED.version,
          description = EXCLUDED.description,
          category = EXCLUDED.category,
          type = EXCLUDED.type,
          maintainer_email = EXCLUDED.maintainer_email,
          license = EXCLUDED.license,
          packages = EXCLUDED.packages,
          dependencies = EXCLUDED.dependencies,
          tags = EXCLUDED.tags,
          algorithms = EXCLUDED.algorithms,
          source_path = EXCLUDED.source_path,
          config_path = EXCLUDED.config_path,
          updated_at = CURRENT_TIMESTAMP
        RETURNING id, name
      `,
        [
          module.name,
          module.version,
          module.description,
          module.category,
          module.type,
          module.maintainer_email,
          module.license,
          JSON.stringify(module.packages),
          JSON.stringify(module.dependencies),
          JSON.stringify(module.tags),
          JSON.stringify(module.algorithms),
          module.source_path,
          module.config_path,
        ]
      )) as { rows: Array<{ id: string; name: string }> };

      moduleIds.set(module.name, result.rows[0].id);
      console.log(
        `    ✅ Module ${module.name} added/updated with ID: ${result.rows[0].id}`
      );
    }

    // Clear existing module-package relationships
    await Database.query('DELETE FROM module_packages');
    console.log('    🧹 Cleared existing module-package relationships');

    // Create module-package relationships
    for (const module of modules) {
      const moduleId = moduleIds.get(module.name);
      if (!moduleId) continue;

      console.log(`  - Linking packages to module: ${module.name}`);

      for (let i = 0; i < module.packages.length; i++) {
        const packageName = module.packages[i];
        const packageId = packageMap.get(packageName);

        if (packageId) {
          await Database.query(
            `
            INSERT INTO module_packages (module_id, package_id, is_required, order_index)
            VALUES ($1, $2, $3, $4)
            ON CONFLICT (module_id, package_id) DO UPDATE SET
              is_required = EXCLUDED.is_required,
              order_index = EXCLUDED.order_index
          `,
            [moduleId, packageId, true, i]
          );

          console.log(
            `    ✅ Linked package ${packageName} to module ${module.name}`
          );
        } else {
          console.log(
            `    ⚠️  Package ${packageName} not found for module ${module.name}`
          );
        }
      }
    }

    // Clear existing module dependencies
    await Database.query('DELETE FROM module_dependencies');
    console.log('    🧹 Cleared existing module dependencies');

    // Create module dependencies
    for (const module of modules) {
      const moduleId = moduleIds.get(module.name);
      if (!moduleId) continue;

      console.log(`  - Setting dependencies for module: ${module.name}`);

      for (const depName of module.dependencies) {
        const depModuleId = moduleIds.get(depName);

        if (depModuleId) {
          await Database.query(
            `
            INSERT INTO module_dependencies (module_id, dependency_module_id, dependency_type)
            VALUES ($1, $2, $3)
            ON CONFLICT (module_id, dependency_module_id) DO UPDATE SET
              dependency_type = EXCLUDED.dependency_type
          `,
            [moduleId, depModuleId, 'required']
          );

          console.log(`    ✅ Added dependency: ${module.name} -> ${depName}`);
        } else {
          console.log(
            `    ⚠️  Dependency module ${depName} not found for module ${module.name}`
          );
        }
      }
    }

    // Verify the modules were added
    const countResult = (await Database.query(
      'SELECT COUNT(*) as count FROM modules'
    )) as { rows: Array<{ count: string }> };
    console.log(`\n📊 Total modules in database: ${countResult.rows[0].count}`);

    const modulesResult = (await Database.query(
      'SELECT name, category, type FROM modules ORDER BY name'
    )) as { rows: Array<{ name: string; category: string; type: string }> };
    console.log('\n📋 Modules in database:');
    modulesResult.rows.forEach((row) => {
      console.log(`  - ${row.name} (${row.category}, ${row.type})`);
    });

    // Show module-package relationships
    const relationshipsResult = (await Database.query(`
      SELECT m.name as module_name, rp.name as package_name, mp.order_index
      FROM module_packages mp
      JOIN modules m ON mp.module_id = m.id
      JOIN ros_packages rp ON mp.package_id = rp.id
      ORDER BY m.name, mp.order_index
    `)) as {
      rows: Array<{
        module_name: string;
        package_name: string;
        order_index: number;
      }>;
    };

    console.log('\n🔗 Module-Package Relationships:');
    relationshipsResult.rows.forEach((row) => {
      console.log(
        `  - ${row.module_name} -> ${row.package_name} (order: ${row.order_index})`
      );
    });

    // Show module dependencies
    const depsResult = (await Database.query(`
      SELECT m1.name as module_name, m2.name as dependency_name, md.dependency_type
      FROM module_dependencies md
      JOIN modules m1 ON md.module_id = m1.id
      JOIN modules m2 ON md.dependency_module_id = m2.id
      ORDER BY m1.name, m2.name
    `)) as {
      rows: Array<{
        module_name: string;
        dependency_name: string;
        dependency_type: string;
      }>;
    };

    console.log('\n🔗 Module Dependencies:');
    depsResult.rows.forEach((row) => {
      console.log(
        `  - ${row.module_name} -> ${row.dependency_name} (${row.dependency_type})`
      );
    });

    console.log('\n✅ Modules population completed successfully!');
  } catch (error) {
    console.error('❌ Error populating modules:', error);
    process.exit(1);
  } finally {
    await Database.disconnect();
  }
}

// Run the script
if (require.main === module) {
  populateModules();
}

export { populateModules };
