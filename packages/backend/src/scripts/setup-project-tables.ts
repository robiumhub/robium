#!/usr/bin/env ts-node

import { Database } from '../utils/database';

async function setupProjectTables() {
  try {
    console.log('🚀 Connecting to database...');
    await Database.connect();

    console.log('📦 Setting up project tables...');

    // Add columns to existing projects table
    console.log('  - Adding columns to projects table...');

    const alterQueries = [
      "ALTER TABLE projects ADD COLUMN IF NOT EXISTS version VARCHAR(50) DEFAULT '1.0.0'",
      'ALTER TABLE projects ADD COLUMN IF NOT EXISTS author VARCHAR(255)',
      'ALTER TABLE projects ADD COLUMN IF NOT EXISTS maintainer_email VARCHAR(255)',
      "ALTER TABLE projects ADD COLUMN IF NOT EXISTS license VARCHAR(100) DEFAULT 'Apache-2.0'",
      'ALTER TABLE projects ADD COLUMN IF NOT EXISTS category VARCHAR(100)',
      "ALTER TABLE projects ADD COLUMN IF NOT EXISTS type VARCHAR(50) DEFAULT 'custom'",
      'ALTER TABLE projects ADD COLUMN IF NOT EXISTS is_active BOOLEAN DEFAULT TRUE',
      'ALTER TABLE projects ADD COLUMN IF NOT EXISTS is_public BOOLEAN DEFAULT TRUE',
      'ALTER TABLE projects ADD COLUMN IF NOT EXISTS is_template BOOLEAN DEFAULT FALSE',
      "ALTER TABLE projects ADD COLUMN IF NOT EXISTS config JSONB DEFAULT '{}'",
      "ALTER TABLE projects ADD COLUMN IF NOT EXISTS metadata JSONB DEFAULT '{}'",
      'ALTER TABLE projects ADD COLUMN IF NOT EXISTS workspace_path VARCHAR(500)',
      'ALTER TABLE projects ADD COLUMN IF NOT EXISTS source_path VARCHAR(500)',
      'ALTER TABLE projects ADD COLUMN IF NOT EXISTS config_path VARCHAR(500)',
      "ALTER TABLE projects ADD COLUMN IF NOT EXISTS status VARCHAR(50) DEFAULT 'draft'",
      'ALTER TABLE projects ADD COLUMN IF NOT EXISTS created_by UUID REFERENCES users(id) ON DELETE SET NULL',
      'ALTER TABLE projects ADD COLUMN IF NOT EXISTS updated_by UUID REFERENCES users(id) ON DELETE SET NULL',
    ];

    for (const query of alterQueries) {
      await Database.query(query);
      console.log(`    ✅ Executed: ${query.substring(0, 50)}...`);
    }

    // Create project_module_dependencies table
    console.log('  - Creating project_module_dependencies table...');
    await Database.query(`
      CREATE TABLE IF NOT EXISTS project_module_dependencies (
        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        project_id UUID NOT NULL REFERENCES projects(id) ON DELETE CASCADE,
        module_id UUID NOT NULL REFERENCES modules(id) ON DELETE CASCADE,
        dependency_type VARCHAR(50) NOT NULL DEFAULT 'required',
        version_constraint VARCHAR(100),
        order_index INTEGER DEFAULT 0,
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        UNIQUE(project_id, module_id)
      )
    `);
    console.log('    ✅ project_module_dependencies table created');

    // Create project_packages table
    console.log('  - Creating project_packages table...');
    await Database.query(`
      CREATE TABLE IF NOT EXISTS project_packages (
        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        project_id UUID NOT NULL REFERENCES projects(id) ON DELETE CASCADE,
        package_id UUID NOT NULL REFERENCES ros_packages(id) ON DELETE CASCADE,
        is_required BOOLEAN DEFAULT TRUE,
        order_index INTEGER DEFAULT 0,
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        UNIQUE(project_id, package_id)
      )
    `);
    console.log('    ✅ project_packages table created');

    // Create project_files table
    console.log('  - Creating project_files table...');
    await Database.query(`
      CREATE TABLE IF NOT EXISTS project_files (
        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        project_id UUID NOT NULL REFERENCES projects(id) ON DELETE CASCADE,
        file_path VARCHAR(500) NOT NULL,
        file_type VARCHAR(50),
        content TEXT,
        content_hash VARCHAR(64),
        is_generated BOOLEAN DEFAULT FALSE,
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        UNIQUE(project_id, file_path)
      )
    `);
    console.log('    ✅ project_files table created');

    // Create indexes
    console.log('  - Creating indexes...');
    const indexQueries = [
      'CREATE INDEX IF NOT EXISTS idx_projects_name ON projects(name)',
      'CREATE INDEX IF NOT EXISTS idx_projects_category ON projects(category)',
      'CREATE INDEX IF NOT EXISTS idx_projects_type ON projects(type)',
      'CREATE INDEX IF NOT EXISTS idx_projects_status ON projects(status)',
      'CREATE INDEX IF NOT EXISTS idx_projects_is_active ON projects(is_active)',
      'CREATE INDEX IF NOT EXISTS idx_projects_created_by ON projects(created_by)',
      'CREATE INDEX IF NOT EXISTS idx_project_module_dependencies_project_id ON project_module_dependencies(project_id)',
      'CREATE INDEX IF NOT EXISTS idx_project_module_dependencies_module_id ON project_module_dependencies(module_id)',
      'CREATE INDEX IF NOT EXISTS idx_project_module_dependencies_type ON project_module_dependencies(dependency_type)',
      'CREATE INDEX IF NOT EXISTS idx_project_module_dependencies_order ON project_module_dependencies(order_index)',
      'CREATE INDEX IF NOT EXISTS idx_project_packages_project_id ON project_packages(project_id)',
      'CREATE INDEX IF NOT EXISTS idx_project_packages_package_id ON project_packages(package_id)',
      'CREATE INDEX IF NOT EXISTS idx_project_packages_order ON project_packages(order_index)',
      'CREATE INDEX IF NOT EXISTS idx_project_files_project_id ON project_files(project_id)',
      'CREATE INDEX IF NOT EXISTS idx_project_files_file_type ON project_files(file_type)',
      'CREATE INDEX IF NOT EXISTS idx_project_files_path ON project_files(file_path)',
    ];

    for (const query of indexQueries) {
      await Database.query(query);
      console.log(`    ✅ Created index: ${query.split(' ')[3]}`);
    }

    // Create trigger for project_files updated_at
    console.log('  - Creating trigger for project_files...');
    try {
      await Database.query(`
        CREATE TRIGGER update_project_files_updated_at 
        BEFORE UPDATE ON project_files
        FOR EACH ROW EXECUTE FUNCTION update_updated_at_column()
      `);
      console.log('    ✅ Trigger created');
    } catch (error) {
      console.log('    ⚠️  Trigger might already exist, continuing...');
    }

    console.log('\n✅ Project tables setup completed successfully!');
  } catch (error) {
    console.error('❌ Error setting up project tables:', error);
    process.exit(1);
  } finally {
    await Database.disconnect();
  }
}

// Run the script
if (require.main === module) {
  setupProjectTables();
}

export { setupProjectTables };
