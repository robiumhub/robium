-- UP
-- Alter existing projects table to add new fields
ALTER TABLE projects ADD COLUMN version VARCHAR(50) DEFAULT '1.0.0';
ALTER TABLE projects ADD COLUMN author VARCHAR(255);
ALTER TABLE projects ADD COLUMN maintainer_email VARCHAR(255);
ALTER TABLE projects ADD COLUMN license VARCHAR(100) DEFAULT 'Apache-2.0';
ALTER TABLE projects ADD COLUMN category VARCHAR(100);
ALTER TABLE projects ADD COLUMN type VARCHAR(50) DEFAULT 'custom';
ALTER TABLE projects ADD COLUMN is_active BOOLEAN DEFAULT TRUE;
ALTER TABLE projects ADD COLUMN is_public BOOLEAN DEFAULT TRUE;
ALTER TABLE projects ADD COLUMN is_template BOOLEAN DEFAULT FALSE;
ALTER TABLE projects ADD COLUMN config JSONB DEFAULT '{}';
ALTER TABLE projects ADD COLUMN metadata JSONB DEFAULT '{}';
ALTER TABLE projects ADD COLUMN workspace_path VARCHAR(500);
ALTER TABLE projects ADD COLUMN source_path VARCHAR(500);
ALTER TABLE projects ADD COLUMN config_path VARCHAR(500);
ALTER TABLE projects ADD COLUMN status VARCHAR(50) DEFAULT 'draft';
ALTER TABLE projects ADD COLUMN created_by UUID REFERENCES users(id) ON DELETE SET NULL;
ALTER TABLE projects ADD COLUMN updated_by UUID REFERENCES users(id) ON DELETE SET NULL;

-- Create project_module_dependencies table for project-module relationships
CREATE TABLE IF NOT EXISTS project_module_dependencies (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    project_id UUID NOT NULL REFERENCES projects(id) ON DELETE CASCADE,
    module_id UUID NOT NULL REFERENCES modules(id) ON DELETE CASCADE,
    dependency_type VARCHAR(50) NOT NULL DEFAULT 'required',
    version_constraint VARCHAR(100), -- e.g., ">=1.0.0", "~1.2.0"
    order_index INTEGER DEFAULT 0, -- For ordering modules within project
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(project_id, module_id)
);

-- Create project_packages table for project-package relationships (direct package dependencies)
CREATE TABLE IF NOT EXISTS project_packages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    project_id UUID NOT NULL REFERENCES projects(id) ON DELETE CASCADE,
    package_id UUID NOT NULL REFERENCES ros_packages(id) ON DELETE CASCADE,
    is_required BOOLEAN DEFAULT TRUE,
    order_index INTEGER DEFAULT 0, -- For ordering packages within project
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(project_id, package_id)
);

-- Create project_files table for project file management
CREATE TABLE IF NOT EXISTS project_files (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    project_id UUID NOT NULL REFERENCES projects(id) ON DELETE CASCADE,
    file_path VARCHAR(500) NOT NULL,
    file_type VARCHAR(50), -- e.g., 'dockerfile', 'docker-compose', 'launch', 'config'
    content TEXT, -- File content (for small files)
    content_hash VARCHAR(64), -- SHA-256 hash of content
    is_generated BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(project_id, file_path)
);

-- Create indexes for better query performance
CREATE INDEX idx_projects_name ON projects(name);
CREATE INDEX idx_projects_category ON projects(category);
CREATE INDEX idx_projects_type ON projects(type);
CREATE INDEX idx_projects_status ON projects(status);
CREATE INDEX idx_projects_is_active ON projects(is_active);
CREATE INDEX idx_projects_created_by ON projects(created_by);

CREATE INDEX idx_project_module_dependencies_project_id ON project_module_dependencies(project_id);
CREATE INDEX idx_project_module_dependencies_module_id ON project_module_dependencies(module_id);
CREATE INDEX idx_project_module_dependencies_type ON project_module_dependencies(dependency_type);
CREATE INDEX idx_project_module_dependencies_order ON project_module_dependencies(order_index);

CREATE INDEX idx_project_packages_project_id ON project_packages(project_id);
CREATE INDEX idx_project_packages_package_id ON project_packages(package_id);
CREATE INDEX idx_project_packages_order ON project_packages(order_index);

CREATE INDEX idx_project_files_project_id ON project_files(project_id);
CREATE INDEX idx_project_files_file_type ON project_files(file_type);
CREATE INDEX idx_project_files_path ON project_files(file_path);

-- Create trigger for project_files updated_at
CREATE TRIGGER IF NOT EXISTS update_project_files_updated_at BEFORE UPDATE ON project_files
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- DOWN
DROP TRIGGER IF EXISTS update_project_files_updated_at ON project_files;
DROP INDEX IF EXISTS idx_project_files_path;
DROP INDEX IF EXISTS idx_project_files_file_type;
DROP INDEX IF EXISTS idx_project_files_project_id;
DROP INDEX IF EXISTS idx_project_packages_order;
DROP INDEX IF EXISTS idx_project_packages_package_id;
DROP INDEX IF EXISTS idx_project_packages_project_id;
DROP INDEX IF EXISTS idx_project_module_dependencies_order;
DROP INDEX IF EXISTS idx_project_module_dependencies_type;
DROP INDEX IF EXISTS idx_project_module_dependencies_module_id;
DROP INDEX IF EXISTS idx_project_module_dependencies_project_id;
DROP INDEX IF EXISTS idx_projects_created_by;
DROP INDEX IF EXISTS idx_projects_is_active;
DROP INDEX IF EXISTS idx_projects_status;
DROP INDEX IF EXISTS idx_projects_type;
DROP INDEX IF EXISTS idx_projects_category;
DROP INDEX IF EXISTS idx_projects_name;
DROP TABLE IF EXISTS project_files;
DROP TABLE IF EXISTS project_packages;
DROP TABLE IF EXISTS project_module_dependencies; 