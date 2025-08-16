-- UP
-- Create extension for UUID generation
CREATE EXTENSION IF NOT EXISTS pgcrypto;

-- Create function to update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ language 'plpgsql';

-- USERS
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    username VARCHAR(100) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    is_active BOOLEAN DEFAULT TRUE,
    role VARCHAR(20) NOT NULL DEFAULT 'user' CHECK (role IN ('admin', 'user')),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_username ON users(username);
CREATE TRIGGER update_users_updated_at BEFORE UPDATE ON users
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- PROJECTS
CREATE TABLE projects (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name VARCHAR(255) NOT NULL,
    description TEXT,
    owner_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    -- Extended fields
    version VARCHAR(50) DEFAULT '1.0.0',
    author VARCHAR(255),
    maintainer_email VARCHAR(255),
    license VARCHAR(100) DEFAULT 'Apache-2.0',
    type VARCHAR(50) DEFAULT 'custom',
    is_active BOOLEAN DEFAULT TRUE,
    is_template BOOLEAN DEFAULT FALSE,
    config JSONB DEFAULT '{}',
    metadata JSONB DEFAULT '{}',
    workspace_path VARCHAR(500),
    source_path VARCHAR(500),
    config_path VARCHAR(500),
    created_by UUID REFERENCES users(id) ON DELETE SET NULL,
    updated_by UUID REFERENCES users(id) ON DELETE SET NULL,
    tags TEXT[] DEFAULT '{}'::text[],
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_projects_owner_id ON projects(owner_id);
CREATE INDEX idx_projects_created_at ON projects(created_at);
CREATE INDEX idx_projects_name ON projects(name);
CREATE INDEX idx_projects_type ON projects(type);
CREATE INDEX idx_projects_is_active ON projects(is_active);
CREATE INDEX idx_projects_created_by ON projects(created_by);
CREATE INDEX idx_projects_tags ON projects USING GIN (tags);
CREATE TRIGGER update_projects_updated_at BEFORE UPDATE ON projects
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- SESSIONS (JWT/refresh)
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token_hash VARCHAR(255) NOT NULL,
    refresh_token_hash VARCHAR(255),
    expires_at TIMESTAMP NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    last_used_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    user_agent TEXT,
    ip_address INET
);
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_expires_at ON sessions(expires_at);

-- PROJECT MEMBERS
CREATE TABLE project_members (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    project_id UUID NOT NULL REFERENCES projects(id) ON DELETE CASCADE,
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL DEFAULT 'member' CHECK (role IN ('owner', 'admin', 'member', 'viewer')),
    joined_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(project_id, user_id)
);
CREATE INDEX idx_project_members_project_id ON project_members(project_id);
CREATE INDEX idx_project_members_user_id ON project_members(user_id);

-- PROJECT CONFIGURATIONS
CREATE TABLE project_configurations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    project_id UUID NOT NULL REFERENCES projects(id) ON DELETE CASCADE,
    type VARCHAR(20) NOT NULL DEFAULT 'python' CHECK (type IN ('python', 'nodejs', 'java', 'golang', 'rust', 'cpp', 'custom')),
    version VARCHAR(20) NOT NULL DEFAULT '1.0.0',
    base_image VARCHAR(255) NOT NULL DEFAULT 'python:3.11-slim',
    workdir VARCHAR(255) NOT NULL DEFAULT '/app',
    system_dependencies JSONB DEFAULT '[]',
    python_dependencies JSONB DEFAULT '[]',
    node_dependencies JSONB DEFAULT '[]',
    java_dependencies JSONB DEFAULT '[]',
    golang_dependencies JSONB DEFAULT '[]',
    rust_dependencies JSONB DEFAULT '[]',
    cpp_dependencies JSONB DEFAULT '[]',
    custom_dependencies JSONB DEFAULT '[]',
    environment_variables JSONB DEFAULT '{}',
    ports JSONB DEFAULT '[]',
    volumes JSONB DEFAULT '[]',
    command TEXT,
    entrypoint TEXT,
    runtime_user VARCHAR(100),
    health_check_command TEXT,
    health_check_interval VARCHAR(20),
    health_check_timeout VARCHAR(20),
    health_check_retries INTEGER DEFAULT 3,
    health_check_start_period VARCHAR(20),
    build_args JSONB DEFAULT '{}',
    labels JSONB DEFAULT '{}',
    multi_stage BOOLEAN DEFAULT FALSE,
    stages JSONB DEFAULT '[]',
    category VARCHAR(100),
    tags JSONB DEFAULT '[]',
    is_public BOOLEAN DEFAULT FALSE,
    algorithms JSONB DEFAULT '[]',
    max_memory INTEGER DEFAULT 2048,
    cpu_limit INTEGER DEFAULT 2,
    enable_gpu BOOLEAN DEFAULT FALSE,
    auto_save BOOLEAN DEFAULT TRUE,
    enable_debugging BOOLEAN DEFAULT FALSE,
    enable_logging BOOLEAN DEFAULT TRUE,
    backup_frequency VARCHAR(20) DEFAULT 'weekly',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(project_id)
);
CREATE INDEX idx_project_configurations_project_id ON project_configurations(project_id);
CREATE INDEX idx_project_configurations_type ON project_configurations(type);
CREATE INDEX idx_project_configurations_category ON project_configurations(category);
CREATE TRIGGER update_project_configurations_updated_at BEFORE UPDATE ON project_configurations
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- USER ACTIVITY LOGS
CREATE TABLE user_activity_logs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    project_id UUID REFERENCES projects(id) ON DELETE SET NULL,
    action VARCHAR(100) NOT NULL,
    details JSONB DEFAULT '{}',
    ip_address INET,
    user_agent TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_user_activity_logs_user_id ON user_activity_logs(user_id);
CREATE INDEX idx_user_activity_logs_project_id ON user_activity_logs(project_id);
CREATE INDEX idx_user_activity_logs_created_at ON user_activity_logs(created_at);

-- CONTAINER STATES
CREATE TABLE container_states (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    project_id UUID NOT NULL REFERENCES projects(id) ON DELETE CASCADE,
    container_id VARCHAR(255),
    container_name VARCHAR(255) NOT NULL,
    status VARCHAR(50) NOT NULL DEFAULT 'stopped',
    image VARCHAR(255),
    ports JSONB DEFAULT '{}',
    volumes JSONB DEFAULT '[]',
    environment JSONB DEFAULT '[]',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    started_at TIMESTAMP,
    stopped_at TIMESTAMP
);
CREATE INDEX idx_container_states_project_id ON container_states(project_id);
CREATE INDEX idx_container_states_status ON container_states(status);
CREATE INDEX idx_container_states_created_at ON container_states(created_at);
CREATE TRIGGER update_container_states_updated_at BEFORE UPDATE ON container_states
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- ADMIN AUDIT TRAILS
CREATE TABLE admin_audit_trails (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    admin_user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    target_user_id UUID REFERENCES users(id) ON DELETE SET NULL,
    target_project_id UUID REFERENCES projects(id) ON DELETE SET NULL,
    action VARCHAR(100) NOT NULL,
    details JSONB DEFAULT '{}',
    ip_address INET,
    user_agent TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_admin_audit_trails_admin_user_id ON admin_audit_trails(admin_user_id);
CREATE INDEX idx_admin_audit_trails_created_at ON admin_audit_trails(created_at);

-- ROS PACKAGES
CREATE TABLE ros_packages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name VARCHAR(255) NOT NULL UNIQUE,
    version VARCHAR(50) NOT NULL DEFAULT '0.0.0',
    description TEXT,
    category VARCHAR(100),
    type VARCHAR(50) NOT NULL DEFAULT 'mock' CHECK (type IN ('mock', 'production', 'custom')),
    maintainer_email VARCHAR(255),
    license VARCHAR(100) DEFAULT 'Apache-2.0',
    build_dependencies JSONB DEFAULT '[]',
    runtime_dependencies JSONB DEFAULT '[]',
    test_dependencies JSONB DEFAULT '[]',
    build_type VARCHAR(50) DEFAULT 'ament_cmake',
    packages JSONB DEFAULT '[]',
    published_topics JSONB DEFAULT '[]',
    subscribed_topics JSONB DEFAULT '[]',
    advertised_services JSONB DEFAULT '[]',
    called_services JSONB DEFAULT '[]',
    tags JSONB DEFAULT '[]',
    algorithms JSONB DEFAULT '[]',
    source_path VARCHAR(500),
    package_xml_path VARCHAR(500),
    cmake_lists_path VARCHAR(500),
    supported_robots TEXT[] DEFAULT '{}'::text[],
    is_active BOOLEAN DEFAULT TRUE,
    is_public BOOLEAN DEFAULT TRUE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_ros_packages_name ON ros_packages(name);
CREATE INDEX idx_ros_packages_category ON ros_packages(category);
CREATE INDEX idx_ros_packages_type ON ros_packages(type);
CREATE INDEX idx_ros_packages_is_active ON ros_packages(is_active);
CREATE INDEX idx_ros_packages_created_at ON ros_packages(created_at);
CREATE INDEX idx_ros_packages_supported_robots ON ros_packages USING GIN (supported_robots);
CREATE TRIGGER update_ros_packages_updated_at BEFORE UPDATE ON ros_packages
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TABLE ros_package_versions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    package_id UUID NOT NULL REFERENCES ros_packages(id) ON DELETE CASCADE,
    version VARCHAR(50) NOT NULL,
    changelog TEXT,
    release_notes TEXT,
    is_stable BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(package_id, version)
);
CREATE INDEX idx_ros_package_versions_package_id ON ros_package_versions(package_id);
CREATE INDEX idx_ros_package_versions_version ON ros_package_versions(version);
CREATE INDEX idx_ros_package_versions_is_stable ON ros_package_versions(is_stable);

CREATE TABLE ros_package_dependencies (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    package_id UUID NOT NULL REFERENCES ros_packages(id) ON DELETE CASCADE,
    dependency_package_id UUID NOT NULL REFERENCES ros_packages(id) ON DELETE CASCADE,
    dependency_type VARCHAR(50) NOT NULL CHECK (dependency_type IN ('build', 'runtime', 'test')),
    version_constraint VARCHAR(100),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(package_id, dependency_package_id, dependency_type)
);
CREATE INDEX idx_ros_package_dependencies_package_id ON ros_package_dependencies(package_id);
CREATE INDEX idx_ros_package_dependencies_dependency_package_id ON ros_package_dependencies(dependency_package_id);
CREATE INDEX idx_ros_package_dependencies_type ON ros_package_dependencies(dependency_type);

-- MODULES
CREATE TABLE modules (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name VARCHAR(255) NOT NULL UNIQUE,
    version VARCHAR(50) NOT NULL DEFAULT '1.0.0',
    description TEXT,
    category VARCHAR(100),
    type VARCHAR(50) NOT NULL DEFAULT 'core' CHECK (type IN ('core', 'advanced', 'custom')),
    maintainer_email VARCHAR(255),
    license VARCHAR(100) DEFAULT 'Apache-2.0',
    packages JSONB DEFAULT '[]',
    dependencies JSONB DEFAULT '[]',
    is_active BOOLEAN DEFAULT TRUE,
    is_public BOOLEAN DEFAULT TRUE,
    is_default BOOLEAN DEFAULT FALSE,
    tags JSONB DEFAULT '[]',
    algorithms JSONB DEFAULT '[]',
    source_path VARCHAR(500),
    config_path VARCHAR(500),
    supported_robots TEXT[] DEFAULT '{}'::text[],
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_modules_name ON modules(name);
CREATE INDEX idx_modules_category ON modules(category);
CREATE INDEX idx_modules_type ON modules(type);
CREATE INDEX idx_modules_is_active ON modules(is_active);
CREATE INDEX idx_modules_is_default ON modules(is_default);
CREATE INDEX idx_modules_created_at ON modules(created_at);
CREATE INDEX idx_modules_supported_robots ON modules USING GIN (supported_robots);
CREATE TRIGGER update_modules_updated_at BEFORE UPDATE ON modules
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TABLE module_dependencies (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    module_id UUID NOT NULL REFERENCES modules(id) ON DELETE CASCADE,
    dependency_module_id UUID NOT NULL REFERENCES modules(id) ON DELETE CASCADE,
    dependency_type VARCHAR(50) NOT NULL DEFAULT 'required' CHECK (dependency_type IN ('required', 'optional', 'conflicts')),
    version_constraint VARCHAR(100),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(module_id, dependency_module_id)
);
CREATE INDEX idx_module_dependencies_module_id ON module_dependencies(module_id);
CREATE INDEX idx_module_dependencies_dependency_module_id ON module_dependencies(dependency_module_id);
CREATE INDEX idx_module_dependencies_type ON module_dependencies(dependency_type);

CREATE TABLE module_packages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    module_id UUID NOT NULL REFERENCES modules(id) ON DELETE CASCADE,
    package_id UUID NOT NULL REFERENCES ros_packages(id) ON DELETE CASCADE,
    is_required BOOLEAN DEFAULT TRUE,
    order_index INTEGER DEFAULT 0,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(module_id, package_id)
);
CREATE INDEX idx_module_packages_module_id ON module_packages(module_id);
CREATE INDEX idx_module_packages_package_id ON module_packages(package_id);
CREATE INDEX idx_module_packages_order_index ON module_packages(order_index);

-- PROJECT <-> MODULE / PACKAGE RELATIONS
CREATE TABLE project_module_dependencies (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    project_id UUID NOT NULL REFERENCES projects(id) ON DELETE CASCADE,
    module_id UUID NOT NULL REFERENCES modules(id) ON DELETE CASCADE,
    dependency_type VARCHAR(50) NOT NULL DEFAULT 'required',
    version_constraint VARCHAR(100),
    order_index INTEGER DEFAULT 0,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(project_id, module_id)
);
CREATE INDEX idx_project_module_dependencies_project_id ON project_module_dependencies(project_id);
CREATE INDEX idx_project_module_dependencies_module_id ON project_module_dependencies(module_id);
CREATE INDEX idx_project_module_dependencies_type ON project_module_dependencies(dependency_type);
CREATE INDEX idx_project_module_dependencies_order ON project_module_dependencies(order_index);

CREATE TABLE project_packages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    project_id UUID NOT NULL REFERENCES projects(id) ON DELETE CASCADE,
    package_id UUID NOT NULL REFERENCES ros_packages(id) ON DELETE CASCADE,
    is_required BOOLEAN DEFAULT TRUE,
    order_index INTEGER DEFAULT 0,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(project_id, package_id)
);
CREATE INDEX idx_project_packages_project_id ON project_packages(project_id);
CREATE INDEX idx_project_packages_package_id ON project_packages(package_id);
CREATE INDEX idx_project_packages_order ON project_packages(order_index);

-- PROJECT FILES
CREATE TABLE project_files (
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
);
CREATE INDEX idx_project_files_project_id ON project_files(project_id);
CREATE INDEX idx_project_files_file_type ON project_files(file_type);
CREATE INDEX idx_project_files_path ON project_files(file_path);
CREATE TRIGGER update_project_files_updated_at BEFORE UPDATE ON project_files
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- DOWN

-- Drop triggers
DROP TRIGGER IF EXISTS update_project_files_updated_at ON project_files;
DROP TRIGGER IF EXISTS update_modules_updated_at ON modules;
DROP TRIGGER IF EXISTS update_ros_packages_updated_at ON ros_packages;
DROP TRIGGER IF EXISTS update_container_states_updated_at ON container_states;
DROP TRIGGER IF EXISTS update_project_configurations_updated_at ON project_configurations;
DROP TRIGGER IF EXISTS update_projects_updated_at ON projects;
DROP TRIGGER IF EXISTS update_users_updated_at ON users;

-- Drop indexes
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
DROP INDEX IF EXISTS idx_module_packages_order_index;
DROP INDEX IF EXISTS idx_module_packages_package_id;
DROP INDEX IF EXISTS idx_module_packages_module_id;
DROP INDEX IF EXISTS idx_module_dependencies_type;
DROP INDEX IF EXISTS idx_module_dependencies_dependency_module_id;
DROP INDEX IF EXISTS idx_module_dependencies_module_id;
DROP INDEX IF EXISTS idx_modules_supported_robots;
DROP INDEX IF EXISTS idx_modules_created_at;
DROP INDEX IF EXISTS idx_modules_is_default;
DROP INDEX IF EXISTS idx_modules_is_active;
DROP INDEX IF EXISTS idx_modules_type;
DROP INDEX IF EXISTS idx_modules_category;
DROP INDEX IF EXISTS idx_modules_name;
DROP INDEX IF EXISTS idx_ros_package_dependencies_type;
DROP INDEX IF EXISTS idx_ros_package_dependencies_dependency_package_id;
DROP INDEX IF EXISTS idx_ros_package_dependencies_package_id;
DROP INDEX IF EXISTS idx_ros_package_versions_is_stable;
DROP INDEX IF EXISTS idx_ros_package_versions_version;
DROP INDEX IF EXISTS idx_ros_package_versions_package_id;
DROP INDEX IF EXISTS idx_ros_packages_supported_robots;
DROP INDEX IF EXISTS idx_ros_packages_created_at;
DROP INDEX IF EXISTS idx_ros_packages_is_active;
DROP INDEX IF EXISTS idx_ros_packages_type;
DROP INDEX IF EXISTS idx_ros_packages_category;
DROP INDEX IF EXISTS idx_ros_packages_name;
DROP INDEX IF EXISTS idx_admin_audit_trails_created_at;
DROP INDEX IF EXISTS idx_admin_audit_trails_admin_user_id;
DROP INDEX IF EXISTS idx_container_states_created_at;
DROP INDEX IF EXISTS idx_container_states_status;
DROP INDEX IF EXISTS idx_container_states_project_id;
DROP INDEX IF EXISTS idx_user_activity_logs_created_at;
DROP INDEX IF EXISTS idx_user_activity_logs_project_id;
DROP INDEX IF EXISTS idx_user_activity_logs_user_id;
DROP INDEX IF EXISTS idx_project_configurations_category;
DROP INDEX IF EXISTS idx_project_configurations_type;
DROP INDEX IF EXISTS idx_project_configurations_project_id;
DROP INDEX IF EXISTS idx_projects_tags;
DROP INDEX IF EXISTS idx_projects_created_by;
DROP INDEX IF EXISTS idx_projects_is_active;
DROP INDEX IF EXISTS idx_projects_type;
DROP INDEX IF EXISTS idx_projects_name;
DROP INDEX IF EXISTS idx_projects_created_at;
DROP INDEX IF EXISTS idx_projects_owner_id;
DROP INDEX IF EXISTS idx_sessions_expires_at;
DROP INDEX IF EXISTS idx_sessions_user_id;
DROP INDEX IF EXISTS idx_project_members_user_id;
DROP INDEX IF EXISTS idx_project_members_project_id;
DROP INDEX IF EXISTS idx_users_username;
DROP INDEX IF EXISTS idx_users_email;

-- Drop tables (in reverse order due to foreign key constraints)
DROP TABLE IF EXISTS project_files;
DROP TABLE IF EXISTS project_packages;
DROP TABLE IF EXISTS project_module_dependencies;
DROP TABLE IF EXISTS module_packages;
DROP TABLE IF EXISTS module_dependencies;
DROP TABLE IF EXISTS modules;
DROP TABLE IF EXISTS ros_package_dependencies;
DROP TABLE IF EXISTS ros_package_versions;
DROP TABLE IF EXISTS ros_packages;
DROP TABLE IF EXISTS admin_audit_trails;
DROP TABLE IF EXISTS container_states;
DROP TABLE IF EXISTS user_activity_logs;
DROP TABLE IF EXISTS project_configurations;
DROP TABLE IF EXISTS project_members;
DROP TABLE IF EXISTS sessions;
DROP TABLE IF EXISTS projects;
DROP TABLE IF EXISTS users;

-- Drop function
DROP FUNCTION IF EXISTS update_updated_at_column();