-- UP
-- SQLite-compatible initial schema

-- USERS
CREATE TABLE users (
    id TEXT PRIMARY KEY,
    email TEXT UNIQUE NOT NULL,
    username TEXT UNIQUE NOT NULL,
    password_hash TEXT NOT NULL,
    is_active INTEGER DEFAULT 1,
    role TEXT NOT NULL DEFAULT 'user' CHECK (role IN ('admin', 'user')),
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_username ON users(username);

-- PROJECTS
CREATE TABLE projects (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    description TEXT,
    owner_id TEXT NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    -- Extended fields
    version TEXT DEFAULT '1.0.0',
    author TEXT,
    maintainer_email TEXT,
    license TEXT DEFAULT 'Apache-2.0',
    type TEXT DEFAULT 'custom',
    is_active INTEGER DEFAULT 1,
    is_template INTEGER DEFAULT 0,
    config TEXT DEFAULT '{}',
    metadata TEXT DEFAULT '{}',
    workspace_path TEXT,
    source_path TEXT,
    config_path TEXT,
    created_by TEXT REFERENCES users(id) ON DELETE SET NULL,
    updated_by TEXT REFERENCES users(id) ON DELETE SET NULL,
    tags TEXT DEFAULT '[]',
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_projects_owner_id ON projects(owner_id);
CREATE INDEX idx_projects_created_at ON projects(created_at);
CREATE INDEX idx_projects_name ON projects(name);
CREATE INDEX idx_projects_type ON projects(type);
CREATE INDEX idx_projects_is_active ON projects(is_active);
CREATE INDEX idx_projects_created_by ON projects(created_by);

-- SESSIONS (JWT/refresh)
CREATE TABLE sessions (
    id TEXT PRIMARY KEY,
    user_id TEXT NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token_hash TEXT NOT NULL,
    refresh_token_hash TEXT,
    expires_at DATETIME NOT NULL,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    last_used_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    user_agent TEXT,
    ip_address TEXT
);
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_expires_at ON sessions(expires_at);

-- PROJECT MEMBERS
CREATE TABLE project_members (
    id TEXT PRIMARY KEY,
    project_id TEXT NOT NULL REFERENCES projects(id) ON DELETE CASCADE,
    user_id TEXT NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    role TEXT NOT NULL DEFAULT 'member' CHECK (role IN ('owner', 'admin', 'member', 'viewer')),
    joined_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(project_id, user_id)
);
CREATE INDEX idx_project_members_project_id ON project_members(project_id);
CREATE INDEX idx_project_members_user_id ON project_members(user_id);

-- PROJECT CONFIGURATIONS
CREATE TABLE project_configurations (
    id TEXT PRIMARY KEY,
    project_id TEXT NOT NULL REFERENCES projects(id) ON DELETE CASCADE,
    type TEXT NOT NULL DEFAULT 'python' CHECK (type IN ('python', 'nodejs', 'java', 'golang', 'rust', 'cpp', 'custom')),
    version TEXT NOT NULL DEFAULT '1.0.0',
    base_image TEXT NOT NULL DEFAULT 'python:3.11-slim',
    workdir TEXT NOT NULL DEFAULT '/app',
    system_dependencies TEXT DEFAULT '[]',
    python_dependencies TEXT DEFAULT '[]',
    node_dependencies TEXT DEFAULT '[]',
    java_dependencies TEXT DEFAULT '[]',
    golang_dependencies TEXT DEFAULT '[]',
    rust_dependencies TEXT DEFAULT '[]',
    cpp_dependencies TEXT DEFAULT '[]',
    custom_dependencies TEXT DEFAULT '[]',
    environment_variables TEXT DEFAULT '{}',
    ports TEXT DEFAULT '[]',
    volumes TEXT DEFAULT '[]',
    command TEXT,
    entrypoint TEXT,
    runtime_user TEXT,
    health_check_command TEXT,
    health_check_interval TEXT,
    health_check_timeout TEXT,
    health_check_retries INTEGER DEFAULT 3,
    health_check_start_period TEXT,
    build_args TEXT DEFAULT '{}',
    labels TEXT DEFAULT '{}',
    multi_stage INTEGER DEFAULT 0,
    stages TEXT DEFAULT '[]',
    category TEXT,
    tags TEXT DEFAULT '[]',
    is_public INTEGER DEFAULT 0,
    algorithms TEXT DEFAULT '[]',
    max_memory INTEGER DEFAULT 2048,
    cpu_limit INTEGER DEFAULT 2,
    enable_gpu INTEGER DEFAULT 0,
    auto_save INTEGER DEFAULT 1,
    enable_debugging INTEGER DEFAULT 0,
    enable_logging INTEGER DEFAULT 1,
    backup_frequency TEXT DEFAULT 'weekly',
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    updated_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(project_id)
);
CREATE INDEX idx_project_configurations_project_id ON project_configurations(project_id);
CREATE INDEX idx_project_configurations_type ON project_configurations(type);
CREATE INDEX idx_project_configurations_category ON project_configurations(category);

-- USER ACTIVITY LOGS
CREATE TABLE user_activity_logs (
    id TEXT PRIMARY KEY,
    user_id TEXT NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    project_id TEXT REFERENCES projects(id) ON DELETE SET NULL,
    action TEXT NOT NULL,
    details TEXT DEFAULT '{}',
    ip_address TEXT,
    user_agent TEXT,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_user_activity_logs_user_id ON user_activity_logs(user_id);
CREATE INDEX idx_user_activity_logs_project_id ON user_activity_logs(project_id);
CREATE INDEX idx_user_activity_logs_created_at ON user_activity_logs(created_at);

-- CONTAINER STATES
CREATE TABLE container_states (
    id TEXT PRIMARY KEY,
    project_id TEXT NOT NULL REFERENCES projects(id) ON DELETE CASCADE,
    container_id TEXT,
    container_name TEXT NOT NULL,
    status TEXT NOT NULL DEFAULT 'stopped',
    image TEXT,
    ports TEXT DEFAULT '{}',
    volumes TEXT DEFAULT '[]',
    environment TEXT DEFAULT '[]',
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    updated_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    started_at DATETIME,
    stopped_at DATETIME
);
CREATE INDEX idx_container_states_project_id ON container_states(project_id);
CREATE INDEX idx_container_states_status ON container_states(status);
CREATE INDEX idx_container_states_created_at ON container_states(created_at);

-- ADMIN AUDIT TRAILS
CREATE TABLE admin_audit_trails (
    id TEXT PRIMARY KEY,
    admin_user_id TEXT NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    target_user_id TEXT REFERENCES users(id) ON DELETE SET NULL,
    target_project_id TEXT REFERENCES projects(id) ON DELETE SET NULL,
    action TEXT NOT NULL,
    details TEXT DEFAULT '{}',
    ip_address TEXT,
    user_agent TEXT,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_admin_audit_trails_admin_user_id ON admin_audit_trails(admin_user_id);
CREATE INDEX idx_admin_audit_trails_created_at ON admin_audit_trails(created_at);

-- ROS PACKAGES
CREATE TABLE ros_packages (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL UNIQUE,
    version TEXT NOT NULL DEFAULT '0.0.0',
    description TEXT,
    category TEXT,
    type TEXT NOT NULL DEFAULT 'mock' CHECK (type IN ('mock', 'production', 'custom')),
    maintainer_email TEXT,
    license TEXT DEFAULT 'Apache-2.0',
    build_dependencies TEXT DEFAULT '[]',
    runtime_dependencies TEXT DEFAULT '[]',
    test_dependencies TEXT DEFAULT '[]',
    build_type TEXT DEFAULT 'ament_cmake',
    packages TEXT DEFAULT '[]',
    published_topics TEXT DEFAULT '[]',
    subscribed_topics TEXT DEFAULT '[]',
    advertised_services TEXT DEFAULT '[]',
    called_services TEXT DEFAULT '[]',
    tags TEXT DEFAULT '[]',
    algorithms TEXT DEFAULT '[]',
    source_path TEXT,
    package_xml_path TEXT,
    cmake_lists_path TEXT,
    supported_robots TEXT DEFAULT '[]',
    is_active INTEGER DEFAULT 1,
    is_public INTEGER DEFAULT 1,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_ros_packages_name ON ros_packages(name);
CREATE INDEX idx_ros_packages_category ON ros_packages(category);
CREATE INDEX idx_ros_packages_type ON ros_packages(type);
CREATE INDEX idx_ros_packages_is_active ON ros_packages(is_active);
CREATE INDEX idx_ros_packages_created_at ON ros_packages(created_at);

-- ROS PACKAGE VERSIONS
CREATE TABLE ros_package_versions (
    id TEXT PRIMARY KEY,
    package_id TEXT NOT NULL REFERENCES ros_packages(id) ON DELETE CASCADE,
    version TEXT NOT NULL,
    changelog TEXT,
    release_date DATETIME,
    is_active INTEGER DEFAULT 1,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(package_id, version)
);
CREATE INDEX idx_ros_package_versions_package_id ON ros_package_versions(package_id);

-- ROBOTS
CREATE TABLE robots (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL UNIQUE,
    model TEXT NOT NULL,
    manufacturer TEXT,
    description TEXT,
    category TEXT,
    type TEXT NOT NULL DEFAULT 'mobile' CHECK (type IN ('mobile', 'manipulator', 'humanoid', 'aerial', 'underwater', 'custom')),
    capabilities TEXT DEFAULT '[]',
    sensors TEXT DEFAULT '[]',
    actuators TEXT DEFAULT '[]',
    dimensions TEXT DEFAULT '{}',
    weight REAL,
    max_speed REAL,
    battery_capacity INTEGER,
    supported_ros_versions TEXT DEFAULT '[]',
    supported_packages TEXT DEFAULT '[]',
    documentation_url TEXT,
    image_url TEXT,
    is_active INTEGER DEFAULT 1,
    is_public INTEGER DEFAULT 1,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_robots_name ON robots(name);
CREATE INDEX idx_robots_model ON robots(model);
CREATE INDEX idx_robots_category ON robots(category);
CREATE INDEX idx_robots_type ON robots(type);
CREATE INDEX idx_robots_is_active ON robots(is_active);

-- MODULES
CREATE TABLE modules (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL UNIQUE,
    category TEXT NOT NULL,
    description TEXT,
    version TEXT NOT NULL DEFAULT '1.0.0',
    type TEXT NOT NULL DEFAULT 'communication' CHECK (type IN ('communication', 'navigation', 'vision', 'manipulation', 'debug', 'gui', 'custom')),
    dependencies TEXT DEFAULT '[]',
    configuration TEXT DEFAULT '{}',
    is_active INTEGER DEFAULT 1,
    is_public INTEGER DEFAULT 1,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_modules_name ON modules(name);
CREATE INDEX idx_modules_category ON modules(category);
CREATE INDEX idx_modules_type ON modules(type);
CREATE INDEX idx_modules_is_active ON modules(is_active);

-- DOWN
DROP TABLE IF EXISTS modules;
DROP TABLE IF EXISTS robots;
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
