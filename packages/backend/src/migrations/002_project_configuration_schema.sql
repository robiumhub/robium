-- UP
-- Create project_configurations table to store rich project configuration data
CREATE TABLE project_configurations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    project_id UUID NOT NULL REFERENCES projects(id) ON DELETE CASCADE,
    
    -- Basic configuration
    type VARCHAR(20) NOT NULL DEFAULT 'python' CHECK (type IN ('python', 'nodejs', 'java', 'golang', 'rust', 'cpp', 'custom')),
    version VARCHAR(20) NOT NULL DEFAULT '1.0.0',
    base_image VARCHAR(255) NOT NULL DEFAULT 'python:3.11-slim',
    workdir VARCHAR(255) NOT NULL DEFAULT '/app',
    
    -- Dependencies (stored as JSON arrays)
    system_dependencies JSONB DEFAULT '[]',
    python_dependencies JSONB DEFAULT '[]',
    node_dependencies JSONB DEFAULT '[]',
    java_dependencies JSONB DEFAULT '[]',
    golang_dependencies JSONB DEFAULT '[]',
    rust_dependencies JSONB DEFAULT '[]',
    cpp_dependencies JSONB DEFAULT '[]',
    custom_dependencies JSONB DEFAULT '[]',
    
    -- Environment configuration
    environment_variables JSONB DEFAULT '{}',
    ports JSONB DEFAULT '[]',
    volumes JSONB DEFAULT '[]',
    
    -- Runtime configuration
    command TEXT,
    entrypoint TEXT,
    runtime_user VARCHAR(100),
    
    -- Health check configuration
    health_check_command TEXT,
    health_check_interval VARCHAR(20),
    health_check_timeout VARCHAR(20),
    health_check_retries INTEGER DEFAULT 3,
    health_check_start_period VARCHAR(20),
    
    -- Build configuration
    build_args JSONB DEFAULT '{}',
    labels JSONB DEFAULT '{}',
    multi_stage BOOLEAN DEFAULT FALSE,
    stages JSONB DEFAULT '[]',
    
    -- Project settings
    category VARCHAR(100),
    tags JSONB DEFAULT '[]',
    is_public BOOLEAN DEFAULT FALSE,
    algorithms JSONB DEFAULT '[]',
    
    -- Resource limits
    max_memory INTEGER DEFAULT 2048, -- MB
    cpu_limit INTEGER DEFAULT 2,
    enable_gpu BOOLEAN DEFAULT FALSE,
    
    -- Development settings
    auto_save BOOLEAN DEFAULT TRUE,
    enable_debugging BOOLEAN DEFAULT FALSE,
    enable_logging BOOLEAN DEFAULT TRUE,
    backup_frequency VARCHAR(20) DEFAULT 'weekly',
    
    -- Metadata
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    
    -- Ensure one configuration per project
    UNIQUE(project_id)
);

-- Create user_activity_logs table for tracking user actions
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

-- Create container_states table for tracking container lifecycle
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

-- Create admin_audit_trails table for admin actions
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

-- Create indexes for better query performance
CREATE INDEX idx_project_configurations_project_id ON project_configurations(project_id);
CREATE INDEX idx_project_configurations_type ON project_configurations(type);
CREATE INDEX idx_project_configurations_category ON project_configurations(category);
CREATE INDEX idx_user_activity_logs_user_id ON user_activity_logs(user_id);
CREATE INDEX idx_user_activity_logs_project_id ON user_activity_logs(project_id);
CREATE INDEX idx_user_activity_logs_created_at ON user_activity_logs(created_at);
CREATE INDEX idx_container_states_project_id ON container_states(project_id);
CREATE INDEX idx_container_states_status ON container_states(status);
CREATE INDEX idx_container_states_created_at ON container_states(created_at);
CREATE INDEX idx_admin_audit_trails_admin_user_id ON admin_audit_trails(admin_user_id);
CREATE INDEX idx_admin_audit_trails_created_at ON admin_audit_trails(created_at);

-- Create trigger for project_configurations updated_at
CREATE TRIGGER update_project_configurations_updated_at BEFORE UPDATE ON project_configurations
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- Create trigger for container_states updated_at
CREATE TRIGGER update_container_states_updated_at BEFORE UPDATE ON container_states
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- DOWN
-- Drop triggers
DROP TRIGGER IF EXISTS update_project_configurations_updated_at ON project_configurations;
DROP TRIGGER IF EXISTS update_container_states_updated_at ON container_states;

-- Drop indexes
DROP INDEX IF EXISTS idx_project_configurations_project_id;
DROP INDEX IF EXISTS idx_project_configurations_type;
DROP INDEX IF EXISTS idx_project_configurations_category;
DROP INDEX IF EXISTS idx_user_activity_logs_user_id;
DROP INDEX IF EXISTS idx_user_activity_logs_project_id;
DROP INDEX IF EXISTS idx_user_activity_logs_created_at;
DROP INDEX IF EXISTS idx_container_states_project_id;
DROP INDEX IF EXISTS idx_container_states_status;
DROP INDEX IF EXISTS idx_container_states_created_at;
DROP INDEX IF EXISTS idx_admin_audit_trails_admin_user_id;
DROP INDEX IF EXISTS idx_admin_audit_trails_created_at;

-- Drop tables (in reverse order due to foreign key constraints)
DROP TABLE IF EXISTS admin_audit_trails;
DROP TABLE IF EXISTS container_states;
DROP TABLE IF EXISTS user_activity_logs;
DROP TABLE IF EXISTS project_configurations; 