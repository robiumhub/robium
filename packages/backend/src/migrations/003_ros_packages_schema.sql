-- UP
-- Create ros_packages table to store ROS package information
CREATE TABLE ros_packages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name VARCHAR(255) NOT NULL UNIQUE,
    version VARCHAR(50) NOT NULL DEFAULT '0.0.0',
    description TEXT,
    category VARCHAR(100),
    type VARCHAR(50) NOT NULL DEFAULT 'mock' CHECK (type IN ('mock', 'production', 'custom')),
    
    -- Package metadata
    maintainer_email VARCHAR(255),
    license VARCHAR(100) DEFAULT 'Apache-2.0',
    
    -- Dependencies (stored as JSON arrays)
    build_dependencies JSONB DEFAULT '[]',
    runtime_dependencies JSONB DEFAULT '[]',
    test_dependencies JSONB DEFAULT '[]',
    
    -- Package configuration
    build_type VARCHAR(50) DEFAULT 'ament_cmake',
    packages JSONB DEFAULT '[]', -- Array of package names within this module
    
    -- Topics and services
    published_topics JSONB DEFAULT '[]',
    subscribed_topics JSONB DEFAULT '[]',
    advertised_services JSONB DEFAULT '[]',
    called_services JSONB DEFAULT '[]',
    
    -- Tags and metadata
    tags JSONB DEFAULT '[]',
    algorithms JSONB DEFAULT '[]',
    
    -- File system information
    source_path VARCHAR(500), -- Path to package source files
    package_xml_path VARCHAR(500), -- Path to package.xml
    cmake_lists_path VARCHAR(500), -- Path to CMakeLists.txt
    
    -- Status and availability
    is_active BOOLEAN DEFAULT TRUE,
    is_public BOOLEAN DEFAULT TRUE,
    
    -- Metadata
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create ros_package_versions table for version history
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

-- Create ros_package_dependencies table for complex dependency relationships
CREATE TABLE ros_package_dependencies (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    package_id UUID NOT NULL REFERENCES ros_packages(id) ON DELETE CASCADE,
    dependency_package_id UUID NOT NULL REFERENCES ros_packages(id) ON DELETE CASCADE,
    dependency_type VARCHAR(50) NOT NULL CHECK (dependency_type IN ('build', 'runtime', 'test')),
    version_constraint VARCHAR(100), -- e.g., ">=1.0.0", "~1.2.0"
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(package_id, dependency_package_id, dependency_type)
);

-- Create indexes for better query performance
CREATE INDEX idx_ros_packages_name ON ros_packages(name);
CREATE INDEX idx_ros_packages_category ON ros_packages(category);
CREATE INDEX idx_ros_packages_type ON ros_packages(type);
CREATE INDEX idx_ros_packages_is_active ON ros_packages(is_active);
CREATE INDEX idx_ros_packages_created_at ON ros_packages(created_at);

CREATE INDEX idx_ros_package_versions_package_id ON ros_package_versions(package_id);
CREATE INDEX idx_ros_package_versions_version ON ros_package_versions(version);
CREATE INDEX idx_ros_package_versions_is_stable ON ros_package_versions(is_stable);

CREATE INDEX idx_ros_package_dependencies_package_id ON ros_package_dependencies(package_id);
CREATE INDEX idx_ros_package_dependencies_dependency_package_id ON ros_package_dependencies(dependency_package_id);
CREATE INDEX idx_ros_package_dependencies_type ON ros_package_dependencies(dependency_type);

-- Create triggers for automatic timestamp updates
CREATE TRIGGER update_ros_packages_updated_at BEFORE UPDATE ON ros_packages
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- DOWN
DROP TRIGGER IF EXISTS update_ros_packages_updated_at ON ros_packages;
DROP INDEX IF EXISTS idx_ros_package_dependencies_type;
DROP INDEX IF EXISTS idx_ros_package_dependencies_dependency_package_id;
DROP INDEX IF EXISTS idx_ros_package_dependencies_package_id;
DROP INDEX IF EXISTS idx_ros_package_versions_is_stable;
DROP INDEX IF EXISTS idx_ros_package_versions_version;
DROP INDEX IF EXISTS idx_ros_package_versions_package_id;
DROP INDEX IF EXISTS idx_ros_packages_created_at;
DROP INDEX IF EXISTS idx_ros_packages_is_active;
DROP INDEX IF EXISTS idx_ros_packages_type;
DROP INDEX IF EXISTS idx_ros_packages_category;
DROP INDEX IF EXISTS idx_ros_packages_name;
DROP TABLE IF EXISTS ros_package_dependencies;
DROP TABLE IF EXISTS ros_package_versions;
DROP TABLE IF EXISTS ros_packages; 