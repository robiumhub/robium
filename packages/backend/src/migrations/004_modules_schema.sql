-- UP
-- Create modules table to store module information
CREATE TABLE modules (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name VARCHAR(255) NOT NULL UNIQUE,
    version VARCHAR(50) NOT NULL DEFAULT '1.0.0',
    description TEXT,
    category VARCHAR(100),
    type VARCHAR(50) NOT NULL DEFAULT 'core' CHECK (type IN ('core', 'advanced', 'custom')),
    
    -- Module metadata
    maintainer_email VARCHAR(255),
    license VARCHAR(100) DEFAULT 'Apache-2.0',
    
    -- Module configuration
    packages JSONB DEFAULT '[]', -- Array of package names within this module
    dependencies JSONB DEFAULT '[]', -- Array of dependency module names
    
    -- Module settings
    is_active BOOLEAN DEFAULT TRUE,
    is_public BOOLEAN DEFAULT TRUE,
    is_default BOOLEAN DEFAULT FALSE,
    
    -- Tags and metadata
    tags JSONB DEFAULT '[]',
    algorithms JSONB DEFAULT '[]',
    
    -- File system information
    source_path VARCHAR(500), -- Path to module source files
    config_path VARCHAR(500), -- Path to module configuration
    
    -- Metadata
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create module_dependencies table for complex dependency relationships
CREATE TABLE module_dependencies (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    module_id UUID NOT NULL REFERENCES modules(id) ON DELETE CASCADE,
    dependency_module_id UUID NOT NULL REFERENCES modules(id) ON DELETE CASCADE,
    dependency_type VARCHAR(50) NOT NULL DEFAULT 'required' CHECK (dependency_type IN ('required', 'optional', 'conflicts')),
    version_constraint VARCHAR(100), -- e.g., ">=1.0.0", "~1.2.0"
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(module_id, dependency_module_id)
);

-- Create module_packages table for module-package relationships
CREATE TABLE module_packages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    module_id UUID NOT NULL REFERENCES modules(id) ON DELETE CASCADE,
    package_id UUID NOT NULL REFERENCES ros_packages(id) ON DELETE CASCADE,
    is_required BOOLEAN DEFAULT TRUE,
    order_index INTEGER DEFAULT 0, -- For ordering packages within module
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(module_id, package_id)
);

-- Create indexes for better query performance
CREATE INDEX idx_modules_name ON modules(name);
CREATE INDEX idx_modules_category ON modules(category);
CREATE INDEX idx_modules_type ON modules(type);
CREATE INDEX idx_modules_is_active ON modules(is_active);
CREATE INDEX idx_modules_is_default ON modules(is_default);
CREATE INDEX idx_modules_created_at ON modules(created_at);

CREATE INDEX idx_module_dependencies_module_id ON module_dependencies(module_id);
CREATE INDEX idx_module_dependencies_dependency_module_id ON module_dependencies(dependency_module_id);
CREATE INDEX idx_module_dependencies_type ON module_dependencies(dependency_type);

CREATE INDEX idx_module_packages_module_id ON module_packages(module_id);
CREATE INDEX idx_module_packages_package_id ON module_packages(package_id);
CREATE INDEX idx_module_packages_order_index ON module_packages(order_index);

-- Create triggers for automatic timestamp updates
CREATE TRIGGER update_modules_updated_at BEFORE UPDATE ON modules
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- DOWN
DROP TRIGGER IF EXISTS update_modules_updated_at ON modules;
DROP INDEX IF EXISTS idx_module_packages_order_index;
DROP INDEX IF EXISTS idx_module_packages_package_id;
DROP INDEX IF EXISTS idx_module_packages_module_id;
DROP INDEX IF EXISTS idx_module_dependencies_type;
DROP INDEX IF EXISTS idx_module_dependencies_dependency_module_id;
DROP INDEX IF EXISTS idx_module_dependencies_module_id;
DROP INDEX IF EXISTS idx_modules_created_at;
DROP INDEX IF EXISTS idx_modules_is_default;
DROP INDEX IF EXISTS idx_modules_is_active;
DROP INDEX IF EXISTS idx_modules_type;
DROP INDEX IF EXISTS idx_modules_category;
DROP INDEX IF EXISTS idx_modules_name;
DROP TABLE IF EXISTS module_packages;
DROP TABLE IF EXISTS module_dependencies;
DROP TABLE IF EXISTS modules; 