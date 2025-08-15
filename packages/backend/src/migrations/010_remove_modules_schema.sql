-- UP
-- Remove module-related tables and references

-- Drop project_module_dependencies table (project-module relationships)
DROP TABLE IF EXISTS project_module_dependencies CASCADE;

-- Drop module_packages table (module-package relationships)
DROP TABLE IF EXISTS module_packages CASCADE;

-- Drop module_dependencies table (module-module relationships)
DROP TABLE IF EXISTS module_dependencies CASCADE;

-- Drop modules table (main modules table)
DROP TABLE IF EXISTS modules CASCADE;

-- Remove module-related indexes
DROP INDEX IF EXISTS idx_project_module_dependencies_project_id;
DROP INDEX IF EXISTS idx_project_module_dependencies_module_id;
DROP INDEX IF EXISTS idx_project_module_dependencies_type;
DROP INDEX IF EXISTS idx_project_module_dependencies_order;

-- DOWN
-- Recreate module-related tables (if needed to rollback)

-- Note: This migration removes the entire module system
-- Rolling back would require recreating the entire module schema
-- which is complex and not recommended
