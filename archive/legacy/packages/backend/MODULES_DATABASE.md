# Modules Database Implementation

This document describes the implementation of modules storage in the Robium database system, replacing the previous JSON file-based approach.

## 📋 Overview

The modules are now stored in the database with comprehensive metadata, dependencies, and package relationships, making them easily searchable and manageable through the API.

## 🗄️ Database Schema

### Tables Created

#### 1. `modules` - Main modules table

- **Primary Key**: `id` (UUID)
- **Unique Constraint**: `name` (VARCHAR(255))
- **Key Fields**:
  - `name`: Module name (e.g., "localization", "navigation")
  - `version`: Module version (e.g., "1.0.0")
  - `description`: Module description
  - `category`: Module category (e.g., "localization", "navigation", "perception")
  - `type`: Module type ("core", "advanced", "custom")
  - `maintainer_email`: Maintainer contact
  - `license`: License information
  - `packages`: JSON array of package names within this module
  - `dependencies`: JSON array of dependency module names
  - `tags`: JSON array of tags
  - `algorithms`: JSON array of algorithms
  - `source_path`: Path to module source files
  - `config_path`: Path to module configuration
  - `is_active`: Boolean flag for active modules
  - `is_public`: Boolean flag for public modules
  - `is_default`: Boolean flag for default modules

#### 2. `module_dependencies` - Module dependency relationships

- **Primary Key**: `id` (UUID)
- **Foreign Keys**: `module_id` → `modules.id`, `dependency_module_id` → `modules.id`
- **Key Fields**:
  - `dependency_type`: Type of dependency ("required", "optional", "conflicts")
  - `version_constraint`: Version constraint (e.g., ">=1.0.0")

#### 3. `module_packages` - Module-package relationships

- **Primary Key**: `id` (UUID)
- **Foreign Keys**: `module_id` → `modules.id`, `package_id` → `ros_packages.id`
- **Key Fields**:
  - `is_required`: Boolean flag for required packages
  - `order_index`: Integer for ordering packages within module

## 📦 Modules Stored

### 1. **localization**

- **Category**: localization
- **Type**: core
- **Packages**: amcl_localization, kalman_filter
- **Dependencies**: None
- **Algorithms**: amcl, kalman-filter, tf2, rviz
- **Tags**: localization, positioning, state-estimation, amcl, kalman-filter

### 2. **navigation**

- **Category**: navigation
- **Type**: core
- **Packages**: amcl_localization, path_planning
- **Dependencies**: localization (required)
- **Algorithms**: nav2, amcl, tf2, rviz
- **Tags**: navigation, autonomous, path-planning, motion-planning

### 3. **person_tracking**

- **Category**: perception
- **Type**: advanced
- **Packages**: person_tracking
- **Dependencies**: None
- **Algorithms**: opencv, pcl, rviz
- **Tags**: perception, person-detection, tracking, computer-vision

## 🔌 API Endpoints

### Base URL: `/modules`

#### 1. **GET /** - List all modules

- **Query Parameters**:
  - `category`: Filter by category
  - `type`: Filter by type
  - `search`: Search in name and description
- **Response**: Array of modules with package and dependency counts

#### 2. **GET /categories** - Get all categories

- **Response**: Array of unique category names
- **Example**: `["localization", "navigation", "perception"]`

#### 3. **GET /types** - Get all types

- **Response**: Array of unique type names
- **Example**: `["advanced", "core"]`

#### 4. **GET /search** - Search modules

- **Query Parameters**:
  - `q`: Search query (required)
  - `category`: Filter by category
  - `type`: Filter by type
  - `limit`: Maximum results (default: 10)
- **Response**: Search results with count and query

#### 5. **GET /:id** - Get specific module

- **Path Parameter**: `id` (UUID)
- **Response**: Single module with full metadata, packages, dependencies, and dependents

#### 6. **GET /:id/packages** - Get packages for a specific module

- **Path Parameter**: `id` (UUID)
- **Response**: Array of packages with relationship metadata

#### 7. **GET /:id/dependencies** - Get dependencies for a specific module

- **Path Parameter**: `id` (UUID)
- **Response**: Array of dependency modules with relationship metadata

## 🛠️ Implementation Files

### Database Migration

- **File**: `src/migrations/004_modules_schema.sql`
- **Purpose**: Creates the database schema for modules

### Population Script

- **File**: `src/scripts/populate-modules.ts`
- **Command**: `npm run populate:modules`
- **Purpose**: Populates the database with the three specified modules

### API Routes

- **File**: `src/routes/modules.ts`
- **Purpose**: Provides REST API endpoints for modules

### Main Application

- **File**: `src/index.ts`
- **Integration**: Routes registered at `/modules`

## 🚀 Usage Examples

### List all modules

```bash
curl -X GET http://localhost:8000/modules
```

### Get categories

```bash
curl -X GET http://localhost:8000/modules/categories
```

### Get types

```bash
curl -X GET http://localhost:8000/modules/types
```

### Search modules

```bash
curl -X GET "http://localhost:8000/modules/search?q=navigation"
```

### Get specific module

```bash
curl -X GET http://localhost:8000/modules/8c8828df-e73e-41e5-bafe-959eb3951218
```

### Get module packages

```bash
curl -X GET http://localhost:8000/modules/8c8828df-e73e-41e5-bafe-959eb3951218/packages
```

### Get module dependencies

```bash
curl -X GET http://localhost:8000/modules/8c8828df-e73e-41e5-bafe-959eb3951218/dependencies
```

### Filter by category

```bash
curl -X GET "http://localhost:8000/modules?category=navigation"
```

## 🔄 Integration with Robium

The modules database integrates with the existing Robium system:

1. **Project Creation**: Projects can reference modules by name
2. **Package System**: Modules are linked to ROS packages through relationships
3. **Dependency Resolution**: Module dependencies are properly tracked
4. **API Integration**: Frontend can query available modules and their relationships

## 📊 Current Status

- ✅ **Database Schema**: Created and migrated
- ✅ **Modules**: 3 modules populated with correct dependencies
- ✅ **API Endpoints**: All endpoints working
- ✅ **Search Functionality**: Full-text search implemented
- ✅ **Filtering**: Category and type filtering available
- ✅ **Relationships**: Module-package and module-dependency relationships established
- ✅ **Integration**: Connected to main application
- ✅ **Cleanup**: Old JSON files removed

## 🔗 Module Dependencies

The dependency graph is:

```
localization (no dependencies)
    ↑
navigation (depends on localization)
person_tracking (no dependencies)
```

## 🔗 Module-Package Relationships

- **localization**: amcl_localization, kalman_filter
- **navigation**: amcl_localization, path_planning
- **person_tracking**: person_tracking

## 🔮 Future Enhancements

1. **Version Management**: Implement module versioning
2. **Dependency Resolution**: Add dependency graph functionality
3. **Module Upload**: Allow users to upload custom modules
4. **Validation**: Add module validation and testing
5. **Analytics**: Track module usage and popularity
6. **Collaboration**: Allow module sharing and collaboration

## 🧪 Testing

All endpoints have been tested and are working correctly:

- ✅ List modules: Returns 3 modules with counts
- ✅ Categories: Returns 3 categories (localization, navigation, perception)
- ✅ Types: Returns 2 types (advanced, core)
- ✅ Search: Finds modules by name/description
- ✅ Individual module: Returns full module metadata with packages and dependencies
- ✅ Module packages: Returns packages for specific module
- ✅ Module dependencies: Returns dependencies for specific module

## 🗑️ Cleanup

The old JSON-based module system has been completely removed:

- ✅ **Files Removed**: All `packages/shared/modules/*.json` files deleted
- ✅ **Database Migration**: New schema replaces file-based approach
- ✅ **API Updated**: All endpoints now use database instead of file system
- ✅ **Dependencies**: Proper relationships established between modules and packages

The modules are now fully integrated into the Robium database system and ready for use in project creation and workspace generation.
