# ROS Packages Database Implementation

This document describes the implementation of ROS packages storage in the Robium database system.

## 📋 Overview

The ROS packages are now stored in the database with comprehensive metadata, making them easily searchable and manageable through the API.

## 🗄️ Database Schema

### Tables Created

#### 1. `ros_packages` - Main ROS packages table

- **Primary Key**: `id` (UUID)
- **Unique Constraint**: `name` (VARCHAR(255))
- **Key Fields**:
  - `name`: Package name (e.g., "amcl_localization")
  - `version`: Package version (e.g., "0.0.0")
  - `description`: Package description
  - `category`: Package category (e.g., "localization", "perception")
  - `type`: Package type ("mock", "production", "custom")
  - `maintainer_email`: Maintainer contact
  - `license`: License information
  - `build_dependencies`: JSON array of build dependencies
  - `runtime_dependencies`: JSON array of runtime dependencies
  - `test_dependencies`: JSON array of test dependencies
  - `build_type`: Build system (e.g., "ament_cmake")
  - `packages`: JSON array of package names within this module
  - `published_topics`: JSON array of published topics
  - `subscribed_topics`: JSON array of subscribed topics
  - `advertised_services`: JSON array of advertised services
  - `called_services`: JSON array of called services
  - `tags`: JSON array of tags
  - `algorithms`: JSON array of algorithms
  - `source_path`: Path to source files
  - `package_xml_path`: Path to package.xml
  - `cmake_lists_path`: Path to CMakeLists.txt
  - `is_active`: Boolean flag for active packages
  - `is_public`: Boolean flag for public packages

#### 2. `ros_package_versions` - Version history

- **Primary Key**: `id` (UUID)
- **Foreign Key**: `package_id` → `ros_packages.id`
- **Key Fields**:
  - `version`: Version string
  - `changelog`: Version changelog
  - `release_notes`: Release notes
  - `is_stable`: Boolean flag for stable versions

#### 3. `ros_package_dependencies` - Dependency relationships

- **Primary Key**: `id` (UUID)
- **Foreign Keys**: `package_id` → `ros_packages.id`, `dependency_package_id` → `ros_packages.id`
- **Key Fields**:
  - `dependency_type`: Type of dependency ("build", "runtime", "test")
  - `version_constraint`: Version constraint (e.g., ">=1.0.0")

## 📦 Mock Packages Stored

### 1. **amcl_localization**

- **Category**: localization
- **Type**: mock
- **Topics**: `/amcl_localization/package_info`, `/amcl_pose`
- **Algorithms**: amcl, tf2, rviz
- **Tags**: localization, amcl, monte-carlo, pose-estimation

### 2. **person_tracking**

- **Category**: perception
- **Type**: mock
- **Topics**: `/person_tracking/package_info`, `/person_detections`, `/person_poses`
- **Algorithms**: opencv, pcl, rviz
- **Tags**: perception, person-detection, tracking, computer-vision

### 3. **path_planning**

- **Category**: navigation
- **Type**: mock
- **Topics**: `/path_planning/package_info`, `/planned_path`, `/cmd_vel`
- **Algorithms**: nav2, tf2, rviz
- **Tags**: navigation, path-planning, autonomous, motion-planning

### 4. **kalman_filter**

- **Category**: estimation
- **Type**: mock
- **Topics**: `/kalman_filter/package_info`, `/filtered_odometry`, `/filtered_pose`
- **Algorithms**: kalman-filter, sensor-fusion, state-estimation
- **Tags**: estimation, kalman-filter, state-estimation, sensor-fusion

## 🔌 API Endpoints

### Base URL: `/ros-packages`

#### 1. **GET /** - List all ROS packages

- **Query Parameters**:
  - `category`: Filter by category
  - `type`: Filter by type
  - `search`: Search in name and description
- **Response**: Array of ROS packages with metadata

#### 2. **GET /categories** - Get all categories

- **Response**: Array of unique category names
- **Example**: `["estimation", "localization", "navigation", "perception"]`

#### 3. **GET /types** - Get all types

- **Response**: Array of unique type names
- **Example**: `["mock"]`

#### 4. **GET /search** - Search ROS packages

- **Query Parameters**:
  - `q`: Search query (required)
  - `category`: Filter by category
  - `type`: Filter by type
  - `limit`: Maximum results (default: 10)
- **Response**: Search results with count and query

#### 5. **GET /:id** - Get specific ROS package

- **Path Parameter**: `id` (UUID)
- **Response**: Single ROS package with full metadata

## 🛠️ Implementation Files

### Database Migration

- **File**: `src/migrations/003_ros_packages_schema.sql`
- **Purpose**: Creates the database schema for ROS packages

### Population Script

- **File**: `src/scripts/populate-ros-packages.ts`
- **Command**: `npm run populate:ros-packages`
- **Purpose**: Populates the database with mock ROS packages

### API Routes

- **File**: `src/routes/ros-packages.ts`
- **Purpose**: Provides REST API endpoints for ROS packages

### Main Application

- **File**: `src/index.ts`
- **Integration**: Routes registered at `/ros-packages`

## 🚀 Usage Examples

### List all packages

```bash
curl -X GET http://localhost:8000/ros-packages
```

### Get categories

```bash
curl -X GET http://localhost:8000/ros-packages/categories
```

### Search packages

```bash
curl -X GET "http://localhost:8000/ros-packages/search?q=localization"
```

### Get specific package

```bash
curl -X GET http://localhost:8000/ros-packages/acc07d73-8aee-43ff-b996-8ca128ca2c3c
```

### Filter by category

```bash
curl -X GET "http://localhost:8000/ros-packages?category=localization"
```

## 🔄 Integration with Robium

The ROS packages database integrates with the existing Robium system:

1. **Project Creation**: Projects can reference ROS packages by name
2. **Module System**: ROS packages are part of the module metadata system
3. **Workspace Generation**: Packages are copied to generated workspaces
4. **API Integration**: Frontend can query available packages

## 📊 Current Status

- ✅ **Database Schema**: Created and migrated
- ✅ **Mock Packages**: 4 packages populated
- ✅ **API Endpoints**: All endpoints working
- ✅ **Search Functionality**: Full-text search implemented
- ✅ **Filtering**: Category and type filtering available
- ✅ **Integration**: Connected to main application

## 🔮 Future Enhancements

1. **Version Management**: Implement package versioning
2. **Dependency Resolution**: Add dependency graph functionality
3. **Package Upload**: Allow users to upload custom packages
4. **Validation**: Add package validation and testing
5. **Analytics**: Track package usage and popularity
6. **Collaboration**: Allow package sharing and collaboration

## 🧪 Testing

All endpoints have been tested and are working correctly:

- ✅ List packages: Returns 4 mock packages
- ✅ Categories: Returns 4 categories
- ✅ Types: Returns 1 type ("mock")
- ✅ Search: Finds packages by name/description
- ✅ Individual package: Returns full package metadata

The ROS packages are now fully integrated into the Robium database system and ready for use in project creation and workspace generation.
