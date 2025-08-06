# Robium Shared Package

This package contains shared schemas, types, and utilities for the Robium platform.

## Module-Based Architecture

The project now uses a modular architecture where:

### 1. **Module Metadata** (`modules/`)
- Each module has its own metadata file (e.g., `navigation_core.json`)
- Contains module information: name, description, version, category, packages, dependencies, tags
- Used for discovery, validation, and composition

### 2. **Project Configuration** (`templates/`)
- Projects reference modules by name in the `modules` array
- Simple composition: just list the modules you want to include
- Platform resolves module dependencies and copies required ROS packages

### 3. **ROS Packages** (`ros/src/`)
- Actual ROS packages are stored in the ROS workspace
- Organized by module directories (e.g., `ros/src/navigation_core/`)
- Each module directory contains multiple ROS packages

## Directory Structure

```
packages/shared/
├── modules/                                # Module metadata files
│   ├── navigation_core.json
│   ├── sensor_fusion.json
│   ├── perception_basic.json
│   └── manipulation_core.json
├── schemas/                                # Schema definitions
│   ├── project-config.schema.json         # Project configuration schema
│   ├── project-metadata.schema.json       # Project metadata schema
│   └── module-metadata.schema.json        # Module metadata schema
├── templates/                              # Project templates
│   ├── navigation-project.json
│   ├── manipulation-project.json
│   ├── perception-project.json
│   └── custom-project.json
├── types/                                  # TypeScript type definitions
│   ├── project-config.ts
│   ├── project-metadata.ts
│   └── module-metadata.ts
├── utils/                                  # Utilities
│   └── module-manager.ts                   # Module management utilities
├── validation/                             # Validation logic
│   └── config-validator.ts                 # Configuration validation
├── __tests__/                              # Tests
├── index.ts                                # Main entry point
└── run-tests.ts                            # Test runner

ros/
├── src/                                    # ROS workspace
│   ├── navigation_core/                    # Module directory
│   │   ├── nav2_bringup/                   # ROS package
│   │   ├── nav2_controller/                # ROS package
│   │   └── nav2_planner/                   # ROS package
│   ├── sensor_fusion/                      # Module directory
│   │   ├── robot_localization/             # ROS package
│   │   └── camera_lidar_fusion/            # ROS package
│   ├── perception_basic/                   # Module directory
│   │   ├── vision_msgs/                    # ROS package
│   │   └── object_detection/               # ROS package
│   └── manipulation_core/                  # Module directory
│       ├── moveit_core/                    # ROS package
│       └── gripper_control/                # ROS package
```

## Module Metadata Schema

```json
{
  "name": "navigation_core",
  "description": "Core navigation functionality including path planning, localization, and control",
  "version": "1.0.0",
  "category": "navigation",
  "packages": [
    "nav2_bringup",
    "nav2_controller",
    "nav2_planner"
  ],
  "dependencies": [],
  "tags": ["navigation", "path-planning", "localization"]
}
```

## Project Configuration Schema

```json
{
  "metadata": {
    "name": "my-navigation-project"
  },
  "modules": [
    "navigation_core",
    "sensor_fusion"
  ]
}
```

## Available Modules

### Navigation
- **navigation_core**: Core navigation functionality (path planning, localization, control)

### Perception
- **sensor_fusion**: Sensor fusion and state estimation
- **perception_basic**: Basic perception capabilities (object detection, vision)

### Manipulation
- **manipulation_core**: Core manipulation functionality (arm control, gripper)

## Usage

### 1. Creating a New Project

```typescript
import { ModuleManager } from './utils/module-manager';

const moduleManager = new ModuleManager();

// Get available modules
const modules = await moduleManager.getAllModules();

// Validate project configuration
const projectConfig = {
  metadata: { name: "my-project" },
  modules: ["navigation_core", "sensor_fusion"]
};

const { valid, invalid } = await moduleManager.validateModuleReferences(projectConfig.modules);
```

### 2. Getting Packages from Modules

```typescript
// Get all ROS packages needed for a project
const packages = await moduleManager.getPackagesFromModules([
  "navigation_core",
  "sensor_fusion"
]);
// Returns: ["nav2_bringup", "nav2_controller", "robot_localization", ...]
```

### 3. Module Discovery

```typescript
// Get modules by category
const navigationModules = await moduleManager.getModulesByCategory('navigation');

// Get modules by tags
const perceptionModules = await moduleManager.getModulesByTags(['vision', 'object-detection']);
```

## Benefits

1. **Clean Separation**: Metadata vs implementation
2. **Easy Composition**: Just reference module names
3. **Discoverable**: Browse and search modules by category/tags
4. **Maintainable**: Modules can be versioned independently
5. **Flexible**: Mix and match modules for different project types

## Testing

Run the test suite:

```bash
npm test
```

This will validate:
- Schema validation for all project templates
- Module manager functionality
- Module discovery and validation 