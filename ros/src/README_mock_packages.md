# Mock ROS Packages for Robium

This directory contains mock ROS2 packages that provide minimal implementations for testing and development purposes. Each package compiles successfully and publishes topics about its functionality.

## 📦 Available Mock Packages

### 1. **amcl_localization**

- **Purpose**: Mock AMCL (Adaptive Monte Carlo Localization) package
- **Topics Published**:
  - `/amcl_localization/package_info` - Package description
  - `/amcl_pose` - Mock pose estimate with covariance
  - TF transforms: `map` → `base_link`

### 2. **person_tracking**

- **Purpose**: Mock person detection and tracking package
- **Topics Published**:
  - `/person_tracking/package_info` - Package description
  - `/person_detections` - Mock 3D person detections
  - `/person_poses` - Mock person pose array

### 3. **path_planning**

- **Purpose**: Mock path planning and navigation package
- **Topics Published**:
  - `/path_planning/package_info` - Package description
  - `/planned_path` - Mock navigation path
  - `/cmd_vel` - Mock velocity commands

### 4. **kalman_filter**

- **Purpose**: Mock Kalman filter for state estimation
- **Topics Published**:
  - `/kalman_filter/package_info` - Package description
  - `/filtered_odometry` - Mock filtered odometry
  - `/filtered_pose` - Mock filtered pose with covariance

## 🚀 Building and Running

### Build the packages:

```bash
cd ros/
colcon build --packages-select amcl_localization person_tracking path_planning kalman_filter
```

### Source the workspace:

```bash
source install/setup.bash
```

### Run all mock packages:

```bash
ros2 launch mock_packages_launch.py
```

### Run individual packages:

```bash
# AMCL Localization
ros2 run amcl_localization amcl_mock_node

# Person Tracking
ros2 run person_tracking person_tracking_mock_node

# Path Planning
ros2 run path_planning path_planning_mock_node

# Kalman Filter
ros2 run kalman_filter kalman_filter_mock_node
```

## 📊 Monitoring Topics

### List all topics:

```bash
ros2 topic list
```

### Monitor package info topics:

```bash
ros2 topic echo /amcl_localization/package_info
ros2 topic echo /person_tracking/package_info
ros2 topic echo /path_planning/package_info
ros2 topic echo /kalman_filter/package_info
```

### Monitor specific data topics:

```bash
# AMCL pose
ros2 topic echo /amcl_pose

# Person detections
ros2 topic echo /person_detections

# Planned path
ros2 topic echo /planned_path

# Filtered odometry
ros2 topic echo /filtered_odometry
```

## 🔧 Package Structure

Each mock package follows the standard ROS2 package structure:

```
package_name/
├── package.xml          # Package metadata and dependencies
├── CMakeLists.txt       # Build configuration
└── src/
    └── package_mock_node.cpp  # Mock implementation
```

## 🎯 Use Cases

These mock packages are useful for:

- **Testing**: Verify that your system can integrate with ROS2 packages
- **Development**: Provide placeholder implementations during development
- **Documentation**: Demonstrate expected topic interfaces
- **CI/CD**: Ensure build system works with ROS2 packages
- **Learning**: Understand ROS2 package structure and topic publishing

## 🔄 Integration with Robium

These packages can be referenced in Robium project configurations:

```json
{
  "modules": [
    "amcl_localization",
    "person_tracking",
    "path_planning",
    "kalman_filter"
  ]
}
```

The Robium system will automatically include these packages when creating ROS2 workspaces for projects that reference them.
