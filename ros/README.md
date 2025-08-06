# Robium ROS Workspace

This directory contains the global ROS workspace and Docker base image for the Robium robotics development platform.

## Directory Structure

```
ros/
├── Dockerfile              # Base Docker image with ROS 2 Humble
├── docker-compose.yml      # Local Docker registry setup
├── .dockerignore          # Files to exclude from Docker build
└── src/                   # ROS packages source directory
    └── hello_world/       # Example ROS package
        ├── package.xml    # Package metadata
        └── CMakeLists.txt # Build configuration
```

## Build Process

### Prerequisites

- Docker
- Docker Compose
- Node.js (for version extraction)

### Building the ROS Image

1. Start the local Docker registry:
   ```bash
   cd ros
   docker-compose up -d
   ```

2. Build and push the ROS image:
   ```bash
   chmod +x scripts/build-ros-image.sh
   ./scripts/build-ros-image.sh
   ```

The script will:
- Extract the version from `package.json`
- Build the Docker image with ROS 2 Humble
- Tag the image with the project version
- Push the image to the local registry

### Manual Build

To build manually:
```bash
cd ros
docker build -t localhost:5000/robium-ros:latest .
docker push localhost:5000/robium-ros:latest
```

## Usage

### Running the ROS Container

```bash
docker run -it --rm localhost:5000/robium-ros:latest
```

### Using the ROS Workspace

The workspace is located at `/home/ros/ros_ws` inside the container. The ROS environment is automatically sourced.

## Security Features

- Non-root user (`ros`) for container execution
- Minimal attack surface with unnecessary packages removed
- Cleaned apt cache to reduce image size

## Maintenance

### Adding New ROS Packages

1. Create a new directory in `ros/src/`
2. Add the package source code
3. Ensure `package.xml` and `CMakeLists.txt` are properly configured
4. Rebuild the Docker image

### Updating Dependencies

The `rosdep` system automatically resolves and installs ROS package dependencies. To update:

1. Modify package dependencies in `package.xml` files
2. Rebuild the Docker image

### Version Management

The image version is automatically extracted from the project's `package.json` file. To update the version:

1. Update the version in `package.json`
2. Rebuild the image using the build script

## Troubleshooting

### Build Failures

- Ensure the local Docker registry is running
- Check that all ROS packages have valid `package.xml` and `CMakeLists.txt` files
- Verify that dependencies are correctly specified

### Runtime Issues

- Check that the ROS environment is properly sourced
- Verify that the workspace is correctly built
- Ensure all required packages are installed

## Development

### Local Development

For local development without Docker:

1. Install ROS 2 Humble on your system
2. Clone the workspace to a local directory
3. Run `colcon build` to build the workspace
4. Source the workspace: `source install/setup.bash`

### Testing

To test the workspace:

1. Build the Docker image
2. Run the container
3. Verify that ROS commands work: `ros2 --help`
4. Test package functionality as needed 