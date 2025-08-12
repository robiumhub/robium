# Robium - Robot Development Platform

A comprehensive platform for developing, managing, and deploying robot applications with Docker containerization.

## 🚀 Quick Start

### Prerequisites

- Node.js v18+
- PostgreSQL
- Docker (for containerization)
- **For ROS Development:**
  - **Option 1 (Local):** ROS 2 (Humble or later), vcstool, rosdep, colcon
  - **Option 2 (Containerized):** Docker and Docker Compose (recommended)

### Clone with Submodules

```bash
git clone --recursive git@github.com:mdemirst/robium.git
cd robium
```

### Development Environment

**Start the development servers:**

```bash
# From the project root directory
./scripts/start-dev.sh
```

**Stop all servers:**

```bash
# From the project root directory
./scripts/cleanup.sh
```

**Alternative commands:**

```bash
# Start servers
npm run dev

# Clean up processes
npm run clean

# Kill ports only
npm run kill-ports
```

### ROS Workspace Setup

#### Option 1: Local Development

```bash
# Set up ROS workspace
cd ros
./ros setup

# Build ROS packages
./ros build

# Source the workspace (optional)
source install/setup.bash
```

#### Option 2: Containerized Development (Recommended)

```bash
# Build and enter container
cd ros
docker compose up --build -d
docker compose exec ros-workspace bash

# Inside container, workspace is already set up and built
./ros help
./ros build
source install/setup.bash
```

### Access Your Application

- **Frontend:** http://localhost:3000
- **Backend API:** http://localhost:8000

## 📁 Project Structure

```
robium/
├── packages/
│   ├── backend/          # Node.js/Express API server
│   ├── frontend/         # React frontend application
│   └── shared/           # Shared utilities and types
├── scripts/              # Development and deployment scripts
├── docs/                 # Project documentation
└── ros/                  # ROS2 workspace (Git submodule)
    ├── meta/
    │   └── core.vcs.yaml # ROS package dependencies
    ├── scripts/
    │   └── bootstrap.sh  # Setup script
    ├── src/              # ROS packages (auto-generated)
    ├── ros               # Workspace management commands
    └── .gitignore        # Git ignore rules
```

## 🔧 Features

### ✅ Completed

- **User Authentication & Authorization**
- **Project Management** with rich configuration storage
- **Dockerfile Generation** based on project settings
- **Database Schema** with project configurations, user activity logs, container states
- **Frontend Interface** with project creation wizard
- **View Dockerfile** functionality

### 🚧 In Progress

- Container lifecycle management
- Advanced algorithm selection

### ✅ ROS Integration

- **ROS Workspace Management** via Git submodule
- **Dependency Management** with vcstool and rosdep
- **Build System** with colcon and Makefile
- **Package Version Control** with exact version pinning

## 🛠️ Development

### Prerequisites

- Node.js v18+
- PostgreSQL
- Docker (for containerization)

### Database Setup

The application automatically runs migrations on startup. The database schema includes:

- User management
- Project configurations
- Activity logging
- Container state tracking

### ROS Submodule Management

The ROS workspace is managed as a Git submodule. To update it:

```bash
# Update the submodule to latest version
cd ros && git pull && cd ..

# Commit the submodule update
git add ros
git commit -m "Update ros submodule"
```

To clone the repository with submodules:

```bash
git clone --recursive git@github.com:mdemirst/robium.git
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

### API Endpoints

- `POST /projects` - Create new project with configuration
- `GET /projects` - List user projects
- `GET /dockerfiles/:projectId` - View generated Dockerfile
- `POST /dockerfiles/:projectId/generate` - Generate new Dockerfile

## 📝 License

This project is licensed under the MIT License.
