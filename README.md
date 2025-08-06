# Robium - Robot Development Platform

A comprehensive platform for developing, managing, and deploying robot applications with Docker containerization.

## 🚀 Quick Start

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
└── ros/                  # ROS2 packages and configurations
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
- ROS2 integration
- Advanced algorithm selection

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

### API Endpoints
- `POST /projects` - Create new project with configuration
- `GET /projects` - List user projects
- `GET /dockerfiles/:projectId` - View generated Dockerfile
- `POST /dockerfiles/:projectId/generate` - Generate new Dockerfile

## 📝 License

This project is licensed under the MIT License.
