# Development Scripts

This directory contains scripts to manage the Robium development environment.

## 🚀 Quick Start

### Start Development Environment
```bash
# From project root
npm run dev
# or
./scripts/start-dev.sh
```

### Stop All Servers
```bash
# From project root
npm run clean
# or
./scripts/cleanup.sh
```

### Kill Ports Only
```bash
npm run kill-ports
```

## 📋 Available Scripts

### `start-dev.sh`
- **Purpose:** Starts both backend and frontend servers
- **Features:**
  - Automatically kills existing processes on ports 3000, 3001, and 8000
  - Waits for ports to be available before starting
  - Starts backend first, then frontend
  - Provides status updates
  - Graceful shutdown with Ctrl+C
  - Prevents multiple instances

### `cleanup.sh`
- **Purpose:** Stops all development servers and clears ports
- **Features:**
  - Kills all npm processes
  - Stops ts-node-dev and react-scripts
  - Clears ports 3000, 3001, and 8000
  - Verifies ports are free
  - Safe to run multiple times

## 🔧 Memory Optimization

The frontend has been configured to prevent memory issues:

- **Disabled ForkTsCheckerWebpackPlugin** to prevent TypeScript memory crashes
- **Increased Node.js memory limit** to 4GB
- **Optimized webpack chunks** for better memory usage
- **Split Material-UI** into separate chunks

## 🎯 Port Usage

- **Frontend:** http://localhost:3000
- **Backend:** http://localhost:8000
- **WebSocket:** ws://localhost:8000/ws

## 🚨 Troubleshooting

### Port Already in Use
```bash
./scripts/cleanup.sh
npm run dev
```

### Memory Issues
The frontend is configured to handle memory better, but if you still encounter issues:
```bash
# Increase memory limit manually
cd packages/frontend
NODE_OPTIONS="--max-old-space-size=8192" npm start
```

### Multiple Instances
```bash
# Kill all processes and start fresh
./scripts/cleanup.sh
npm run dev
```

## 📝 Usage Examples

```bash
# Start development
npm run dev

# In another terminal, stop servers
npm run clean

# Check what's running on ports
lsof -i :3000 -i :8000

# Start only backend
cd packages/backend && npm run dev

# Start only frontend
cd packages/frontend && npm start
``` 