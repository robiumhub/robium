#!/bin/bash

# Cleanup script to stop all development servers
# Usage: ./scripts/cleanup.sh

echo "🧹 Cleaning up development servers..."

# Kill all npm processes
echo "🔄 Stopping npm processes..."
pkill -f "npm run dev" 2>/dev/null || true
pkill -f "npm start" 2>/dev/null || true

# Kill specific development tools
echo "🔄 Stopping development tools..."
pkill -f "ts-node-dev" 2>/dev/null || true
pkill -f "react-scripts" 2>/dev/null || true

# Kill processes on specific ports
echo "🔄 Clearing ports..."
lsof -ti:3000 | xargs kill -9 2>/dev/null || true
lsof -ti:3001 | xargs kill -9 2>/dev/null || true
lsof -ti:8000 | xargs kill -9 2>/dev/null || true

# Wait a moment
sleep 1

# Verify ports are clear
echo "✅ Checking port status..."
if lsof -i:3000 >/dev/null 2>&1; then
    echo "⚠️  Port 3000 is still in use"
else
    echo "✅ Port 3000 is free"
fi

if lsof -i:8000 >/dev/null 2>&1; then
    echo "⚠️  Port 8000 is still in use"
else
    echo "✅ Port 8000 is free"
fi

echo "�� Cleanup complete!" 