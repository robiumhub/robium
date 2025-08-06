#!/bin/bash

# Development startup script that prevents multiple instances
# Usage: ./scripts/start-dev.sh

set -e

echo "🚀 Starting Robium Development Environment..."

# Function to kill processes on specific ports
kill_port() {
    local port=$1
    local pids=$(lsof -ti:$port 2>/dev/null || true)
    if [ ! -z "$pids" ]; then
        echo "🔄 Killing processes on port $port: $pids"
        kill -9 $pids 2>/dev/null || true
        sleep 1
    fi
}

# Function to check if port is free
check_port() {
    local port=$1
    if lsof -i:$port >/dev/null 2>&1; then
        echo "❌ Port $port is still in use. Please wait..."
        return 1
    fi
    return 0
}

# Clean up any existing processes
echo "🧹 Cleaning up existing processes..."
kill_port 3000
kill_port 3001
kill_port 8000

# Wait for ports to be free
echo "⏳ Waiting for ports to be available..."
while ! check_port 3000 || ! check_port 8000; do
    sleep 1
done

echo "✅ Ports are available"

# Start backend
echo "🔧 Starting backend server..."
cd packages/backend
npm run dev &
BACKEND_PID=$!
cd ../..

# Wait for backend to start
echo "⏳ Waiting for backend to start..."
sleep 3

# Check if backend is running
if ! lsof -i:8000 >/dev/null 2>&1; then
    echo "❌ Backend failed to start"
    exit 1
fi

echo "✅ Backend started successfully"

# Start frontend
echo "🎨 Starting frontend server..."
cd packages/frontend
npm start &
FRONTEND_PID=$!
cd ../..

# Wait for frontend to start
echo "⏳ Waiting for frontend to start..."
sleep 5

# Check if frontend is running
if ! lsof -i:3000 >/dev/null 2>&1; then
    echo "❌ Frontend failed to start"
    exit 1
fi

echo "✅ Frontend started successfully"

echo ""
echo "🎉 Development environment is ready!"
echo "📱 Frontend: http://localhost:3000"
echo "🔧 Backend:  http://localhost:8000"
echo ""
echo "Press Ctrl+C to stop all servers"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🔄 Shutting down servers..."
    kill $BACKEND_PID 2>/dev/null || true
    kill $FRONTEND_PID 2>/dev/null || true
    kill_port 3000
    kill_port 8000
    echo "✅ Servers stopped"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Wait for user to stop
wait 