#!/bin/bash

# Fast development startup script
# Runs backend and database in Docker, frontend locally for better performance
# Usage: ./scripts/start-dev-fast.sh

set -e

echo "🚀 Starting Robium Fast Development Environment..."

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

# Cleanup function
cleanup() {
    echo "🧹 Cleaning up..."
    kill_port 3000
    docker compose down
    echo "✅ Cleanup complete"
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Start database and backend in Docker
echo "🗄️  Starting database and backend..."
docker compose up -d database backend

# Wait for database to be ready
echo "⏳ Waiting for database to be ready..."
sleep 5

# Check database status
if docker ps --filter "name=database" --format "{{.Status}}" | grep -q "Up"; then
    echo "✅ Database started successfully"
else
    echo "❌ Database failed to start"
    exit 1
fi

# Wait for backend to start
echo "⏳ Waiting for backend to start..."
sleep 5

# Check if backend is running
if ! lsof -i:8000 >/dev/null 2>&1; then
    echo "❌ Backend failed to start"
    exit 1
fi

# Test backend health
echo "🏥 Testing backend health..."
if curl -s http://localhost:8000/health >/dev/null 2>&1; then
    echo "✅ Backend is healthy"
else
    echo "⚠️  Backend started but health check failed (this might be normal during startup)"
fi

# Start frontend locally
echo "🎨 Starting frontend locally..."
cd packages/frontend
npm start &
FRONTEND_PID=$!
cd ../..

# Wait for frontend to start
echo "⏳ Waiting for frontend to start..."
sleep 10

# Check if frontend is running
if ! lsof -i:3000 >/dev/null 2>&1; then
    echo "❌ Frontend failed to start"
    exit 1
fi

# Test frontend
echo "🌐 Testing frontend..."
if curl -s -I http://localhost:3000 | head -1 | grep -q "200\|302"; then
    echo "✅ Frontend is responding"
else
    echo "⚠️  Frontend started but not responding yet (this might be normal during startup)"
fi

echo ""
echo "🎉 Fast development environment is ready!"
echo "📱 Frontend: http://localhost:3000 (running locally)"
echo "🔧 Backend:  http://localhost:8000 (running in Docker)"
echo "🗄️  Database: localhost:5432 (running in Docker)"
echo ""
echo "Press Ctrl+C to stop all servers"

# Function to cleanup on exit
cleanup_on_exit() {
    echo ""
    echo "🔄 Shutting down servers..."
    kill $FRONTEND_PID 2>/dev/null || true
    cleanup
    echo "✅ All services stopped"
    exit 0
}

# Set up signal handlers
trap cleanup_on_exit SIGINT SIGTERM

# Wait for user to stop
wait

