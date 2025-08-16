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

# Comprehensive cleanup function
cleanup_all() {
    echo "🧹 Performing comprehensive cleanup..."
    
    # Kill all Node.js related processes
    echo "🔄 Stopping Node.js processes..."
    pkill -f "node\|npm\|ts-node\|react-scripts" 2>/dev/null || true
    
    # Stop and remove all Docker containers
    echo "🐳 Stopping Docker containers..."
    docker stop $(docker ps -aq) 2>/dev/null || true
    docker rm $(docker ps -aq) 2>/dev/null || true
    
    # Kill processes on specific ports
    echo "🔌 Clearing ports..."
    kill_port 3000
    kill_port 8000
    kill_port 5432
    
    # Stop Docker Compose services
    echo "📦 Stopping Docker Compose services..."
    docker-compose down -v 2>/dev/null || true
    docker compose down -v 2>/dev/null || true
    
    # Wait a moment for cleanup to complete
    sleep 2
    
    # Verify ports are clear
    echo "✅ Checking if ports are clear..."
    if lsof -i:3000,8000,5432 >/dev/null 2>&1; then
        echo "⚠️  Some ports are still in use, but continuing..."
    else
        echo "✅ All ports are clear"
    fi
}

# Perform comprehensive cleanup
cleanup_all

# Wait for ports to be free
echo "⏳ Waiting for ports to be available..."
while ! check_port 3000 || ! check_port 8000; do
    sleep 1
done

echo "✅ Ports are available"

# Start database first
echo "🗄️  Starting database..."
docker compose up -d database

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

# Start backend
echo "🔧 Starting backend server..."
cd packages/backend
npm run dev &
BACKEND_PID=$!
cd ../..

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

# Start frontend
echo "🎨 Starting frontend server..."
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
echo "🎉 Development environment is ready!"
echo "📱 Frontend: http://localhost:3000"
echo "🔧 Backend:  http://localhost:8000"
echo "🗄️  Database: localhost:5432"
echo ""
echo "Press Ctrl+C to stop all servers"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🔄 Shutting down servers..."
    kill $BACKEND_PID 2>/dev/null || true
    kill $FRONTEND_PID 2>/dev/null || true
    cleanup_all
    echo "✅ All services stopped"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Wait for user to stop
wait 