#!/bin/bash

# Fast development startup script for Robium
# Optimized for quick startup with memory improvements

set -e

echo "🚀 Starting Robium development environment (Fast Mode)..."

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker first."
    exit 1
fi

# Set environment variables for faster startup
export COMPOSE_DOCKER_CLI_BUILD=1
export DOCKER_BUILDKIT=1
export FAST_REFRESH=false
export CHOKIDAR_USEPOLLING=false
export WATCHPACK_POLLING=false
export GENERATE_SOURCEMAP=false

# Stop any existing containers
echo "🛑 Stopping existing containers..."
docker compose down --remove-orphans

# Build with cache optimization
echo "🔨 Building containers with optimizations..."
docker compose build --parallel --no-cache frontend backend

# Start services with optimized settings
echo "▶️  Starting services..."
docker compose up -d

# Wait for services to be ready
echo "⏳ Waiting for services to be ready..."
sleep 5

# Check service status
echo "📊 Service Status:"
docker compose ps

# Check if frontend is responding
echo "🔍 Checking frontend availability..."
for i in {1..30}; do
    if curl -s -f http://localhost:3000 > /dev/null 2>&1; then
        echo "✅ Frontend is ready at http://localhost:3000"
        break
    fi
    if [ $i -eq 30 ]; then
        echo "❌ Frontend failed to start within 30 seconds"
        docker compose logs frontend --tail=20
        exit 1
    fi
    echo "⏳ Waiting for frontend... ($i/30)"
    sleep 2
done

# Check if backend is responding
echo "🔍 Checking backend availability..."
for i in {1..15}; do
    if curl -s -f http://localhost:8000/health > /dev/null 2>&1; then
        echo "✅ Backend is ready at http://localhost:8000"
        break
    fi
    if [ $i -eq 15 ]; then
        echo "❌ Backend failed to start within 15 seconds"
        docker compose logs backend --tail=20
        exit 1
    fi
    echo "⏳ Waiting for backend... ($i/15)"
    sleep 2
done

echo ""
echo "🎉 Robium development environment is ready!"
echo "📱 Frontend: http://localhost:3000"
echo "🔧 Backend:  http://localhost:8000"
echo "🗄️  Database: localhost:5432"
echo ""
echo "💡 Tips:"
echo "   - Use 'docker compose logs -f [service]' to follow logs"
echo "   - Use 'docker compose down' to stop all services"
echo "   - Use 'docker compose restart [service]' to restart a specific service"

