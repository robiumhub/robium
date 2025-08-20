# Robium

A modern web platform for robotics project configurations and Dockerfile generation.

## Quick Start

### Prerequisites

- Node.js 18+
- npm 8+

### Run the Application

**Option 1: Start both frontend and backend together**

```bash
npm install
npm start
```

This will start both services:

- Backend: `http://localhost:8000`
- Frontend: `http://localhost:3000`

**Option 2: Start services individually**

```bash
# Install dependencies
npm install

# Start backend
cd packages/backend
npm run build
node dist/index.js

# Start frontend (in a new terminal)
cd packages/frontend
npm start
```

### Test the Backend

Run the smoke tests to verify everything is working:

```bash
node test-smoke.js
```

Expected: 6/7 tests passing (admin dashboard requires admin token).

## Development

### Backend Commands

```bash
cd packages/backend
npm run build          # Build TypeScript
npm run db:migrate     # Run database migrations
npm run seed:all       # Seed sample data
```

### Frontend Commands

```bash
cd packages/frontend
npm start              # Start development server
npm run build          # Build for production
```

### Root Commands

```bash
npm start              # Start both frontend and backend
npm run dev            # Start both frontend and backend
npm run build:backend  # Build only backend
npm run build          # Build all packages
npm run clean          # Clean build artifacts
```

## API Endpoints

- `GET /health` - Health check
- `GET /api/auth/me` - Get current user
- `GET /api/projects` - List projects
- `GET /api/projects/templates` - List templates
- `GET /api/dockerfiles/:projectId` - Get project dockerfile
- `GET /api/integrations/github/status` - GitHub integration status

## Project Structure

```
robium/
├── packages/
│   ├── backend/          # Express.js API server
│   ├── frontend/         # React application
│   └── shared/           # Shared TypeScript types
├── docs/                 # Project documentation
└── test-smoke.js         # API smoke tests
```

## Environment Variables

Create `.env` file in project root:

```env
PORT=8000
NODE_ENV=development
CORS_ORIGIN=http://localhost:3000
JWT_SECRET=your_jwt_secret
```

## Current Status

✅ **Backend**: Fully functional and running  
✅ **Frontend**: Development server working  
✅ **API**: All endpoints working correctly  
✅ **Database**: Connected with migrations  
✅ **Testing**: Smoke tests passing (6/7)  
✅ **Build**: Both frontend and backend build successfully  
✅ **Start Script**: `npm start` works for both services
