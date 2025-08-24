# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Robium is a modern web platform for robotics project configurations and Dockerfile generation. It consists of a React frontend, Express.js backend, and SQLite database in a TypeScript monorepo structure.

## Common Development Commands

### Quick Start
```bash
npm install                    # Install all dependencies
npm start                     # Start both frontend and backend
npm run dev                   # Same as npm start
```

### Individual Services
```bash
npm run dev:frontend          # Start React app (localhost:3000)
npm run dev:backend           # Start Express server (localhost:8000)
npm run start:backend         # Start backend in production mode
```

### Build and Test
```bash
npm run build                 # Build all packages
npm run build:frontend        # Build React app only
npm run build:backend         # Build Express server only
npm run test                  # Run all tests
npm run lint                  # Run linting for all packages
npm run format                # Format code with Prettier
```

### Database Operations
```bash
npm run db:migrate            # Run database migrations
npm run seed:all              # Seed all sample data
npm run seed:users            # Seed sample users
npm run seed:projects         # Seed sample projects  
npm run seed:templates        # Seed sample templates
npm run db:reset              # Reset database (destructive)
```

### Testing
```bash
npm run test:frontend         # Run frontend tests
npm run test:backend          # Run backend tests
npm run test:github           # Test GitHub integration
node test-smoke.js            # Run API smoke tests
```

## Architecture

### Monorepo Structure
- **packages/frontend/**: React application with Material-UI
- **packages/backend/**: Express.js API server with SQLite
- **packages/shared/**: Shared TypeScript types and utilities

### Database
- SQLite database with migrations system
- Database file located at `data/robium.db` (relative to project root)
- WAL mode enabled for better concurrency
- Foreign keys enabled

### Authentication
- JWT-based authentication
- Token stored in localStorage on frontend
- Automatic token refresh and logout on 401 errors
- Role-based access (user/admin)

### API Structure
- RESTful API with consistent response format
- All responses follow `ApiResponse<T>` interface
- Proper error handling with status codes
- Request/response logging with Morgan

## Key Technologies

### Frontend
- React 18 with TypeScript
- Material-UI for components
- Axios for API calls
- React Router for navigation
- React Testing Library for tests

### Backend  
- Express.js with TypeScript
- SQLite3 database
- bcryptjs for password hashing
- jsonwebtoken for authentication
- Joi for input validation
- Octokit for GitHub integration

## Development Workflow

### Environment Setup
1. Copy `.env.example` to `.env` and configure
2. Run `npm install` to install dependencies
3. Run `npm run db:migrate` to set up database
4. Run `npm run seed:all` to add sample data
5. Run `npm start` to start development servers

### Adding Dependencies
```bash
# Add to specific package
npm install express --workspace=packages/backend
npm install react --workspace=packages/frontend

# Add shared dev dependencies to root
npm install typescript --save-dev
```

### Database Changes
1. Create migration file in `packages/backend/src/migrations/`
2. Include both UP and DOWN sections
3. Run `npm run db:migrate` to apply changes
4. Update TypeScript types accordingly

### GitHub Integration
- Requires `GITHUB_TOKEN` environment variable
- Only admin users can create GitHub repositories
- Test with `npm run test:github`

## Project-Specific Patterns

### API Service Layer
- All API calls go through `ApiService` class in frontend
- Automatic authentication header injection
- Consistent error handling and response typing
- Located at `packages/frontend/src/services/api.ts`

### Database Manager
- Singleton `Database` class handles connections
- Proper error handling and logging
- Health check functionality
- Located at `packages/backend/src/utils/database.ts`

### Shared Types
- Common interfaces in `packages/shared/`
- Used by both frontend and backend
- Includes API responses, entities, and DTOs

### Authentication Context
- React Context provides auth state
- Automatic token validation on app load
- Logout on token expiration

## Testing Strategy

### Backend Tests
- Use Jest for unit tests
- Database transactions for test isolation
- Mock external services (GitHub API)

### Frontend Tests
- React Testing Library for components
- Jest for utility functions
- Mock API service for integration tests

### Smoke Tests
- `test-smoke.js` verifies all API endpoints
- Expects 6/7 tests passing (admin requires token)
- Run before deployments

## Important Notes

- Use `npm run lint` before commits to ensure code quality
- Database migrations are automatically run on server startup
- Frontend proxy configured to backend at `http://localhost:8000`
- All API routes prefixed with `/api/`
- WebSocket support available for real-time features
- GitHub integration is optional and admin-only