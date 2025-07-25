# Robium Project Rules for AI Assistants

## Project Overview

Robium is a comprehensive web-based robotics development studio IDE that enables visual programming through drag-and-drop interfaces. It integrates deeply with ROS2 (Robot Operating System) and provides a suite of robotics algorithms for easy application development.

## Current Project Architecture

### Monorepo Structure
- **npm workspaces** for dependency management
- **TypeScript** throughout the entire stack
- **Docker Compose** for development environment
- **PostgreSQL** database with migration system
- **Express.js** backend with comprehensive middleware
- **React** frontend with TypeScript

### Core Technologies Stack

- **Frontend**: React + TypeScript, Create React App
- **Backend**: Node.js + Express.js + TypeScript
- **Database**: PostgreSQL with custom migration system
- **Authentication**: JWT with bcryptjs hashing
- **Real-time**: WebSocket server integration
- **Development**: Docker Compose, hot reloading
- **Testing**: Jest with ts-jest
- **Code Quality**: ESLint, Prettier, Husky pre-commit hooks
- **CI/CD**: GitHub Actions

## Current File Structure

```
robium/
├── packages/                    # npm workspaces
│   ├── frontend/               # React frontend application
│   │   ├── src/
│   │   │   ├── index.tsx      # React entry point
│   │   │   └── (components, pages, etc.)
│   │   ├── public/
│   │   ├── package.json       # Frontend dependencies
│   │   ├── tsconfig.json      # Frontend TypeScript config
│   │   └── Dockerfile         # Frontend container
│   ├── backend/               # Express.js backend API
│   │   ├── src/
│   │   │   ├── index.ts       # Express server entry
│   │   │   ├── routes/        # API route handlers
│   │   │   ├── models/        # Data models
│   │   │   ├── middleware/    # Express middleware
│   │   │   ├── utils/         # Utility functions
│   │   │   │   ├── database.ts    # PostgreSQL connection
│   │   │   │   └── migrations.ts  # Migration system
│   │   │   ├── types/         # TypeScript definitions
│   │   │   ├── migrations/    # Database migration files
│   │   │   ├── scripts/       # CLI scripts
│   │   │   └── __tests__/     # Jest tests
│   │   ├── package.json       # Backend dependencies
│   │   ├── jest.config.js     # Jest configuration
│   │   ├── tsconfig.json      # Backend TypeScript config
│   │   ├── .env.example       # Environment template
│   │   └── Dockerfile         # Backend container
│   └── shared/                # Shared types and utilities
├── ros/                       # ROS container configuration
│   └── Dockerfile             # ROS container setup
├── .taskmaster/               # Task Master AI project management
│   ├── tasks/                 # Generated tasks
│   ├── docs/                  # Project documentation
│   └── config.json            # Task Master configuration
├── .cursor/                   # Cursor IDE rules
├── .github/workflows/         # GitHub Actions CI/CD
├── docker-compose.yml         # Multi-service development
├── package.json               # Root workspace configuration
├── .eslintrc.js              # Shared ESLint configuration
├── .prettierrc               # Code formatting rules
└── .husky/                   # Git hooks
```

## Database Architecture

### Schema Design
- **users**: User accounts with authentication (UUID, email, username, password_hash, role)
- **projects**: User projects with ownership (UUID, name, description, owner_id)
- **sessions**: JWT session management (UUID, user_id, token_hash, expires_at)
- **project_members**: Project collaboration (UUID, project_id, user_id, role)

### Migration System
- **Custom migration framework** with UP/DOWN support
- **CLI commands**: `npm run migrate:up`, `migrate:down`, `migrate:status`, `migrate:reset`
- **Automatic execution** on server startup
- **Transaction-based** migrations for data integrity

### Database Features
- **UUID primary keys** for all tables
- **Role-based access control** (admin/user, project permissions)
- **Automatic timestamps** with PostgreSQL triggers
- **Connection pooling** with pg library
- **Comprehensive indexes** for query optimization

## Development Guidelines

### Backend Development Rules

1. **Express.js Patterns**:
   ```typescript
   // ✅ DO: Use async/await with proper error handling
   app.get('/api/endpoint', async (req: AuthRequest, res: Response) => {
     try {
       const result = await Database.query('SELECT * FROM table');
       res.json({ success: true, data: result.rows });
     } catch (error) {
       res.status(500).json({ error: error.message });
     }
   });
   
   // ❌ DON'T: Use callbacks or missing error handling
   app.get('/api/endpoint', (req, res) => {
     Database.query('SELECT * FROM table', (err, result) => {
       res.json(result); // Missing error handling
     });
   });
   ```

2. **Database Operations**:
   ```typescript
   // ✅ DO: Use parameterized queries
   const result = await Database.query(
     'SELECT * FROM users WHERE email = $1',
     [email]
   );
   
   // ✅ DO: Use transactions for complex operations
   await Database.transaction(async (client) => {
     await client.query('INSERT INTO users ...');
     await client.query('INSERT INTO projects ...');
   });
   
   // ❌ DON'T: Use string concatenation (SQL injection risk)
   const result = await Database.query(
     `SELECT * FROM users WHERE email = '${email}'`
   );
   ```

3. **TypeScript Types**:
   ```typescript
   // ✅ DO: Define interfaces in src/types/index.ts
   interface User {
     id: string;
     email: string;
     username: string;
     role: UserRole;
   }
   
   interface AuthRequest extends Request {
     user?: JWTPayload;
   }
   ```

### Frontend Development Rules

1. **React Component Structure**:
   ```typescript
   // ✅ DO: Use functional components with TypeScript
   interface ComponentProps {
     title: string;
     onSubmit: (data: FormData) => void;
   }
   
   export const Component: React.FC<ComponentProps> = ({ title, onSubmit }) => {
     const [loading, setLoading] = useState(false);
     
     const handleSubmit = useCallback(async (data: FormData) => {
       setLoading(true);
       try {
         await onSubmit(data);
       } finally {
         setLoading(false);
       }
     }, [onSubmit]);
     
     return <div>{title}</div>;
   };
   ```

2. **API Integration**:
   ```typescript
   // ✅ DO: Create service functions for API calls
   export const userService = {
     async getUser(id: string): Promise<User> {
       const response = await fetch(`/api/users/${id}`);
       if (!response.ok) throw new Error('Failed to fetch user');
       return response.json();
     }
   };
   ```

### Docker Development Rules

1. **Container Structure**:
   - **Frontend**: React app with hot reloading via volume mounts
   - **Backend**: Express.js with source code volume mounts for development
   - **Database**: PostgreSQL with persistent data volume
   - **ROS**: Ubuntu container ready for ROS2 integration

2. **Volume Mount Strategy**:
   ```yaml
   # ✅ DO: Mount source directories only, not entire packages
   volumes:
     - ./packages/frontend/src:/app/packages/frontend/src
     - ./packages/backend/src:/app/packages/backend/src
   
   # ❌ DON'T: Mount entire package (overwrites node_modules)
   volumes:
     - ./packages/frontend:/app
   ```

## Code Quality Standards

### TypeScript Configuration
- **Strict mode enabled** for all packages
- **Shared types** in `packages/shared/` when needed
- **Path mapping** configured for imports
- **No any types** allowed in production code

### Testing Requirements
- **Jest** for unit and integration tests
- **Test setup** in `src/__tests__/setup.ts`
- **Coverage reports** generated for all packages
- **Database tests** use transaction rollback

### Code Formatting
- **Prettier** for consistent formatting
- **ESLint** for code quality and TypeScript rules
- **Husky pre-commit hooks** for automated checks
- **GitHub Actions** for CI/CD validation

## Security Implementation

### Authentication System
- **JWT tokens** with secure secrets from environment
- **bcryptjs** for password hashing (12 rounds)
- **Session management** with refresh tokens
- **Role-based access control** (admin/user roles)

### Database Security
- **Parameterized queries** to prevent SQL injection
- **Connection pooling** with proper limits
- **Environment variables** for sensitive configuration
- **UUID primary keys** instead of incremental IDs

### Container Security
- **Non-root users** in production containers
- **Minimal base images** (node:18-alpine)
- **Environment isolation** between services
- **Secrets management** via environment variables

## Task Master Integration

### Project Management
- **AI-powered task generation** from PRDs
- **Subtask breakdown** with complexity analysis
- **Migration tracking** and status reporting
- **Progress logging** with timestamped updates

### Development Workflow
1. **Initialize** Task Master in project root
2. **Parse PRD** to generate initial tasks
3. **Expand tasks** into subtasks based on complexity
4. **Update progress** as implementation proceeds
5. **Mark completion** when tasks are verified

## Performance Guidelines

### Backend Performance
- **Connection pooling** for database (2-20 connections)
- **Query optimization** with proper indexes
- **Async/await** for non-blocking operations
- **Error handling** with appropriate HTTP status codes

### Frontend Performance
- **Code splitting** with dynamic imports
- **Lazy loading** for non-critical components
- **Memoization** for expensive calculations
- **Bundle optimization** with Create React App

### Database Performance
- **Indexes** on frequently queried columns
- **Foreign key constraints** with CASCADE options
- **Triggers** for automatic timestamp updates
- **Query monitoring** with execution time logging

## Common Patterns

### API Response Structure
```typescript
// ✅ DO: Consistent API response format
interface ApiResponse<T = any> {
  success: boolean;
  message?: string;
  data?: T;
  error?: string;
}
```

### Error Handling
```typescript
// ✅ DO: Comprehensive error handling
app.use((err: ApiError, req: Request, res: Response, next: NextFunction) => {
  console.error('Error:', err);
  
  res.status(err.status || 500).json({
    error: err.message || 'Internal Server Error',
    ...(process.env.NODE_ENV === 'development' && { stack: err.stack })
  });
});
```

### Migration Pattern
```sql
-- ✅ DO: Include both UP and DOWN sections
-- UP
CREATE TABLE example (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name VARCHAR(255) NOT NULL
);

-- DOWN
DROP TABLE IF EXISTS example;
```

## AI Assistant Behavior

When working on this project, AI assistants should:

1. **Follow the monorepo structure** - work within packages/frontend or packages/backend
2. **Use the established database patterns** - migrations, connection pooling, parameterized queries
3. **Maintain TypeScript strict typing** throughout the codebase
4. **Update Task Master progress** when completing subtasks
5. **Test Docker integration** after making infrastructure changes
6. **Follow security patterns** for authentication and database access
7. **Document changes** in Task Master subtask updates
8. **Use the migration system** for any database schema changes

## Current Development Status

### Completed (Task 1 & 2.1-2.2)
- ✅ **Monorepo setup** with npm workspaces
- ✅ **Docker multi-service environment** (frontend, backend, database, ROS)
- ✅ **Express.js backend** with TypeScript and comprehensive middleware
- ✅ **Database schema** with users, projects, sessions, project_members
- ✅ **Migration system** with CLI commands and automatic execution
- ✅ **Development tooling** (ESLint, Prettier, Husky, GitHub Actions)
- ✅ **Health check endpoints** with database status monitoring

### In Progress (Task 2.3+)
- 🔄 **User Model implementation** with validation and business logic
- ⏳ **Authentication endpoints** (signup, login, logout)
- ⏳ **JWT handling** with refresh tokens
- ⏳ **Role-based access control** system
- ⏳ **WebSocket authentication** and real-time features

### Architecture Notes
- **Database**: PostgreSQL with custom migration framework
- **Authentication**: JWT-based with bcryptjs password hashing
- **Real-time**: WebSocket server integrated with Express
- **Development**: Hot reloading with Docker volume mounts
- **Testing**: Jest with TypeScript support and database transactions
