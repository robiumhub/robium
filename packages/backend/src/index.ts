import express from 'express';
import cors from 'cors';
import helmet from 'helmet';
import morgan from 'morgan';
import dotenv from 'dotenv';
import path from 'path';

import { Database } from './utils/database';
import { MigrationManager } from './utils/migrations';
import { logger } from './utils/logger';
import { globalErrorHandler, notFoundHandler } from './middleware/errorHandler';

// Load environment variables
// Look for .env file in parent directory when running from dist folder
dotenv.config({ path: path.join(__dirname, '../.env') });

// Import routes
import authRoutes from './routes/auth';
import projectsRoutes from './routes/projects';
import dockerfilesRoutes from './routes/dockerfiles';
import integrationsRoutes from './routes/integrations.github';
import adminRoutes from './routes/admin';
import usersRoutes from './routes/users';

const app = express();
const PORT = process.env.PORT || 8000;

// Security middleware
app.use(helmet());

// CORS configuration
app.use(
  cors({
    origin: process.env.CORS_ORIGIN || 'http://localhost:3000',
    credentials: true,
  })
);

// Logging middleware
app.use(
  morgan('combined', {
    stream: {
      write: (message: string) => {
        logger.info('HTTP Request', { message: message.trim() });
      },
    },
  })
);

// Body parsing middleware
app.use(express.json({ limit: '10mb' }));
app.use(express.urlencoded({ extended: true }));

// Health check endpoint
app.get('/health', async (req, res) => {
  try {
    const dbHealth = await Database.healthCheck();
    const githubIntegration = Boolean(process.env.GITHUB_TOKEN);

    res.status(200).json({
      status: 'OK',
      message: 'Robium Backend Server is running',
      timestamp: new Date().toISOString(),
      version: '0.1.0',
      database: dbHealth ? 'connected' : 'disconnected',
      githubIntegration,
      services: {
        api: 'running',
        database: dbHealth ? 'healthy' : 'unhealthy',
      },
    });
  } catch (error) {
    logger.error('Health check failed', {
      error: error instanceof Error ? error.message : 'Unknown error',
    });
    res.status(503).json({
      status: 'ERROR',
      message: 'Service Unavailable',
      timestamp: new Date().toISOString(),
      version: '0.1.0',
      database: 'error',
      error: error instanceof Error ? error.message : 'Unknown error',
    });
  }
});

// API routes
app.use('/api/auth', authRoutes);
app.use('/api/projects', projectsRoutes);
app.use('/api/dockerfiles', dockerfilesRoutes);
app.use('/api/integrations/github', integrationsRoutes);
app.use('/api/admin', adminRoutes);
app.use('/api/users', usersRoutes);

// Serve static files from the frontend build directory (for production)
if (process.env.NODE_ENV === 'production') {
  const frontendBuildPath = path.join(__dirname, '../../frontend/build');
  app.use(express.static(frontendBuildPath));

  // Serve index.html for all non-API routes (SPA routing)
  app.get('*', (req, res) => {
    // Skip API routes
    if (req.path.startsWith('/api/') || req.path.startsWith('/health')) {
      return notFoundHandler(req, res);
    }

    res.sendFile(path.join(frontendBuildPath, 'index.html'));
  });
}

// 404 handler for API routes
app.use('*', notFoundHandler);

// Global error handler (must be last)
app.use(globalErrorHandler);

// Start server
async function startServer() {
  try {
    // Initialize database
    await Database.connect();
    logger.info('Database connected successfully');

    // Run migrations
    const migrationManager = new MigrationManager();
    await migrationManager.runPendingMigrations();
    logger.info('Database migrations completed');

    // Start server
    app.listen(Number(PORT), () => {
      logger.info(`Server started on port ${PORT}`, {
        port: PORT,
        environment: process.env.NODE_ENV || 'development',
        nodeVersion: process.version,
        githubIntegration: Boolean(process.env.GITHUB_TOKEN),
      });
    });
  } catch (error) {
    logger.error('Failed to start server', {
      error: error instanceof Error ? error.message : 'Unknown error',
    });
    process.exit(1);
  }
}

// Handle uncaught exceptions
process.on('uncaughtException', (error) => {
  logger.error('Uncaught Exception', {}, error);
  process.exit(1);
});

// Handle unhandled promise rejections
process.on('unhandledRejection', (reason, promise) => {
  logger.error('Unhandled Rejection', { reason, promise });
  process.exit(1);
});

// Start the server
startServer();
