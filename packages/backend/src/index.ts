import express from 'express';
import cors from 'cors';
import helmet from 'helmet';
import morgan from 'morgan';
import dotenv from 'dotenv';
import path from 'path';

import { WebSocketServer } from './websocket/WebSocketServer';
import { Database } from './utils/database';
import { MigrationManager } from './utils/migrations';
import { logger } from './utils/logger';
import {
  addRequestId,
  requestTiming,
  globalErrorHandler,
  notFoundHandler,
  gracefulShutdown,
  setupErrorHandlers,
} from './middleware/errorHandler';

// Load environment variables
dotenv.config();

// Setup global error handlers
setupErrorHandlers();

// Import routes
import authRoutes from './routes/auth';
import adminRoutes from './routes/admin';
import modulesRoutes from './routes/modules';
import projectsRoutes from './routes/projects';
import dockerfilesRoutes from './routes/dockerfiles';
import rosPackagesRoutes from './routes/ros-packages';
import robotsRoutes from './routes/robots';
// import apiRoutes from './routes/api';

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

// Request tracking middleware
app.use(addRequestId);
app.use(requestTiming);

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

    res.status(200).json({
      status: 'OK',
      message: 'Robium Backend Server is running',
      timestamp: new Date().toISOString(),
      version: '0.1.0',
      database: dbHealth ? 'connected' : 'disconnected',
      services: {
        api: 'running',
        database: dbHealth ? 'healthy' : 'unhealthy',
        websocket: 'ready',
      },
      requestId: (req as any).requestId, // eslint-disable-line @typescript-eslint/no-explicit-any
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
      requestId: (req as any).requestId, // eslint-disable-line @typescript-eslint/no-explicit-any
    });
  }
});

// API routes
app.use('/auth', authRoutes);
app.use('/admin', adminRoutes);
app.use('/modules', modulesRoutes);
app.use('/projects', projectsRoutes);
app.use('/dockerfiles', dockerfilesRoutes);
app.use('/ros-packages', rosPackagesRoutes);
app.use('/robots', robotsRoutes);
// app.use('/api', apiRoutes);

// Serve static files from the frontend build directory (for production)
if (process.env.NODE_ENV === 'production') {
  const frontendBuildPath = path.join(__dirname, '../../frontend/build');
  app.use(express.static(frontendBuildPath));

  // Serve index.html for all non-API routes (SPA routing)
  app.get('*', (req, res) => {
    // Skip API routes
    if (
      req.path.startsWith('/auth') ||
      req.path.startsWith('/admin') ||
      req.path.startsWith('/modules') ||
      req.path.startsWith('/projects') ||
      req.path.startsWith('/dockerfiles') ||
      req.path.startsWith('/ros-packages') ||
      req.path.startsWith('/health') ||
      req.path.startsWith('/ws')
    ) {
      return notFoundHandler(req, res);
    }

    res.sendFile(path.join(frontendBuildPath, 'index.html'));
  });
}

// 404 handler for API routes
app.use('*', notFoundHandler);

// Global error handler (must be last)
app.use(globalErrorHandler);

// Create WebSocket server
const wsServer = new WebSocketServer(app, {
  heartbeat: {
    interval: 30000, // 30 seconds
    timeout: 5000, // 5 seconds
    maxMissedHeartbeats: 2,
  },
  maxConnections: 1000,
  enableLogging: true,
});

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

    // Start WebSocket server (which also handles HTTP)
    await wsServer.start(Number(PORT));
    logger.info(`Server started on port ${PORT}`, {
      port: PORT,
      environment: process.env.NODE_ENV || 'development',
      nodeVersion: process.version,
    });

    // Setup graceful shutdown
    process.on('SIGTERM', () => gracefulShutdown(wsServer, 'SIGTERM'));
    process.on('SIGINT', () => gracefulShutdown(wsServer, 'SIGINT'));
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
