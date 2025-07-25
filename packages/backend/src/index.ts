import express from 'express';
import cors from 'cors';
import helmet from 'helmet';
import morgan from 'morgan';
import dotenv from 'dotenv';
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
  setupErrorHandlers 
} from './middleware/errorHandler';

// Load environment variables
dotenv.config();

// Setup global error handlers
setupErrorHandlers();

// Import routes
import authRoutes from './routes/auth';
import adminRoutes from './routes/admin';
// import apiRoutes from './routes/api';

const app = express();
const PORT = process.env.PORT || 8000;

// Security middleware
app.use(helmet());

// CORS configuration
app.use(
  cors({
    origin: process.env.FRONTEND_URL || 'http://localhost:3000',
    credentials: true,
  })
);

// Request tracking middleware
app.use(addRequestId);
app.use(requestTiming);

// Logging middleware
app.use(morgan('combined', {
  stream: {
    write: (message: string) => {
      logger.info('HTTP Request', { message: message.trim() });
    }
  }
}));

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
      requestId: req.requestId
    });
  } catch (error) {
    logger.error('Health check failed', { error: error instanceof Error ? error.message : 'Unknown error' });
    res.status(503).json({
      status: 'ERROR',
      message: 'Service Unavailable',
      timestamp: new Date().toISOString(),
      version: '0.1.0',
      database: 'error',
      error: error instanceof Error ? error.message : 'Unknown error',
      requestId: req.requestId
    });
  }
});

// API routes
app.use('/auth', authRoutes);
app.use('/admin', adminRoutes);
// app.use('/api', apiRoutes);

// 404 handler
app.use('*', notFoundHandler);

// Global error handler (must be last)
app.use(globalErrorHandler);

// Create WebSocket server
const wsServer = new WebSocketServer(app, {
  heartbeat: {
    interval: 30000, // 30 seconds
    timeout: 5000,   // 5 seconds
    maxMissedHeartbeats: 2
  },
  maxConnections: 1000,
  enableLogging: true
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

    // Start HTTP server
    const server = app.listen(Number(PORT), () => {
      logger.info(`Server started on port ${PORT}`, {
        port: PORT,
        environment: process.env.NODE_ENV || 'development',
        nodeVersion: process.version
      });
    });

    // Start WebSocket server
    await wsServer.start(Number(PORT));
    logger.info('WebSocket server started');

    // Setup graceful shutdown
    process.on('SIGTERM', () => gracefulShutdown(server, 'SIGTERM'));
    process.on('SIGINT', () => gracefulShutdown(server, 'SIGINT'));

  } catch (error) {
    logger.error('Failed to start server', { error: error instanceof Error ? error.message : 'Unknown error' });
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
