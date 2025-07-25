"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
const express_1 = __importDefault(require("express"));
const cors_1 = __importDefault(require("cors"));
const helmet_1 = __importDefault(require("helmet"));
const morgan_1 = __importDefault(require("morgan"));
const dotenv_1 = __importDefault(require("dotenv"));
const WebSocketServer_1 = require("./websocket/WebSocketServer");
const database_1 = require("./utils/database");
const migrations_1 = require("./utils/migrations");
const logger_1 = require("./utils/logger");
const errorHandler_1 = require("./middleware/errorHandler");
// Load environment variables
dotenv_1.default.config();
// Setup global error handlers
(0, errorHandler_1.setupErrorHandlers)();
// Import routes
const auth_1 = __importDefault(require("./routes/auth"));
const admin_1 = __importDefault(require("./routes/admin"));
// import apiRoutes from './routes/api';
const app = (0, express_1.default)();
const PORT = process.env.PORT || 8000;
// Security middleware
app.use((0, helmet_1.default)());
// CORS configuration
app.use((0, cors_1.default)({
    origin: process.env.FRONTEND_URL || 'http://localhost:3000',
    credentials: true,
}));
// Request tracking middleware
app.use(errorHandler_1.addRequestId);
app.use(errorHandler_1.requestTiming);
// Logging middleware
app.use((0, morgan_1.default)('combined', {
    stream: {
        write: (message) => {
            logger_1.logger.info('HTTP Request', { message: message.trim() });
        },
    },
}));
// Body parsing middleware
app.use(express_1.default.json({ limit: '10mb' }));
app.use(express_1.default.urlencoded({ extended: true }));
// Health check endpoint
app.get('/health', async (req, res) => {
    try {
        const dbHealth = await database_1.Database.healthCheck();
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
            requestId: req.requestId,
        });
    }
    catch (error) {
        logger_1.logger.error('Health check failed', {
            error: error instanceof Error ? error.message : 'Unknown error',
        });
        res.status(503).json({
            status: 'ERROR',
            message: 'Service Unavailable',
            timestamp: new Date().toISOString(),
            version: '0.1.0',
            database: 'error',
            error: error instanceof Error ? error.message : 'Unknown error',
            requestId: req.requestId,
        });
    }
});
// API routes
app.use('/auth', auth_1.default);
app.use('/admin', admin_1.default);
// app.use('/api', apiRoutes);
// 404 handler
app.use('*', errorHandler_1.notFoundHandler);
// Global error handler (must be last)
app.use(errorHandler_1.globalErrorHandler);
// Create WebSocket server
const wsServer = new WebSocketServer_1.WebSocketServer(app, {
    heartbeat: {
        interval: 30000,
        timeout: 5000,
        maxMissedHeartbeats: 2,
    },
    maxConnections: 1000,
    enableLogging: true,
});
// Start server
async function startServer() {
    try {
        // Initialize database
        await database_1.Database.connect();
        logger_1.logger.info('Database connected successfully');
        // Run migrations
        const migrationManager = new migrations_1.MigrationManager();
        await migrationManager.runPendingMigrations();
        logger_1.logger.info('Database migrations completed');
        // Start HTTP server
        const server = app.listen(Number(PORT), () => {
            logger_1.logger.info(`Server started on port ${PORT}`, {
                port: PORT,
                environment: process.env.NODE_ENV || 'development',
                nodeVersion: process.version,
            });
        });
        // Start WebSocket server
        await wsServer.start(Number(PORT));
        logger_1.logger.info('WebSocket server started');
        // Setup graceful shutdown
        process.on('SIGTERM', () => (0, errorHandler_1.gracefulShutdown)(server, 'SIGTERM'));
        process.on('SIGINT', () => (0, errorHandler_1.gracefulShutdown)(server, 'SIGINT'));
    }
    catch (error) {
        logger_1.logger.error('Failed to start server', {
            error: error instanceof Error ? error.message : 'Unknown error',
        });
        process.exit(1);
    }
}
// Handle uncaught exceptions
process.on('uncaughtException', (error) => {
    logger_1.logger.error('Uncaught Exception', {}, error);
    process.exit(1);
});
// Handle unhandled promise rejections
process.on('unhandledRejection', (reason, promise) => {
    logger_1.logger.error('Unhandled Rejection', { reason, promise });
    process.exit(1);
});
// Start the server
startServer();
