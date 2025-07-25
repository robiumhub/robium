import express from 'express';
import cors from 'cors';
import helmet from 'helmet';
import morgan from 'morgan';
import dotenv from 'dotenv';
import { createServer } from 'http';
import { WebSocketServer } from 'ws';
import { Database } from './utils/database';
import { MigrationManager } from './utils/migrations';

// Load environment variables
dotenv.config();

// Import routes (will be created later)
// import authRoutes from './routes/auth';
// import apiRoutes from './routes/api';

const app = express();
const PORT = process.env.PORT || 8000;

// Security middleware
app.use(helmet());

// CORS configuration
app.use(cors({
  origin: process.env.FRONTEND_URL || 'http://localhost:3000',
  credentials: true
}));

// Logging middleware
app.use(morgan('combined'));

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
        websocket: 'ready'
      }
    });
  } catch (error) {
    res.status(503).json({
      status: 'ERROR',
      message: 'Service Unavailable',
      timestamp: new Date().toISOString(),
      version: '0.1.0',
      database: 'error',
      error: error instanceof Error ? error.message : 'Unknown error'
    });
  }
});

// API routes (will be uncommented as routes are created)
// app.use('/auth', authRoutes);
// app.use('/api', apiRoutes);

// 404 handler
app.use('*', (req, res) => {
  res.status(404).json({
    error: 'Route not found',
    message: `Cannot ${req.method} ${req.originalUrl}`
  });
});

// Global error handler
app.use((err: any, req: express.Request, res: express.Response, next: express.NextFunction) => {
  console.error('Error:', err);
  
  res.status(err.status || 500).json({
    error: err.message || 'Internal Server Error',
    ...(process.env.NODE_ENV === 'development' && { stack: err.stack })
  });
});

// Create HTTP server
const server = createServer(app);

// Create WebSocket server
const wss = new WebSocketServer({ server });

wss.on('connection', (ws) => {
  console.log('New WebSocket connection established');
  
  ws.on('message', (message) => {
    console.log('Received WebSocket message:', message.toString());
    // Echo message back (will be enhanced later)
    ws.send(`Echo: ${message}`);
  });
  
  ws.on('close', () => {
    console.log('WebSocket connection closed');
  });
});

// Initialize database and start server
async function startServer() {
  try {
    // Connect to database
    await Database.connect();
    
    // Run pending migrations
    const migrationManager = new MigrationManager();
    await migrationManager.runPendingMigrations();
    
    // Start server
    server.listen(PORT, () => {
      console.log(`🚀 Robium Backend Server running on port ${PORT}`);
      console.log(`📡 WebSocket server ready for connections`);
      console.log(`🗃️  Database connected and migrations applied`);
      console.log(`🌍 Environment: ${process.env.NODE_ENV || 'development'}`);
      console.log(`🔗 Health check: http://localhost:${PORT}/health`);
    });
    
  } catch (error) {
    console.error('❌ Failed to start server:', error);
    process.exit(1);
  }
}

// Start the server
startServer();

export default app;
