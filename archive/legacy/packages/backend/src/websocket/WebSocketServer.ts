import { createServer, Server as HTTPServer } from 'http';
import { WebSocketServer as WSWebSocketServer } from 'ws';
import { Express } from 'express';
import { ConnectionManager } from './ConnectionManager';
import {
  WebSocketServerConfig,
  WebSocketStats,
  WebSocketMessage,
  WebSocketEventType,
} from './types';

export class WebSocketServer {
  private httpServer: HTTPServer;
  private wss: WSWebSocketServer;
  private connectionManager: ConnectionManager;
  private config: WebSocketServerConfig;
  private isRunning: boolean = false;

  constructor(
    private app: Express,
    config: WebSocketServerConfig = {
      heartbeat: {
        interval: 30000, // 30 seconds
        timeout: 60000, // 60 seconds
        maxMissedHeartbeats: 2,
      },
      enableLogging: true,
      maxConnections: 1000,
    }
  ) {
    this.config = config;
    this.httpServer = createServer(app);
    this.wss = new WSWebSocketServer({
      server: this.httpServer,
      path: config.path || '/ws',
    });
    this.connectionManager = new ConnectionManager(this.wss, config.heartbeat);
  }

  public start(port: number = 8000): Promise<void> {
    return new Promise((resolve, reject) => {
      if (this.isRunning) {
        reject(new Error('WebSocket server is already running'));
        return;
      }

      this.httpServer.listen(port, () => {
        this.isRunning = true;
        console.log(`🚀 WebSocket server started on port ${port}`);
        console.log(
          `📡 WebSocket endpoint: ws://localhost:${port}${this.config.path || '/ws'}`
        );

        if (this.config.enableLogging) {
          this.logServerInfo();
        }

        resolve();
      });

      this.httpServer.on('error', (error) => {
        console.error('❌ WebSocket server error:', error);
        reject(error);
      });
    });
  }

  public stop(): Promise<void> {
    return new Promise((resolve) => {
      if (!this.isRunning) {
        resolve();
        return;
      }

      console.log('🛑 Stopping WebSocket server...');

      this.connectionManager.disconnect();

      this.httpServer.close(() => {
        this.isRunning = false;
        console.log('✅ WebSocket server stopped');
        resolve();
      });
    });
  }

  public getConnectionManager(): ConnectionManager {
    return this.connectionManager;
  }

  public getStats(): WebSocketStats {
    return this.connectionManager.getStats();
  }

  public isServerRunning(): boolean {
    return this.isRunning;
  }

  // Utility methods for external use
  public sendToUser(userId: string, message: WebSocketMessage): void {
    this.connectionManager.sendToUser(userId, message);
  }

  public broadcastToAll(message: WebSocketMessage): void {
    this.connectionManager.broadcastToAll(message);
  }

  public broadcastToRoom(roomId: string, message: WebSocketMessage): void {
    this.connectionManager.broadcastToRoom(roomId, message);
  }

  public broadcastToAuthenticated(message: WebSocketMessage): void {
    this.connectionManager.broadcastToAuthenticated(message);
  }

  public isUserConnected(userId: string): boolean {
    return this.connectionManager.isUserConnected(userId);
  }

  public getRoomMembers(roomId: string): string[] {
    return this.connectionManager.getRoomMembers(roomId);
  }

  // Notification helpers
  public sendNotification(
    userId: string,
    title: string,
    message: string,
    type: 'info' | 'success' | 'warning' | 'error' = 'info',
    data?: unknown,
    actionUrl?: string
  ): void {
    this.sendToUser(userId, {
      type: WebSocketEventType.NOTIFICATION,
      data: {
        type,
        title,
        message,
        data,
        actionUrl,
      },
      timestamp: Date.now(),
    });
  }

  public broadcastNotification(
    title: string,
    message: string,
    type: 'info' | 'success' | 'warning' | 'error' = 'info',
    data?: unknown,
    actionUrl?: string
  ): void {
    this.broadcastToAuthenticated({
      type: WebSocketEventType.NOTIFICATION,
      data: {
        type,
        title,
        message,
        data,
        actionUrl,
      },
      timestamp: Date.now(),
    });
  }

  // Error notification helpers
  public sendError(
    userId: string,
    code: string,
    message: string,
    details?: unknown
  ): void {
    this.sendToUser(userId, {
      type: WebSocketEventType.ERROR,
      data: {
        code,
        message,
        details,
      },
      timestamp: Date.now(),
    });
  }

  // Data synchronization helpers
  public broadcastDataSync(
    entityType: 'project' | 'session' | 'user',
    entityId: string,
    action: 'create' | 'update' | 'delete',
    data: unknown,
    version: number
  ): void {
    this.broadcastToAuthenticated({
      type: WebSocketEventType.DATA_SYNC,
      data: {
        entityType,
        entityId,
        action,
        data,
        version,
        timestamp: Date.now(),
      },
      timestamp: Date.now(),
    });
  }

  // Project update helpers
  public broadcastProjectUpdate(
    projectId: string,
    userId: string,
    action:
      | 'created'
      | 'updated'
      | 'deleted'
      | 'member_added'
      | 'member_removed',
    data: unknown
  ): void {
    this.broadcastToAuthenticated({
      type: WebSocketEventType.PROJECT_UPDATE,
      data: {
        projectId,
        userId,
        action,
        data,
        timestamp: Date.now(),
      },
      timestamp: Date.now(),
    });
  }

  // Session update helpers
  public broadcastSessionUpdate(
    sessionId: string,
    userId: string,
    action: 'started' | 'updated' | 'ended' | 'user_joined' | 'user_left',
    data: unknown
  ): void {
    this.broadcastToAuthenticated({
      type: WebSocketEventType.SESSION_UPDATE,
      data: {
        sessionId,
        userId,
        action,
        data,
        timestamp: Date.now(),
      },
      timestamp: Date.now(),
    });
  }

  // Collaboration helpers
  public broadcastCollaborationUpdate(
    sessionId: string,
    projectId: string,
    userId: string,
    action:
      | 'cursor_move'
      | 'selection_change'
      | 'content_change'
      | 'comment_add',
    data: unknown
  ): void {
    this.broadcastToAuthenticated({
      type: WebSocketEventType.COLLABORATION_UPDATE,
      data: {
        sessionId,
        projectId,
        userId,
        action,
        data,
        timestamp: Date.now(),
      },
      timestamp: Date.now(),
    });
  }

  private logServerInfo(): void {
    console.log('📊 WebSocket Server Configuration:');
    console.log(`   - Heartbeat interval: ${this.config.heartbeat.interval}ms`);
    console.log(`   - Heartbeat timeout: ${this.config.heartbeat.timeout}ms`);
    console.log(
      `   - Max connections: ${this.config.maxConnections || 'unlimited'}`
    );
    console.log(`   - Logging enabled: ${this.config.enableLogging}`);
    console.log(`   - WebSocket path: ${this.config.path || '/ws'}`);
  }

  // Health check method
  public getHealthStatus(): {
    status: 'healthy' | 'unhealthy';
    uptime: number;
    connections: number;
    memory: NodeJS.MemoryUsage;
  } {
    const stats = this.getStats();
    const memoryUsage = process.memoryUsage();

    return {
      status: this.isRunning ? 'healthy' : 'unhealthy',
      uptime: stats.uptime,
      connections: stats.activeConnections,
      memory: memoryUsage,
    };
  }
}
