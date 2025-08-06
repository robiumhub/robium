import { WebSocket, WebSocketServer } from 'ws';
import { IncomingMessage } from 'http';
import { URL } from 'url';
import {
  WebSocketConnection,
  Room,
  UserPresence,
  UserStatus,
  WebSocketMessage,
  AuthenticationResult,
  WebSocketEventType,
  WebSocketStats,
  HeartbeatConfig,
} from './types';
import { AuthService } from '../services/AuthService';
import { JWTPayload } from '../types';
import { ExecutionHandler } from './ExecutionHandler';

export class ConnectionManager {
  private connections: Map<string, WebSocketConnection> = new Map();
  private rooms: Map<string, Room> = new Map();
  private userPresence: Map<string, UserPresence> = new Map();
  private heartbeatConfig: HeartbeatConfig;
  private heartbeatInterval: NodeJS.Timeout | null = null;
  private serverStartTime: number;
  private totalMessages: number = 0;
  private executionHandler: ExecutionHandler;

  constructor(
    private wss: WebSocketServer,
    heartbeatConfig: HeartbeatConfig = {
      interval: 30000, // 30 seconds
      timeout: 60000, // 60 seconds
      maxMissedHeartbeats: 2,
    }
  ) {
    this.heartbeatConfig = heartbeatConfig;
    this.serverStartTime = Date.now();
    this.executionHandler = new ExecutionHandler();
    this.setupWebSocketServer();
    this.startHeartbeat();
  }

  private setupWebSocketServer(): void {
    this.wss.on(
      'connection',
      (ws: WebSocketConnection, request: IncomingMessage) => {
        this.handleConnection(ws, request);
      }
    );
  }

  private async handleConnection(
    ws: WebSocketConnection,
    request: IncomingMessage
  ): Promise<void> {
    try {
      // Extract token from query parameters or headers
      const token = this.extractToken(request);

      // Set initial connection state
      ws.isAlive = true;
      ws.metadata = {
        userId: '',
        username: '',
        role: '',
        connectedAt: Date.now(),
        lastHeartbeat: Date.now(),
        rooms: new Set(),
        isAuthenticated: false,
        userAgent: request.headers['user-agent'],
        ip: this.getClientIP(request),
      };

      // Authenticate connection if token is provided
      if (token) {
        const authResult = await this.authenticateConnection(token);
        if (authResult.success && authResult.user) {
          ws.metadata.userId = authResult.user.userId;
          ws.metadata.username = authResult.user.email;
          ws.metadata.role = authResult.user.role;
          ws.metadata.isAuthenticated = true;

          // Add to connections map
          this.connections.set(authResult.user.userId, ws);

          // Update user presence
          this.updateUserPresence(authResult.user.userId, UserStatus.ONLINE);

          // Send authentication success message
          this.sendMessage(ws, {
            type: WebSocketEventType.AUTHENTICATION_SUCCESS,
            data: { user: authResult.user },
            timestamp: Date.now(),
          });

          // Broadcast user online status
          this.broadcastToAll({
            type: WebSocketEventType.USER_ONLINE,
            data: {
              userId: authResult.user.userId,
              username: authResult.user.email,
              status: UserStatus.ONLINE,
              lastSeen: Date.now(),
            },
            timestamp: Date.now(),
          });

          console.log(
            `WebSocket connection authenticated: ${authResult.user.email} (${authResult.user.userId})`
          );
        } else {
          // Send authentication failed message
          this.sendMessage(ws, {
            type: WebSocketEventType.AUTHENTICATION_FAILED,
            data: { error: authResult.error },
            timestamp: Date.now(),
          });

          console.log(`WebSocket authentication failed: ${authResult.error}`);
        }
      } else {
        // Send authentication required message
        this.sendMessage(ws, {
          type: WebSocketEventType.AUTHENTICATION_FAILED,
          data: { error: 'Authentication token required' },
          timestamp: Date.now(),
        });
      }

      // Set up connection event handlers
      ws.on('message', (data: Buffer) => {
        this.handleMessage(ws, data);
      });

      ws.on('close', () => {
        this.handleDisconnection(ws);
      });

      ws.on('error', (error) => {
        this.handleError(ws, error);
      });

      ws.on('pong', () => {
        ws.isAlive = true;
        if (ws.metadata) {
          ws.metadata.lastHeartbeat = Date.now();
        }
      });
    } catch (error) {
      console.error('Error handling WebSocket connection:', error);
      this.sendMessage(ws, {
        type: WebSocketEventType.ERROR,
        data: { error: 'Connection setup failed' },
        timestamp: Date.now(),
      });
    }
  }

  private extractToken(request: IncomingMessage): string | null {
    try {
      const url = new URL(request.url || '', `http://${request.headers.host}`);
      const token = url.searchParams.get('token');

      if (token) {
        return token;
      }

      // Check Authorization header
      const authHeader = request.headers.authorization;
      if (authHeader && authHeader.startsWith('Bearer ')) {
        return authHeader.substring(7);
      }

      return null;
    } catch (error) {
      console.error('Error extracting token:', error);
      return null;
    }
  }

  private async authenticateConnection(
    token: string
  ): Promise<AuthenticationResult> {
    try {
      const user = await AuthService.verifyToken(token);
      const jwtPayload: JWTPayload = {
        userId: user.id,
        email: user.email,
        role: user.role,
      };
      return { success: true, user: jwtPayload };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Authentication failed',
      };
    }
  }

  private getClientIP(request: IncomingMessage): string {
    const forwarded = request.headers['x-forwarded-for'];
    const realIP = request.headers['x-real-ip'];

    if (forwarded) {
      return Array.isArray(forwarded) ? forwarded[0] : forwarded.split(',')[0];
    }

    if (realIP) {
      return Array.isArray(realIP) ? realIP[0] : realIP;
    }

    return request.socket.remoteAddress || 'unknown';
  }

  private handleMessage(ws: WebSocketConnection, data: Buffer): void {
    try {
      const message: WebSocketMessage = JSON.parse(data.toString());
      this.totalMessages++;

      // Update last heartbeat
      if (ws.metadata) {
        ws.metadata.lastHeartbeat = Date.now();
      }

      // Handle heartbeat messages
      if (message.type === WebSocketEventType.HEARTBEAT) {
        ws.isAlive = true;
        this.sendMessage(ws, {
          type: WebSocketEventType.HEARTBEAT,
          timestamp: Date.now(),
        });
        return;
      }

      // Handle authentication messages
      if (message.type === WebSocketEventType.AUTHENTICATE) {
        this.handleAuthentication(ws, message);
        return;
      }

      // Require authentication for other messages
      if (!ws.metadata?.isAuthenticated) {
        this.sendMessage(ws, {
          type: WebSocketEventType.ERROR,
          data: { error: 'Authentication required' },
          timestamp: Date.now(),
        });
        return;
      }

      // Route message to appropriate handler
      this.routeMessage(ws, message);
    } catch (error) {
      console.error('Error handling WebSocket message:', error);
      this.sendMessage(ws, {
        type: WebSocketEventType.ERROR,
        data: { error: 'Invalid message format' },
        timestamp: Date.now(),
      });
    }
  }

  private async handleAuthentication(
    ws: WebSocketConnection,
    message: WebSocketMessage
  ): Promise<void> {
    try {
      const token = message.data as string;
      const authResult = await this.authenticateConnection(token);

      if (authResult.success && authResult.user) {
        // Update connection metadata
        ws.metadata!.userId = authResult.user.userId;
        ws.metadata!.username = authResult.user.email;
        ws.metadata!.role = authResult.user.role;
        ws.metadata!.isAuthenticated = true;

        // Add to connections map
        this.connections.set(authResult.user.userId, ws);

        // Update user presence
        this.updateUserPresence(authResult.user.userId, UserStatus.ONLINE);

        // Send success response
        this.sendMessage(ws, {
          type: WebSocketEventType.AUTHENTICATION_SUCCESS,
          data: { user: authResult.user },
          timestamp: Date.now(),
        });

        // Broadcast user online status
        this.broadcastToAll({
          type: WebSocketEventType.USER_ONLINE,
          data: {
            userId: authResult.user.userId,
            username: authResult.user.email,
            status: UserStatus.ONLINE,
            lastSeen: Date.now(),
          },
          timestamp: Date.now(),
        });

        console.log(
          `WebSocket authentication successful: ${authResult.user.email}`
        );
      } else {
        this.sendMessage(ws, {
          type: WebSocketEventType.AUTHENTICATION_FAILED,
          data: { error: authResult.error },
          timestamp: Date.now(),
        });
      }
    } catch (error) {
      console.error('Error during authentication:', error);
      this.sendMessage(ws, {
        type: WebSocketEventType.AUTHENTICATION_FAILED,
        data: { error: 'Authentication failed' },
        timestamp: Date.now(),
      });
    }
  }

  private routeMessage(
    ws: WebSocketConnection,
    message: WebSocketMessage
  ): void {
    // Route to appropriate handler based on message type
    switch (message.type) {
      case WebSocketEventType.JOIN_ROOM:
        this.handleJoinRoom(ws, message);
        break;
      case WebSocketEventType.LEAVE_ROOM:
        this.handleLeaveRoom(ws, message);
        break;
      case WebSocketEventType.ROOM_MESSAGE:
        this.handleRoomMessage(ws, message);
        break;
      case WebSocketEventType.USER_STATUS_UPDATE:
        this.handleUserStatusUpdate(ws, message);
        break;
      // Execution-related messages
      case WebSocketEventType.TERMINAL_CREATE:
      case WebSocketEventType.TERMINAL_COMMAND:
      case WebSocketEventType.TERMINAL_RESIZE:
      case WebSocketEventType.TERMINAL_CLOSE:
      case WebSocketEventType.LOG_STREAM_CREATE:
      case WebSocketEventType.LOG_STREAM_UPDATE:
      case WebSocketEventType.LOG_STREAM_CLOSE:
        this.executionHandler.handleMessage(ws, message);
        break;
      default:
        // Handle unknown message types
        this.sendMessage(ws, {
          type: WebSocketEventType.ERROR,
          data: { error: `Unknown message type: ${message.type}` },
          timestamp: Date.now(),
        });
    }
  }

  private handleJoinRoom(
    ws: WebSocketConnection,
    message: WebSocketMessage
  ): void {
    const roomId = message.data as string;
    const userId = ws.metadata!.userId;

    // Create room if it doesn't exist
    if (!this.rooms.has(roomId)) {
      this.rooms.set(roomId, {
        id: roomId,
        name: roomId,
        type: 'general',
        members: new Set(),
        createdAt: Date.now(),
        createdBy: userId,
      });
    }

    const room = this.rooms.get(roomId)!;
    room.members.add(userId);
    ws.metadata!.rooms.add(roomId);

    // Notify room members
    this.broadcastToRoom(roomId, {
      type: WebSocketEventType.ROOM_USER_JOINED,
      data: {
        userId,
        username: ws.metadata!.username,
        roomId,
      },
      timestamp: Date.now(),
    });

    console.log(`User ${ws.metadata!.username} joined room ${roomId}`);
  }

  private handleLeaveRoom(
    ws: WebSocketConnection,
    message: WebSocketMessage
  ): void {
    const roomId = message.data as string;
    const userId = ws.metadata!.userId;

    const room = this.rooms.get(roomId);
    if (room) {
      room.members.delete(userId);
      ws.metadata!.rooms.delete(roomId);

      // Remove room if empty
      if (room.members.size === 0) {
        this.rooms.delete(roomId);
      } else {
        // Notify remaining room members
        this.broadcastToRoom(roomId, {
          type: WebSocketEventType.ROOM_USER_LEFT,
          data: {
            userId,
            username: ws.metadata!.username,
            roomId,
          },
          timestamp: Date.now(),
        });
      }
    }

    console.log(`User ${ws.metadata!.username} left room ${roomId}`);
  }

  private handleRoomMessage(
    ws: WebSocketConnection,
    message: WebSocketMessage
  ): void {
    const roomId = message.roomId;
    if (!roomId) {
      this.sendMessage(ws, {
        type: WebSocketEventType.ERROR,
        data: { error: 'Room ID required for room messages' },
        timestamp: Date.now(),
      });
      return;
    }

    const room = this.rooms.get(roomId);
    if (!room || !room.members.has(ws.metadata!.userId)) {
      this.sendMessage(ws, {
        type: WebSocketEventType.ERROR,
        data: { error: 'Not a member of this room' },
        timestamp: Date.now(),
      });
      return;
    }

    // Broadcast message to room members
    this.broadcastToRoom(roomId, {
      type: WebSocketEventType.ROOM_MESSAGE,
      data: {
        roomId,
        message: message.data,
        senderId: ws.metadata!.userId,
        senderName: ws.metadata!.username,
        timestamp: Date.now(),
      },
      timestamp: Date.now(),
    });
  }

  private handleUserStatusUpdate(
    ws: WebSocketConnection,
    message: WebSocketMessage
  ): void {
    const status = message.data as UserStatus;
    const userId = ws.metadata!.userId;

    this.updateUserPresence(userId, status);

    // Broadcast status update
    this.broadcastToAll({
      type: WebSocketEventType.USER_STATUS_UPDATE,
      data: {
        userId,
        username: ws.metadata!.username,
        status,
        lastSeen: Date.now(),
      },
      timestamp: Date.now(),
    });
  }

  private handleDisconnection(ws: WebSocketConnection): void {
    if (ws.metadata?.userId) {
      const userId = ws.metadata.userId;
      const username = ws.metadata.username;

      // Cleanup user sessions
      this.executionHandler.cleanupUserSessions(userId);

      // Remove from connections
      this.connections.delete(userId);

      // Update user presence
      this.updateUserPresence(userId, UserStatus.OFFLINE);

      // Leave all rooms
      ws.metadata.rooms.forEach((roomId) => {
        const room = this.rooms.get(roomId);
        if (room) {
          room.members.delete(userId);
          if (room.members.size === 0) {
            this.rooms.delete(roomId);
          } else {
            this.broadcastToRoom(roomId, {
              type: WebSocketEventType.ROOM_USER_LEFT,
              data: { userId, username, roomId },
              timestamp: Date.now(),
            });
          }
        }
      });

      // Broadcast user offline status
      this.broadcastToAll({
        type: WebSocketEventType.USER_OFFLINE,
        data: {
          userId,
          username,
          status: UserStatus.OFFLINE,
          lastSeen: Date.now(),
        },
        timestamp: Date.now(),
      });

      console.log(`WebSocket connection closed: ${username} (${userId})`);
    }
  }

  private handleError(ws: WebSocketConnection, error: Error): void {
    console.error('WebSocket error:', error);
    this.sendMessage(ws, {
      type: WebSocketEventType.ERROR,
      data: { error: 'Connection error occurred' },
      timestamp: Date.now(),
    });
  }

  private updateUserPresence(userId: string, status: UserStatus): void {
    const presence: UserPresence = {
      userId,
      username: this.connections.get(userId)?.metadata?.username || '',
      status,
      lastSeen: Date.now(),
    };

    this.userPresence.set(userId, presence);
  }

  private startHeartbeat(): void {
    this.heartbeatInterval = setInterval(() => {
      this.wss.clients.forEach((ws: WebSocketConnection) => {
        if (ws.isAlive === false) {
          return ws.terminate();
        }

        ws.isAlive = false;
        ws.ping();
      });
    }, this.heartbeatConfig.interval);
  }

  // Public methods for external use
  public sendMessage(ws: WebSocketConnection, message: WebSocketMessage): void {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(message));
    }
  }

  public sendToUser(userId: string, message: WebSocketMessage): void {
    const connection = this.connections.get(userId);
    if (connection) {
      this.sendMessage(connection, message);
    }
  }

  public broadcastToAll(message: WebSocketMessage): void {
    this.wss.clients.forEach((client: WebSocketConnection) => {
      this.sendMessage(client, message);
    });
  }

  public broadcastToRoom(roomId: string, message: WebSocketMessage): void {
    const room = this.rooms.get(roomId);
    if (room) {
      room.members.forEach((userId) => {
        this.sendToUser(userId, message);
      });
    }
  }

  public broadcastToAuthenticated(message: WebSocketMessage): void {
    this.connections.forEach((connection) => {
      if (connection.metadata?.isAuthenticated) {
        this.sendMessage(connection, message);
      }
    });
  }

  public getConnection(userId: string): WebSocketConnection | undefined {
    return this.connections.get(userId);
  }

  public isUserConnected(userId: string): boolean {
    return this.connections.has(userId);
  }

  public getRoomMembers(roomId: string): string[] {
    const room = this.rooms.get(roomId);
    return room ? Array.from(room.members) : [];
  }

  public getUserPresence(userId: string): UserPresence | undefined {
    return this.userPresence.get(userId);
  }

  public getStats(): WebSocketStats {
    return {
      totalConnections: this.connections.size,
      activeConnections: this.wss.clients.size,
      totalRooms: this.rooms.size,
      totalMessages: this.totalMessages,
      uptime: Date.now() - this.serverStartTime,
      memoryUsage: process.memoryUsage(),
    };
  }

  public disconnect(): void {
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
    }

    this.wss.clients.forEach((client) => {
      client.close();
    });

    this.connections.clear();
    this.rooms.clear();
    this.userPresence.clear();
  }
}
