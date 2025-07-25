import { WebSocket } from 'ws';
import { JWTPayload } from '../types';

// WebSocket message types
export enum WebSocketEventType {
  // Connection events
  CONNECT = 'connect',
  DISCONNECT = 'disconnect',
  RECONNECT = 'reconnect',

  // Authentication events
  AUTHENTICATE = 'authenticate',
  AUTHENTICATION_SUCCESS = 'authentication_success',
  AUTHENTICATION_FAILED = 'authentication_failed',

  // User presence events
  USER_ONLINE = 'user_online',
  USER_OFFLINE = 'user_offline',
  USER_STATUS_UPDATE = 'user_status_update',

  // Room/Channel events
  JOIN_ROOM = 'join_room',
  LEAVE_ROOM = 'leave_room',
  ROOM_MESSAGE = 'room_message',
  ROOM_USER_JOINED = 'room_user_joined',
  ROOM_USER_LEFT = 'room_user_left',

  // Collaboration events
  COLLABORATION_START = 'collaboration_start',
  COLLABORATION_UPDATE = 'collaboration_update',
  COLLABORATION_END = 'collaboration_end',

  // System events
  HEARTBEAT = 'heartbeat',
  ERROR = 'error',
  NOTIFICATION = 'notification',

  // Data synchronization events
  DATA_SYNC = 'data_sync',
  DATA_UPDATE = 'data_update',
  DATA_REQUEST = 'data_request',

  // Project events
  PROJECT_UPDATE = 'project_update',
  PROJECT_USER_JOINED = 'project_user_joined',
  PROJECT_USER_LEFT = 'project_user_left',

  // Session events
  SESSION_START = 'session_start',
  SESSION_UPDATE = 'session_update',
  SESSION_END = 'session_end',
}

// WebSocket message structure
export interface WebSocketMessage<T = unknown> {
  type: WebSocketEventType;
  data?: T;
  timestamp: number;
  userId?: string;
  roomId?: string;
  sessionId?: string;
  error?: string;
}

// Connection metadata
export interface ConnectionMetadata {
  userId: string;
  username: string;
  role: string;
  connectedAt: number;
  lastHeartbeat: number;
  rooms: Set<string>;
  isAuthenticated: boolean;
  userAgent?: string;
  ip?: string;
}

// Room information
export interface Room {
  id: string;
  name: string;
  type: 'project' | 'session' | 'general';
  members: Set<string>;
  createdAt: number;
  createdBy: string;
  metadata?: Record<string, unknown>;
}

// User status
export enum UserStatus {
  ONLINE = 'online',
  OFFLINE = 'offline',
  AWAY = 'away',
  BUSY = 'busy',
  IN_SESSION = 'in_session',
}

// User presence information
export interface UserPresence {
  userId: string;
  username: string;
  status: UserStatus;
  lastSeen: number;
  currentRoom?: string;
  isTyping?: boolean;
}

// Collaboration session
export interface CollaborationSession {
  id: string;
  projectId: string;
  participants: string[];
  startedAt: number;
  lastActivity: number;
  metadata?: Record<string, unknown>;
}

// WebSocket connection with metadata
export interface WebSocketConnection extends WebSocket {
  metadata?: ConnectionMetadata;
  isAlive?: boolean;
}

// Event handler function type
export type EventHandler<T = unknown> = (
  connection: WebSocketConnection,
  message: WebSocketMessage<T>,
  connections: Map<string, WebSocketConnection>
) => void | Promise<void>;

// Authentication result
export interface AuthenticationResult {
  success: boolean;
  user?: JWTPayload;
  error?: string;
}

// Heartbeat configuration
export interface HeartbeatConfig {
  interval: number; // milliseconds
  timeout: number; // milliseconds
  maxMissedHeartbeats: number;
}

// WebSocket server configuration
export interface WebSocketServerConfig {
  port?: number;
  path?: string;
  heartbeat: HeartbeatConfig;
  maxConnections?: number;
  enableLogging?: boolean;
  cors?: {
    origin: string | string[];
    credentials: boolean;
  };
}

// Room message data
export interface RoomMessageData {
  roomId: string;
  message: string;
  senderId: string;
  senderName: string;
  timestamp: number;
}

// Collaboration update data
export interface CollaborationUpdateData {
  sessionId: string;
  projectId: string;
  userId: string;
  action: 'cursor_move' | 'selection_change' | 'content_change' | 'comment_add';
  data: unknown;
  timestamp: number;
}

// Data synchronization data
export interface DataSyncData {
  entityType: 'project' | 'session' | 'user';
  entityId: string;
  action: 'create' | 'update' | 'delete';
  data: unknown;
  version: number;
  timestamp: number;
}

// Project update data
export interface ProjectUpdateData {
  projectId: string;
  userId: string;
  action: 'created' | 'updated' | 'deleted' | 'member_added' | 'member_removed';
  data: unknown;
  timestamp: number;
}

// Session update data
export interface SessionUpdateData {
  sessionId: string;
  userId: string;
  action: 'started' | 'updated' | 'ended' | 'user_joined' | 'user_left';
  data: unknown;
  timestamp: number;
}

// Error message data
export interface ErrorMessageData {
  code: string;
  message: string;
  details?: unknown;
}

// Notification data
export interface NotificationData {
  type: 'info' | 'success' | 'warning' | 'error';
  title: string;
  message: string;
  data?: unknown;
  actionUrl?: string;
}

// WebSocket statistics
export interface WebSocketStats {
  totalConnections: number;
  activeConnections: number;
  totalRooms: number;
  totalMessages: number;
  uptime: number;
  memoryUsage: NodeJS.MemoryUsage;
}
