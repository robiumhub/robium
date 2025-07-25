"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.ConnectionManager = void 0;
const ws_1 = require("ws");
const url_1 = require("url");
const types_1 = require("./types");
const AuthService_1 = require("../services/AuthService");
class ConnectionManager {
    constructor(wss, heartbeatConfig = {
        interval: 30000,
        timeout: 60000,
        maxMissedHeartbeats: 2,
    }) {
        this.wss = wss;
        this.connections = new Map();
        this.rooms = new Map();
        this.userPresence = new Map();
        this.heartbeatInterval = null;
        this.totalMessages = 0;
        this.heartbeatConfig = heartbeatConfig;
        this.serverStartTime = Date.now();
        this.setupWebSocketServer();
        this.startHeartbeat();
    }
    setupWebSocketServer() {
        this.wss.on('connection', (ws, request) => {
            this.handleConnection(ws, request);
        });
    }
    async handleConnection(ws, request) {
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
                    this.updateUserPresence(authResult.user.userId, types_1.UserStatus.ONLINE);
                    // Send authentication success message
                    this.sendMessage(ws, {
                        type: types_1.WebSocketEventType.AUTHENTICATION_SUCCESS,
                        data: { user: authResult.user },
                        timestamp: Date.now(),
                    });
                    // Broadcast user online status
                    this.broadcastToAll({
                        type: types_1.WebSocketEventType.USER_ONLINE,
                        data: {
                            userId: authResult.user.userId,
                            username: authResult.user.email,
                            status: types_1.UserStatus.ONLINE,
                            lastSeen: Date.now(),
                        },
                        timestamp: Date.now(),
                    });
                    console.log(`WebSocket connection authenticated: ${authResult.user.email} (${authResult.user.userId})`);
                }
                else {
                    // Send authentication failed message
                    this.sendMessage(ws, {
                        type: types_1.WebSocketEventType.AUTHENTICATION_FAILED,
                        data: { error: authResult.error },
                        timestamp: Date.now(),
                    });
                    console.log(`WebSocket authentication failed: ${authResult.error}`);
                }
            }
            else {
                // Send authentication required message
                this.sendMessage(ws, {
                    type: types_1.WebSocketEventType.AUTHENTICATION_FAILED,
                    data: { error: 'Authentication token required' },
                    timestamp: Date.now(),
                });
            }
            // Set up connection event handlers
            ws.on('message', (data) => {
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
        }
        catch (error) {
            console.error('Error handling WebSocket connection:', error);
            this.sendMessage(ws, {
                type: types_1.WebSocketEventType.ERROR,
                data: { error: 'Connection setup failed' },
                timestamp: Date.now(),
            });
        }
    }
    extractToken(request) {
        try {
            const url = new url_1.URL(request.url || '', `http://${request.headers.host}`);
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
        }
        catch (error) {
            console.error('Error extracting token:', error);
            return null;
        }
    }
    async authenticateConnection(token) {
        try {
            const user = await AuthService_1.AuthService.verifyToken(token);
            const jwtPayload = {
                userId: user.id,
                email: user.email,
                role: user.role,
            };
            return { success: true, user: jwtPayload };
        }
        catch (error) {
            return {
                success: false,
                error: error instanceof Error ? error.message : 'Authentication failed',
            };
        }
    }
    getClientIP(request) {
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
    handleMessage(ws, data) {
        try {
            const message = JSON.parse(data.toString());
            this.totalMessages++;
            // Update last heartbeat
            if (ws.metadata) {
                ws.metadata.lastHeartbeat = Date.now();
            }
            // Handle heartbeat messages
            if (message.type === types_1.WebSocketEventType.HEARTBEAT) {
                ws.isAlive = true;
                this.sendMessage(ws, {
                    type: types_1.WebSocketEventType.HEARTBEAT,
                    timestamp: Date.now(),
                });
                return;
            }
            // Handle authentication messages
            if (message.type === types_1.WebSocketEventType.AUTHENTICATE) {
                this.handleAuthentication(ws, message);
                return;
            }
            // Require authentication for other messages
            if (!ws.metadata?.isAuthenticated) {
                this.sendMessage(ws, {
                    type: types_1.WebSocketEventType.ERROR,
                    data: { error: 'Authentication required' },
                    timestamp: Date.now(),
                });
                return;
            }
            // Route message to appropriate handler
            this.routeMessage(ws, message);
        }
        catch (error) {
            console.error('Error handling WebSocket message:', error);
            this.sendMessage(ws, {
                type: types_1.WebSocketEventType.ERROR,
                data: { error: 'Invalid message format' },
                timestamp: Date.now(),
            });
        }
    }
    async handleAuthentication(ws, message) {
        try {
            const token = message.data;
            const authResult = await this.authenticateConnection(token);
            if (authResult.success && authResult.user) {
                // Update connection metadata
                ws.metadata.userId = authResult.user.userId;
                ws.metadata.username = authResult.user.email;
                ws.metadata.role = authResult.user.role;
                ws.metadata.isAuthenticated = true;
                // Add to connections map
                this.connections.set(authResult.user.userId, ws);
                // Update user presence
                this.updateUserPresence(authResult.user.userId, types_1.UserStatus.ONLINE);
                // Send success response
                this.sendMessage(ws, {
                    type: types_1.WebSocketEventType.AUTHENTICATION_SUCCESS,
                    data: { user: authResult.user },
                    timestamp: Date.now(),
                });
                // Broadcast user online status
                this.broadcastToAll({
                    type: types_1.WebSocketEventType.USER_ONLINE,
                    data: {
                        userId: authResult.user.userId,
                        username: authResult.user.email,
                        status: types_1.UserStatus.ONLINE,
                        lastSeen: Date.now(),
                    },
                    timestamp: Date.now(),
                });
                console.log(`WebSocket authentication successful: ${authResult.user.email}`);
            }
            else {
                this.sendMessage(ws, {
                    type: types_1.WebSocketEventType.AUTHENTICATION_FAILED,
                    data: { error: authResult.error },
                    timestamp: Date.now(),
                });
            }
        }
        catch (error) {
            console.error('Error during authentication:', error);
            this.sendMessage(ws, {
                type: types_1.WebSocketEventType.AUTHENTICATION_FAILED,
                data: { error: 'Authentication failed' },
                timestamp: Date.now(),
            });
        }
    }
    routeMessage(ws, message) {
        // Route to appropriate handler based on message type
        switch (message.type) {
            case types_1.WebSocketEventType.JOIN_ROOM:
                this.handleJoinRoom(ws, message);
                break;
            case types_1.WebSocketEventType.LEAVE_ROOM:
                this.handleLeaveRoom(ws, message);
                break;
            case types_1.WebSocketEventType.ROOM_MESSAGE:
                this.handleRoomMessage(ws, message);
                break;
            case types_1.WebSocketEventType.USER_STATUS_UPDATE:
                this.handleUserStatusUpdate(ws, message);
                break;
            default:
                // Handle unknown message types
                this.sendMessage(ws, {
                    type: types_1.WebSocketEventType.ERROR,
                    data: { error: `Unknown message type: ${message.type}` },
                    timestamp: Date.now(),
                });
        }
    }
    handleJoinRoom(ws, message) {
        const roomId = message.data;
        const userId = ws.metadata.userId;
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
        const room = this.rooms.get(roomId);
        room.members.add(userId);
        ws.metadata.rooms.add(roomId);
        // Notify room members
        this.broadcastToRoom(roomId, {
            type: types_1.WebSocketEventType.ROOM_USER_JOINED,
            data: {
                userId,
                username: ws.metadata.username,
                roomId,
            },
            timestamp: Date.now(),
        });
        console.log(`User ${ws.metadata.username} joined room ${roomId}`);
    }
    handleLeaveRoom(ws, message) {
        const roomId = message.data;
        const userId = ws.metadata.userId;
        const room = this.rooms.get(roomId);
        if (room) {
            room.members.delete(userId);
            ws.metadata.rooms.delete(roomId);
            // Remove room if empty
            if (room.members.size === 0) {
                this.rooms.delete(roomId);
            }
            else {
                // Notify remaining room members
                this.broadcastToRoom(roomId, {
                    type: types_1.WebSocketEventType.ROOM_USER_LEFT,
                    data: {
                        userId,
                        username: ws.metadata.username,
                        roomId,
                    },
                    timestamp: Date.now(),
                });
            }
        }
        console.log(`User ${ws.metadata.username} left room ${roomId}`);
    }
    handleRoomMessage(ws, message) {
        const roomId = message.roomId;
        if (!roomId) {
            this.sendMessage(ws, {
                type: types_1.WebSocketEventType.ERROR,
                data: { error: 'Room ID required for room messages' },
                timestamp: Date.now(),
            });
            return;
        }
        const room = this.rooms.get(roomId);
        if (!room || !room.members.has(ws.metadata.userId)) {
            this.sendMessage(ws, {
                type: types_1.WebSocketEventType.ERROR,
                data: { error: 'Not a member of this room' },
                timestamp: Date.now(),
            });
            return;
        }
        // Broadcast message to room members
        this.broadcastToRoom(roomId, {
            type: types_1.WebSocketEventType.ROOM_MESSAGE,
            data: {
                roomId,
                message: message.data,
                senderId: ws.metadata.userId,
                senderName: ws.metadata.username,
                timestamp: Date.now(),
            },
            timestamp: Date.now(),
        });
    }
    handleUserStatusUpdate(ws, message) {
        const status = message.data;
        const userId = ws.metadata.userId;
        this.updateUserPresence(userId, status);
        // Broadcast status update
        this.broadcastToAll({
            type: types_1.WebSocketEventType.USER_STATUS_UPDATE,
            data: {
                userId,
                username: ws.metadata.username,
                status,
                lastSeen: Date.now(),
            },
            timestamp: Date.now(),
        });
    }
    handleDisconnection(ws) {
        if (ws.metadata?.userId) {
            const userId = ws.metadata.userId;
            const username = ws.metadata.username;
            // Remove from connections
            this.connections.delete(userId);
            // Update user presence
            this.updateUserPresence(userId, types_1.UserStatus.OFFLINE);
            // Leave all rooms
            ws.metadata.rooms.forEach((roomId) => {
                const room = this.rooms.get(roomId);
                if (room) {
                    room.members.delete(userId);
                    if (room.members.size === 0) {
                        this.rooms.delete(roomId);
                    }
                    else {
                        this.broadcastToRoom(roomId, {
                            type: types_1.WebSocketEventType.ROOM_USER_LEFT,
                            data: { userId, username, roomId },
                            timestamp: Date.now(),
                        });
                    }
                }
            });
            // Broadcast user offline status
            this.broadcastToAll({
                type: types_1.WebSocketEventType.USER_OFFLINE,
                data: {
                    userId,
                    username,
                    status: types_1.UserStatus.OFFLINE,
                    lastSeen: Date.now(),
                },
                timestamp: Date.now(),
            });
            console.log(`WebSocket connection closed: ${username} (${userId})`);
        }
    }
    handleError(ws, error) {
        console.error('WebSocket error:', error);
        this.sendMessage(ws, {
            type: types_1.WebSocketEventType.ERROR,
            data: { error: 'Connection error occurred' },
            timestamp: Date.now(),
        });
    }
    updateUserPresence(userId, status) {
        const presence = {
            userId,
            username: this.connections.get(userId)?.metadata?.username || '',
            status,
            lastSeen: Date.now(),
        };
        this.userPresence.set(userId, presence);
    }
    startHeartbeat() {
        this.heartbeatInterval = setInterval(() => {
            this.wss.clients.forEach((ws) => {
                if (ws.isAlive === false) {
                    return ws.terminate();
                }
                ws.isAlive = false;
                ws.ping();
            });
        }, this.heartbeatConfig.interval);
    }
    // Public methods for external use
    sendMessage(ws, message) {
        if (ws.readyState === ws_1.WebSocket.OPEN) {
            ws.send(JSON.stringify(message));
        }
    }
    sendToUser(userId, message) {
        const connection = this.connections.get(userId);
        if (connection) {
            this.sendMessage(connection, message);
        }
    }
    broadcastToAll(message) {
        this.wss.clients.forEach((client) => {
            this.sendMessage(client, message);
        });
    }
    broadcastToRoom(roomId, message) {
        const room = this.rooms.get(roomId);
        if (room) {
            room.members.forEach((userId) => {
                this.sendToUser(userId, message);
            });
        }
    }
    broadcastToAuthenticated(message) {
        this.connections.forEach((connection) => {
            if (connection.metadata?.isAuthenticated) {
                this.sendMessage(connection, message);
            }
        });
    }
    getConnection(userId) {
        return this.connections.get(userId);
    }
    isUserConnected(userId) {
        return this.connections.has(userId);
    }
    getRoomMembers(roomId) {
        const room = this.rooms.get(roomId);
        return room ? Array.from(room.members) : [];
    }
    getUserPresence(userId) {
        return this.userPresence.get(userId);
    }
    getStats() {
        return {
            totalConnections: this.connections.size,
            activeConnections: this.wss.clients.size,
            totalRooms: this.rooms.size,
            totalMessages: this.totalMessages,
            uptime: Date.now() - this.serverStartTime,
            memoryUsage: process.memoryUsage(),
        };
    }
    disconnect() {
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
exports.ConnectionManager = ConnectionManager;
