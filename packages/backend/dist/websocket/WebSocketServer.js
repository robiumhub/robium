"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.WebSocketServer = void 0;
const http_1 = require("http");
const ws_1 = require("ws");
const ConnectionManager_1 = require("./ConnectionManager");
const types_1 = require("./types");
class WebSocketServer {
    constructor(app, config = {
        heartbeat: {
            interval: 30000,
            timeout: 60000,
            maxMissedHeartbeats: 2,
        },
        enableLogging: true,
        maxConnections: 1000,
    }) {
        this.app = app;
        this.isRunning = false;
        this.config = config;
        this.httpServer = (0, http_1.createServer)(app);
        this.wss = new ws_1.WebSocketServer({
            server: this.httpServer,
            path: config.path || '/ws',
        });
        this.connectionManager = new ConnectionManager_1.ConnectionManager(this.wss, config.heartbeat);
    }
    start(port = 8000) {
        return new Promise((resolve, reject) => {
            if (this.isRunning) {
                reject(new Error('WebSocket server is already running'));
                return;
            }
            this.httpServer.listen(port, () => {
                this.isRunning = true;
                console.log(`🚀 WebSocket server started on port ${port}`);
                console.log(`📡 WebSocket endpoint: ws://localhost:${port}${this.config.path || '/ws'}`);
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
    stop() {
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
    getConnectionManager() {
        return this.connectionManager;
    }
    getStats() {
        return this.connectionManager.getStats();
    }
    isServerRunning() {
        return this.isRunning;
    }
    // Utility methods for external use
    sendToUser(userId, message) {
        this.connectionManager.sendToUser(userId, message);
    }
    broadcastToAll(message) {
        this.connectionManager.broadcastToAll(message);
    }
    broadcastToRoom(roomId, message) {
        this.connectionManager.broadcastToRoom(roomId, message);
    }
    broadcastToAuthenticated(message) {
        this.connectionManager.broadcastToAuthenticated(message);
    }
    isUserConnected(userId) {
        return this.connectionManager.isUserConnected(userId);
    }
    getRoomMembers(roomId) {
        return this.connectionManager.getRoomMembers(roomId);
    }
    // Notification helpers
    sendNotification(userId, title, message, type = 'info', data, actionUrl) {
        this.sendToUser(userId, {
            type: types_1.WebSocketEventType.NOTIFICATION,
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
    broadcastNotification(title, message, type = 'info', data, actionUrl) {
        this.broadcastToAuthenticated({
            type: types_1.WebSocketEventType.NOTIFICATION,
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
    sendError(userId, code, message, details) {
        this.sendToUser(userId, {
            type: types_1.WebSocketEventType.ERROR,
            data: {
                code,
                message,
                details,
            },
            timestamp: Date.now(),
        });
    }
    // Data synchronization helpers
    broadcastDataSync(entityType, entityId, action, data, version) {
        this.broadcastToAuthenticated({
            type: types_1.WebSocketEventType.DATA_SYNC,
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
    broadcastProjectUpdate(projectId, userId, action, data) {
        this.broadcastToAuthenticated({
            type: types_1.WebSocketEventType.PROJECT_UPDATE,
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
    broadcastSessionUpdate(sessionId, userId, action, data) {
        this.broadcastToAuthenticated({
            type: types_1.WebSocketEventType.SESSION_UPDATE,
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
    broadcastCollaborationUpdate(sessionId, projectId, userId, action, data) {
        this.broadcastToAuthenticated({
            type: types_1.WebSocketEventType.COLLABORATION_UPDATE,
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
    logServerInfo() {
        console.log('📊 WebSocket Server Configuration:');
        console.log(`   - Heartbeat interval: ${this.config.heartbeat.interval}ms`);
        console.log(`   - Heartbeat timeout: ${this.config.heartbeat.timeout}ms`);
        console.log(`   - Max connections: ${this.config.maxConnections || 'unlimited'}`);
        console.log(`   - Logging enabled: ${this.config.enableLogging}`);
        console.log(`   - WebSocket path: ${this.config.path || '/ws'}`);
    }
    // Health check method
    getHealthStatus() {
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
exports.WebSocketServer = WebSocketServer;
