import { WebSocketConnection, WebSocketMessage, WebSocketEventType } from './types';
import { TerminalService } from '../services/TerminalService';
import { LogStreamingService } from '../services/LogStreamingService';
import { logger } from '../utils/logger';

export class ExecutionHandler {
  private terminalService: TerminalService;
  private logStreamingService: LogStreamingService;

  constructor() {
    this.terminalService = new TerminalService();
    this.logStreamingService = new LogStreamingService();
  }

  /**
   * Handle execution-related WebSocket messages
   */
  public handleMessage(ws: WebSocketConnection, message: WebSocketMessage): void {
    switch (message.type) {
      case WebSocketEventType.TERMINAL_CREATE:
        this.handleTerminalCreate(ws, message);
        break;
      case WebSocketEventType.TERMINAL_COMMAND:
        this.handleTerminalCommand(ws, message);
        break;
      case WebSocketEventType.TERMINAL_RESIZE:
        this.handleTerminalResize(ws, message);
        break;
      case WebSocketEventType.TERMINAL_CLOSE:
        this.handleTerminalClose(ws, message);
        break;
      case WebSocketEventType.LOG_STREAM_CREATE:
        this.handleLogStreamCreate(ws, message);
        break;
      case WebSocketEventType.LOG_STREAM_UPDATE:
        this.handleLogStreamUpdate(ws, message);
        break;
      case WebSocketEventType.LOG_STREAM_CLOSE:
        this.handleLogStreamClose(ws, message);
        break;
      default:
        logger.warn('Unknown execution message type', {
          type: message.type,
          userId: ws.metadata?.userId,
        });
    }
  }

  /**
   * Handle terminal creation request
   */
  private handleTerminalCreate(ws: WebSocketConnection, message: WebSocketMessage): void {
    try {
      const data = message.data as any;
      const { containerId, shell, cwd, env, maxBufferSize } = data;

      if (!containerId) {
        this.sendError(ws, 'MISSING_CONTAINER_ID', 'Container ID is required');
        return;
      }

      const sessionId = this.terminalService.createSession(containerId, ws, {
        shell,
        cwd,
        env,
        maxBufferSize,
      });

      // Send success response
      this.sendMessage(ws, {
        type: WebSocketEventType.TERMINAL_CREATE,
        data: {
          sessionId,
          containerId,
          status: 'created',
        },
        timestamp: Date.now(),
      });

      logger.info('Terminal session created via WebSocket', {
        sessionId,
        containerId,
        userId: ws.metadata?.userId,
      });
    } catch (error) {
      logger.error('Failed to create terminal session', {
        error: error instanceof Error ? error.message : 'Unknown error',
        userId: ws.metadata?.userId,
      });
      this.sendError(ws, 'TERMINAL_CREATE_FAILED', 'Failed to create terminal session');
    }
  }

  /**
   * Handle terminal command execution
   */
  private handleTerminalCommand(ws: WebSocketConnection, message: WebSocketMessage): void {
    try {
      const data = message.data as any;
      const { sessionId, command } = data;

      if (!sessionId || !command) {
        this.sendError(ws, 'MISSING_PARAMETERS', 'Session ID and command are required');
        return;
      }

      const success = this.terminalService.executeCommand(sessionId, command);

      if (success) {
        // Send acknowledgment
        this.sendMessage(ws, {
          type: WebSocketEventType.TERMINAL_COMMAND,
          data: {
            sessionId,
            command: command.substring(0, 100), // Truncate for security
            status: 'executed',
          },
          timestamp: Date.now(),
        });
      } else {
        this.sendError(ws, 'COMMAND_EXECUTION_FAILED', 'Failed to execute command');
      }
    } catch (error) {
      logger.error('Failed to execute terminal command', {
        error: error instanceof Error ? error.message : 'Unknown error',
        userId: ws.metadata?.userId,
      });
      this.sendError(ws, 'COMMAND_EXECUTION_FAILED', 'Failed to execute command');
    }
  }

  /**
   * Handle terminal resize request
   */
  private handleTerminalResize(ws: WebSocketConnection, message: WebSocketMessage): void {
    try {
      const data = message.data as any;
      const { sessionId, cols, rows } = data;

      if (!sessionId || cols === undefined || rows === undefined) {
        this.sendError(ws, 'MISSING_PARAMETERS', 'Session ID, cols, and rows are required');
        return;
      }

      const success = this.terminalService.resizeTerminal(sessionId, cols, rows);

      if (success) {
        this.sendMessage(ws, {
          type: WebSocketEventType.TERMINAL_RESIZE,
          data: {
            sessionId,
            cols,
            rows,
            status: 'resized',
          },
          timestamp: Date.now(),
        });
      } else {
        this.sendError(ws, 'TERMINAL_RESIZE_FAILED', 'Failed to resize terminal');
      }
    } catch (error) {
      logger.error('Failed to resize terminal', {
        error: error instanceof Error ? error.message : 'Unknown error',
        userId: ws.metadata?.userId,
      });
      this.sendError(ws, 'TERMINAL_RESIZE_FAILED', 'Failed to resize terminal');
    }
  }

  /**
   * Handle terminal close request
   */
  private handleTerminalClose(ws: WebSocketConnection, message: WebSocketMessage): void {
    try {
      const data = message.data as any;
      const { sessionId } = data;

      if (!sessionId) {
        this.sendError(ws, 'MISSING_SESSION_ID', 'Session ID is required');
        return;
      }

      const success = this.terminalService.closeSession(sessionId);

      if (success) {
        this.sendMessage(ws, {
          type: WebSocketEventType.TERMINAL_CLOSE,
          data: {
            sessionId,
            status: 'closed',
          },
          timestamp: Date.now(),
        });
      } else {
        this.sendError(ws, 'TERMINAL_CLOSE_FAILED', 'Failed to close terminal session');
      }
    } catch (error) {
      logger.error('Failed to close terminal session', {
        error: error instanceof Error ? error.message : 'Unknown error',
        userId: ws.metadata?.userId,
      });
      this.sendError(ws, 'TERMINAL_CLOSE_FAILED', 'Failed to close terminal session');
    }
  }

  /**
   * Handle log stream creation request
   */
  private handleLogStreamCreate(ws: WebSocketConnection, message: WebSocketMessage): void {
    try {
      const data = message.data as any;
      const { containerId, follow, tail, since, until, filters, maxBufferSize } = data;

      if (!containerId) {
        this.sendError(ws, 'MISSING_CONTAINER_ID', 'Container ID is required');
        return;
      }

      const streamId = this.logStreamingService.createStream(containerId, ws, {
        follow,
        tail,
        since,
        until,
        filters,
        maxBufferSize,
      });

      // Send success response
      this.sendMessage(ws, {
        type: WebSocketEventType.LOG_STREAM_CREATE,
        data: {
          streamId,
          containerId,
          status: 'created',
        },
        timestamp: Date.now(),
      });

      logger.info('Log stream created via WebSocket', {
        streamId,
        containerId,
        userId: ws.metadata?.userId,
      });
    } catch (error) {
      logger.error('Failed to create log stream', {
        error: error instanceof Error ? error.message : 'Unknown error',
        userId: ws.metadata?.userId,
      });
      this.sendError(ws, 'LOG_STREAM_CREATE_FAILED', 'Failed to create log stream');
    }
  }

  /**
   * Handle log stream filter update
   */
  private handleLogStreamUpdate(ws: WebSocketConnection, message: WebSocketMessage): void {
    try {
      const data = message.data as any;
      const { streamId, filters } = data;

      if (!streamId || !filters) {
        this.sendError(ws, 'MISSING_PARAMETERS', 'Stream ID and filters are required');
        return;
      }

      const success = this.logStreamingService.updateFilters(streamId, filters);

      if (success) {
        this.sendMessage(ws, {
          type: WebSocketEventType.LOG_STREAM_UPDATE,
          data: {
            streamId,
            filters,
            status: 'updated',
          },
          timestamp: Date.now(),
        });
      } else {
        this.sendError(ws, 'LOG_STREAM_UPDATE_FAILED', 'Failed to update log stream filters');
      }
    } catch (error) {
      logger.error('Failed to update log stream filters', {
        error: error instanceof Error ? error.message : 'Unknown error',
        userId: ws.metadata?.userId,
      });
      this.sendError(ws, 'LOG_STREAM_UPDATE_FAILED', 'Failed to update log stream filters');
    }
  }

  /**
   * Handle log stream close request
   */
  private handleLogStreamClose(ws: WebSocketConnection, message: WebSocketMessage): void {
    try {
      const data = message.data as any;
      const { streamId } = data;

      if (!streamId) {
        this.sendError(ws, 'MISSING_STREAM_ID', 'Stream ID is required');
        return;
      }

      const success = this.logStreamingService.stopStream(streamId);

      if (success) {
        this.sendMessage(ws, {
          type: WebSocketEventType.LOG_STREAM_CLOSE,
          data: {
            streamId,
            status: 'closed',
          },
          timestamp: Date.now(),
        });
      } else {
        this.sendError(ws, 'LOG_STREAM_CLOSE_FAILED', 'Failed to close log stream');
      }
    } catch (error) {
      logger.error('Failed to close log stream', {
        error: error instanceof Error ? error.message : 'Unknown error',
        userId: ws.metadata?.userId,
      });
      this.sendError(ws, 'LOG_STREAM_CLOSE_FAILED', 'Failed to close log stream');
    }
  }

  /**
   * Send message to WebSocket connection
   */
  private sendMessage(ws: WebSocketConnection, message: WebSocketMessage): void {
    try {
      if (ws.readyState === ws.OPEN) {
        ws.send(JSON.stringify(message));
      }
    } catch (error) {
      logger.error('Failed to send WebSocket message', {
        error: error instanceof Error ? error.message : 'Unknown error',
        userId: ws.metadata?.userId,
      });
    }
  }

  /**
   * Send error message to WebSocket connection
   */
  private sendError(ws: WebSocketConnection, code: string, message: string): void {
    this.sendMessage(ws, {
      type: WebSocketEventType.ERROR,
      data: {
        code,
        message,
      },
      timestamp: Date.now(),
    });
  }

  /**
   * Get terminal service instance
   */
  public getTerminalService(): TerminalService {
    return this.terminalService;
  }

  /**
   * Get log streaming service instance
   */
  public getLogStreamingService(): LogStreamingService {
    return this.logStreamingService;
  }

  /**
   * Cleanup user sessions when they disconnect
   */
  public cleanupUserSessions(userId: string): void {
    this.terminalService.closeUserSessions(userId);
    this.logStreamingService.closeUserStreams(userId);
  }

  /**
   * Get execution statistics
   */
  public getStats(): {
    terminal: ReturnType<TerminalService['getStats']>;
    logStreaming: ReturnType<LogStreamingService['getStats']>;
  } {
    return {
      terminal: this.terminalService.getStats(),
      logStreaming: this.logStreamingService.getStats(),
    };
  }
} 