import { spawn, ChildProcess } from 'child_process';
import { WebSocketConnection } from '../websocket/types';
import { logger } from '../utils/logger';

export interface TerminalSession {
  id: string;
  containerId: string;
  process: ChildProcess;
  connection: WebSocketConnection;
  isActive: boolean;
  createdAt: Date;
  lastActivity: Date;
  buffer: string[];
  maxBufferSize: number;
}

export interface TerminalCommand {
  command: string;
  args?: string[];
  cwd?: string;
  env?: Record<string, string>;
}

export interface TerminalOutput {
  type: 'stdout' | 'stderr' | 'error';
  data: string;
  timestamp: number;
}

export class TerminalService {
  private sessions: Map<string, TerminalSession> = new Map();
  private sessionCounter: number = 0;

  constructor() {
    this.cleanupInactiveSessions();
  }

  /**
   * Create a new terminal session for a container
   */
  public createSession(
    containerId: string,
    connection: WebSocketConnection,
    options: {
      shell?: string;
      cwd?: string;
      env?: Record<string, string>;
      maxBufferSize?: number;
    } = {}
  ): string {
    const sessionId = `terminal_${containerId}_${++this.sessionCounter}`;
    
    // Execute docker exec to create a shell in the container
    const dockerCommand = 'docker';
    const dockerArgs = [
      'exec',
      '-it',
      containerId,
      options.shell || '/bin/bash'
    ];

    const childProcess = spawn(dockerCommand, dockerArgs, {
      stdio: ['pipe', 'pipe', 'pipe'],
      env: {
        ...process.env,
        ...options.env,
      },
    });

    const session: TerminalSession = {
      id: sessionId,
      containerId,
      process: childProcess,
      connection,
      isActive: true,
      createdAt: new Date(),
      lastActivity: new Date(),
      buffer: [],
      maxBufferSize: options.maxBufferSize || 1000,
    };

    this.sessions.set(sessionId, session);
    this.setupProcessHandlers(session);

    logger.info('Terminal session created', {
      sessionId,
      containerId,
      userId: connection.metadata?.userId,
    });

    return sessionId;
  }

  /**
   * Execute a command in a terminal session
   */
  public executeCommand(sessionId: string, command: string): boolean {
    const session = this.sessions.get(sessionId);
    if (!session || !session.isActive) {
      return false;
    }

    try {
      if (session.process.stdin) {
        session.process.stdin.write(command + '\n');
      }
      session.lastActivity = new Date();
      
      logger.debug('Command executed in terminal', {
        sessionId,
        command: command.substring(0, 100), // Log first 100 chars for security
        userId: session.connection.metadata?.userId,
      });

      return true;
    } catch (error) {
      logger.error('Failed to execute command in terminal', {
        sessionId,
        error: error instanceof Error ? error.message : 'Unknown error',
      });
      return false;
    }
  }

  /**
   * Resize terminal
   */
  public resizeTerminal(sessionId: string, cols: number, rows: number): boolean {
    const session = this.sessions.get(sessionId);
    if (!session || !session.isActive) {
      return false;
    }

    try {
      // Send SIGWINCH signal to resize the terminal
      session.process.kill('SIGWINCH');
      return true;
    } catch (error) {
      logger.error('Failed to resize terminal', {
        sessionId,
        cols,
        rows,
        error: error instanceof Error ? error.message : 'Unknown error',
      });
      return false;
    }
  }

  /**
   * Close a terminal session
   */
  public closeSession(sessionId: string): boolean {
    const session = this.sessions.get(sessionId);
    if (!session) {
      return false;
    }

    try {
      session.isActive = false;
      session.process.kill('SIGTERM');
      
      // Force kill after 5 seconds if still running
      setTimeout(() => {
        if (session.process.killed === false) {
          session.process.kill('SIGKILL');
        }
      }, 5000);

      this.sessions.delete(sessionId);
      
      logger.info('Terminal session closed', {
        sessionId,
        containerId: session.containerId,
        userId: session.connection.metadata?.userId,
      });

      return true;
    } catch (error) {
      logger.error('Failed to close terminal session', {
        sessionId,
        error: error instanceof Error ? error.message : 'Unknown error',
      });
      return false;
    }
  }

  /**
   * Get terminal session information
   */
  public getSession(sessionId: string): TerminalSession | undefined {
    return this.sessions.get(sessionId);
  }

  /**
   * Get all sessions for a user
   */
  public getUserSessions(userId: string): TerminalSession[] {
    return Array.from(this.sessions.values()).filter(
      session => session.connection.metadata?.userId === userId
    );
  }

  /**
   * Get all sessions for a container
   */
  public getContainerSessions(containerId: string): TerminalSession[] {
    return Array.from(this.sessions.values()).filter(
      session => session.containerId === containerId
    );
  }

  /**
   * Get terminal statistics
   */
  public getStats(): {
    totalSessions: number;
    activeSessions: number;
    sessionsByContainer: Record<string, number>;
  } {
    const sessionsByContainer: Record<string, number> = {};
    
    for (const session of this.sessions.values()) {
      sessionsByContainer[session.containerId] = 
        (sessionsByContainer[session.containerId] || 0) + 1;
    }

    return {
      totalSessions: this.sessions.size,
      activeSessions: Array.from(this.sessions.values()).filter(s => s.isActive).length,
      sessionsByContainer,
    };
  }

  /**
   * Setup process event handlers for a terminal session
   */
  private setupProcessHandlers(session: TerminalSession): void {
    const { process, connection } = session;

    // Handle stdout
    process.stdout?.on('data', (data: Buffer) => {
      const output = data.toString();
      this.addToBuffer(session, output);
      this.sendOutput(connection, {
        type: 'stdout',
        data: output,
        timestamp: Date.now(),
      });
    });

    // Handle stderr
    process.stderr?.on('data', (data: Buffer) => {
      const output = data.toString();
      this.addToBuffer(session, output);
      this.sendOutput(connection, {
        type: 'stderr',
        data: output,
        timestamp: Date.now(),
      });
    });

    // Handle process exit
    process.on('exit', (code: number, signal: string) => {
      session.isActive = false;
      this.sendOutput(connection, {
        type: 'error',
        data: `Process exited with code ${code}${signal ? ` (signal: ${signal})` : ''}`,
        timestamp: Date.now(),
      });

      logger.info('Terminal process exited', {
        sessionId: session.id,
        containerId: session.containerId,
        code,
        signal,
        userId: connection.metadata?.userId,
      });
    });

    // Handle process errors
    process.on('error', (error: Error) => {
      session.isActive = false;
      this.sendOutput(connection, {
        type: 'error',
        data: `Process error: ${error.message}`,
        timestamp: Date.now(),
      });

      logger.error('Terminal process error', {
        sessionId: session.id,
        containerId: session.containerId,
        error: error.message,
        userId: connection.metadata?.userId,
      });
    });
  }

  /**
   * Add output to session buffer
   */
  private addToBuffer(session: TerminalSession, output: string): void {
    session.buffer.push(output);
    
    // Maintain buffer size
    if (session.buffer.length > session.maxBufferSize) {
      session.buffer.shift();
    }
  }

  /**
   * Send output to WebSocket connection
   */
  private sendOutput(connection: WebSocketConnection, output: TerminalOutput): void {
    try {
      if (connection.readyState === connection.OPEN) {
        connection.send(JSON.stringify({
          type: 'terminal_output',
          data: output,
          timestamp: Date.now(),
        }));
      }
    } catch (error) {
      logger.error('Failed to send terminal output', {
        error: error instanceof Error ? error.message : 'Unknown error',
        userId: connection.metadata?.userId,
      });
    }
  }

  /**
   * Cleanup inactive sessions periodically
   */
  private cleanupInactiveSessions(): void {
    setInterval(() => {
      const now = new Date();
      const inactiveThreshold = 30 * 60 * 1000; // 30 minutes

      for (const [sessionId, session] of this.sessions.entries()) {
        if (!session.isActive || 
            (now.getTime() - session.lastActivity.getTime()) > inactiveThreshold) {
          this.closeSession(sessionId);
        }
      }
    }, 5 * 60 * 1000); // Check every 5 minutes
  }

  /**
   * Close all sessions for a user
   */
  public closeUserSessions(userId: string): void {
    const userSessions = this.getUserSessions(userId);
    for (const session of userSessions) {
      this.closeSession(session.id);
    }
  }

  /**
   * Close all sessions for a container
   */
  public closeContainerSessions(containerId: string): void {
    const containerSessions = this.getContainerSessions(containerId);
    for (const session of containerSessions) {
      this.closeSession(session.id);
    }
  }
} 