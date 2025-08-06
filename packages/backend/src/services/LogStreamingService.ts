import { spawn, ChildProcess } from 'child_process';
import { WebSocketConnection } from '../websocket/types';
import { logger } from '../utils/logger';

export interface LogStream {
  id: string;
  containerId: string;
  process: ChildProcess | null;
  connection: WebSocketConnection;
  isActive: boolean;
  createdAt: Date;
  lastActivity: Date;
  filters: LogFilter;
  buffer: LogEntry[];
  maxBufferSize: number;
}

export interface LogEntry {
  timestamp: Date;
  level: 'error' | 'warning' | 'info' | 'debug';
  source: string;
  containerId: string;
  message: string;
  metadata?: Record<string, unknown>;
}

export interface LogFilter {
  level?: 'error' | 'warning' | 'info' | 'debug';
  source?: string;
  containerId?: string;
  search?: string;
  limit?: number;
}

export interface LogStreamOptions {
  follow?: boolean;
  tail?: number;
  since?: string;
  until?: string;
  filters?: LogFilter;
  maxBufferSize?: number;
}

export class LogStreamingService {
  private streams: Map<string, LogStream> = new Map();
  private streamCounter: number = 0;

  constructor() {
    this.cleanupInactiveStreams();
  }

  /**
   * Create a new log stream for a container
   */
  public createStream(
    containerId: string,
    connection: WebSocketConnection,
    options: LogStreamOptions = {}
  ): string {
    const streamId = `logstream_${containerId}_${++this.streamCounter}`;
    
    const stream: LogStream = {
      id: streamId,
      containerId,
      process: null,
      connection,
      isActive: true,
      createdAt: new Date(),
      lastActivity: new Date(),
      filters: options.filters || {},
      buffer: [],
      maxBufferSize: options.maxBufferSize || 1000,
    };

    this.streams.set(streamId, stream);
    this.startLogStream(stream, options);

    logger.info('Log stream created', {
      streamId,
      containerId,
      userId: connection.metadata?.userId,
      options,
    });

    return streamId;
  }

  /**
   * Start the log stream process
   */
  private startLogStream(stream: LogStream, options: LogStreamOptions): void {
    const dockerArgs = ['logs'];
    
    if (options.follow) {
      dockerArgs.push('--follow');
    }
    
    if (options.tail) {
      dockerArgs.push('--tail', options.tail.toString());
    }
    
    if (options.since) {
      dockerArgs.push('--since', options.since);
    }
    
    if (options.until) {
      dockerArgs.push('--until', options.until);
    }
    
    dockerArgs.push(stream.containerId);

    const process = spawn('docker', dockerArgs, {
      stdio: ['ignore', 'pipe', 'pipe'],
    });

    stream.process = process;
    this.setupLogProcessHandlers(stream);
  }

  /**
   * Update log filters for a stream
   */
  public updateFilters(streamId: string, filters: LogFilter): boolean {
    const stream = this.streams.get(streamId);
    if (!stream || !stream.isActive) {
      return false;
    }

    stream.filters = { ...stream.filters, ...filters };
    stream.lastActivity = new Date();

    logger.debug('Log stream filters updated', {
      streamId,
      filters,
      userId: stream.connection.metadata?.userId,
    });

    return true;
  }

  /**
   * Stop a log stream
   */
  public stopStream(streamId: string): boolean {
    const stream = this.streams.get(streamId);
    if (!stream) {
      return false;
    }

    try {
      stream.isActive = false;
      
      if (stream.process) {
        stream.process.kill('SIGTERM');
        
        // Force kill after 5 seconds if still running
        setTimeout(() => {
          if (stream.process && !stream.process.killed) {
            stream.process.kill('SIGKILL');
          }
        }, 5000);
      }

      this.streams.delete(streamId);
      
      logger.info('Log stream stopped', {
        streamId,
        containerId: stream.containerId,
        userId: stream.connection.metadata?.userId,
      });

      return true;
    } catch (error) {
      logger.error('Failed to stop log stream', {
        streamId,
        error: error instanceof Error ? error.message : 'Unknown error',
      });
      return false;
    }
  }

  /**
   * Get log stream information
   */
  public getStream(streamId: string): LogStream | undefined {
    return this.streams.get(streamId);
  }

  /**
   * Get all streams for a user
   */
  public getUserStreams(userId: string): LogStream[] {
    return Array.from(this.streams.values()).filter(
      stream => stream.connection.metadata?.userId === userId
    );
  }

  /**
   * Get all streams for a container
   */
  public getContainerStreams(containerId: string): LogStream[] {
    return Array.from(this.streams.values()).filter(
      stream => stream.containerId === containerId
    );
  }

  /**
   * Get log streaming statistics
   */
  public getStats(): {
    totalStreams: number;
    activeStreams: number;
    streamsByContainer: Record<string, number>;
  } {
    const streamsByContainer: Record<string, number> = {};
    
    for (const stream of this.streams.values()) {
      streamsByContainer[stream.containerId] = 
        (streamsByContainer[stream.containerId] || 0) + 1;
    }

    return {
      totalStreams: this.streams.size,
      activeStreams: Array.from(this.streams.values()).filter(s => s.isActive).length,
      streamsByContainer,
    };
  }

  /**
   * Setup process event handlers for a log stream
   */
  private setupLogProcessHandlers(stream: LogStream): void {
    if (!stream.process) return;

    const { process, connection } = stream;

    // Handle stdout (log output)
    process.stdout?.on('data', (data: Buffer) => {
      const logData = data.toString();
      const logEntries = this.parseLogData(logData, stream.containerId);
      
      for (const entry of logEntries) {
        if (this.matchesFilter(entry, stream.filters)) {
          this.addToBuffer(stream, entry);
          this.sendLogEntry(connection, entry);
        }
      }
    });

    // Handle stderr (docker errors)
    process.stderr?.on('data', (data: Buffer) => {
      const errorData = data.toString();
      logger.warn('Docker log command error', {
        streamId: stream.id,
        containerId: stream.containerId,
        error: errorData,
        userId: connection.metadata?.userId,
      });
    });

    // Handle process exit
    process.on('exit', (code: number, signal: string) => {
      stream.isActive = false;
      
      logger.info('Log stream process exited', {
        streamId: stream.id,
        containerId: stream.containerId,
        code,
        signal,
        userId: connection.metadata?.userId,
      });
    });

    // Handle process errors
    process.on('error', (error: Error) => {
      stream.isActive = false;
      
      logger.error('Log stream process error', {
        streamId: stream.id,
        containerId: stream.containerId,
        error: error.message,
        userId: connection.metadata?.userId,
      });
    });
  }

  /**
   * Parse log data from docker logs output
   */
  private parseLogData(logData: string, containerId: string): LogEntry[] {
    const lines = logData.split('\n').filter(line => line.trim());
    const entries: LogEntry[] = [];

    for (const line of lines) {
      try {
        // Try to parse as JSON first (structured logging)
        const jsonMatch = line.match(/^(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}.\d{3}Z)\s+(.+)$/);
        if (jsonMatch) {
          const timestamp = new Date(jsonMatch[1]);
          const jsonData = jsonMatch[2];
          
          try {
            const parsed = JSON.parse(jsonData);
            entries.push({
              timestamp,
              level: parsed.level || 'info',
              source: parsed.source || 'application',
              containerId,
              message: parsed.message || jsonData,
              metadata: parsed.metadata,
            });
            continue;
          } catch {
            // Not valid JSON, treat as plain text
          }
        }

        // Parse as plain text log
        const textMatch = line.match(/^(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}.\d{3}Z)\s+\[(\w+)\]\s+(.+)$/);
        if (textMatch) {
          entries.push({
            timestamp: new Date(textMatch[1]),
            level: this.parseLogLevel(textMatch[2]),
            source: 'application',
            containerId,
            message: textMatch[3],
          });
        } else {
          // Fallback: treat as info level with current timestamp
          entries.push({
            timestamp: new Date(),
            level: 'info',
            source: 'application',
            containerId,
            message: line,
          });
        }
      } catch (error) {
        logger.warn('Failed to parse log line', {
          line: line.substring(0, 100),
          error: error instanceof Error ? error.message : 'Unknown error',
        });
      }
    }

    return entries;
  }

  /**
   * Parse log level from string
   */
  private parseLogLevel(levelStr: string): 'error' | 'warning' | 'info' | 'debug' {
    const level = levelStr.toLowerCase();
    if (level.includes('error') || level.includes('err')) return 'error';
    if (level.includes('warn')) return 'warning';
    if (level.includes('debug')) return 'debug';
    return 'info';
  }

  /**
   * Check if log entry matches filter
   */
  private matchesFilter(entry: LogEntry, filter: LogFilter): boolean {
    if (filter.level && entry.level !== filter.level) {
      return false;
    }
    
    if (filter.source && entry.source !== filter.source) {
      return false;
    }
    
    if (filter.containerId && entry.containerId !== filter.containerId) {
      return false;
    }
    
    if (filter.search && !entry.message.toLowerCase().includes(filter.search.toLowerCase())) {
      return false;
    }
    
    return true;
  }

  /**
   * Add log entry to buffer
   */
  private addToBuffer(stream: LogStream, entry: LogEntry): void {
    stream.buffer.push(entry);
    
    // Maintain buffer size
    if (stream.buffer.length > stream.maxBufferSize) {
      stream.buffer.shift();
    }
  }

  /**
   * Send log entry to WebSocket connection
   */
  private sendLogEntry(connection: WebSocketConnection, entry: LogEntry): void {
    try {
      if (connection.readyState === connection.OPEN) {
        connection.send(JSON.stringify({
          type: 'log_entry',
          data: entry,
          timestamp: Date.now(),
        }));
      }
    } catch (error) {
      logger.error('Failed to send log entry', {
        error: error instanceof Error ? error.message : 'Unknown error',
        userId: connection.metadata?.userId,
      });
    }
  }

  /**
   * Cleanup inactive streams periodically
   */
  private cleanupInactiveStreams(): void {
    setInterval(() => {
      const now = new Date();
      const inactiveThreshold = 30 * 60 * 1000; // 30 minutes

      for (const [streamId, stream] of this.streams.entries()) {
        if (!stream.isActive || 
            (now.getTime() - stream.lastActivity.getTime()) > inactiveThreshold) {
          this.stopStream(streamId);
        }
      }
    }, 5 * 60 * 1000); // Check every 5 minutes
  }

  /**
   * Close all streams for a user
   */
  public closeUserStreams(userId: string): void {
    const userStreams = this.getUserStreams(userId);
    for (const stream of userStreams) {
      this.stopStream(stream.id);
    }
  }

  /**
   * Close all streams for a container
   */
  public closeContainerStreams(containerId: string): void {
    const containerStreams = this.getContainerStreams(containerId);
    for (const stream of containerStreams) {
      this.stopStream(stream.id);
    }
  }
} 