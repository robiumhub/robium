import fs from 'fs';
import path from 'path';

// Log levels
export enum LogLevel {
  ERROR = 0,
  WARN = 1,
  INFO = 2,
  DEBUG = 3,
  TRACE = 4
}

// Log level names for display
export const LOG_LEVEL_NAMES: Record<LogLevel, string> = {
  [LogLevel.ERROR]: 'ERROR',
  [LogLevel.WARN]: 'WARN',
  [LogLevel.INFO]: 'INFO',
  [LogLevel.DEBUG]: 'DEBUG',
  [LogLevel.TRACE]: 'TRACE'
};

// Log entry interface
export interface LogEntry {
  timestamp: string;
  level: LogLevel;
  levelName: string;
  message: string;
  context?: Record<string, unknown>;
  error?: Error;
  requestId?: string;
  userId?: string;
  ip?: string;
  userAgent?: string;
  method?: string;
  url?: string;
  duration?: number;
}

// Logger configuration
export interface LoggerConfig {
  level: LogLevel;
  enableConsole: boolean;
  enableFile: boolean;
  logDir: string;
  maxFileSize: number; // in bytes
  maxFiles: number;
  format: 'json' | 'text';
}

// Default configuration
const DEFAULT_CONFIG: LoggerConfig = {
  level: LogLevel.INFO,
  enableConsole: true,
  enableFile: true,
  logDir: 'logs',
  maxFileSize: 10 * 1024 * 1024, // 10MB
  maxFiles: 5,
  format: 'json'
};

export class Logger {
  private config: LoggerConfig;
  private logStream: fs.WriteStream | null = null;
  private currentLogFile: string | null = null;

  constructor(config: Partial<LoggerConfig> = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.initializeLogDirectory();
    this.rotateLogFile();
  }

  private initializeLogDirectory(): void {
    if (this.config.enableFile) {
      const logDir = path.resolve(this.config.logDir);
      if (!fs.existsSync(logDir)) {
        fs.mkdirSync(logDir, { recursive: true });
      }
    }
  }

  private rotateLogFile(): void {
    if (!this.config.enableFile) return;

    const timestamp = new Date().toISOString().split('T')[0];
    const logFile = path.join(this.config.logDir, `robium-${timestamp}.log`);

    // Close existing stream
    if (this.logStream) {
      this.logStream.end();
    }

    // Create new stream
    this.logStream = fs.createWriteStream(logFile, { flags: 'a' });
    this.currentLogFile = logFile;

    // Check file size and rotate if needed
    this.checkFileSize();
  }

  private checkFileSize(): void {
    if (!this.currentLogFile || !this.logStream) return;

    try {
      const stats = fs.statSync(this.currentLogFile);
      if (stats.size > this.config.maxFileSize) {
        this.rotateLogFile();
      }
    } catch (error) {
      // File doesn't exist or other error, continue with current stream
    }
  }

  private formatLogEntry(entry: LogEntry): string {
    if (this.config.format === 'json') {
      return JSON.stringify(entry) + '\n';
    } else {
      // Text format
      const parts = [
        `[${entry.timestamp}]`,
        `${entry.levelName}`,
        entry.message
      ];

      if (entry.context) {
        parts.push(`Context: ${JSON.stringify(entry.context)}`);
      }

      if (entry.error) {
        parts.push(`Error: ${entry.error.message}`);
        if (entry.error.stack) {
          parts.push(`Stack: ${entry.error.stack}`);
        }
      }

      if (entry.requestId) {
        parts.push(`RequestID: ${entry.requestId}`);
      }

      if (entry.userId) {
        parts.push(`UserID: ${entry.userId}`);
      }

      return parts.join(' | ') + '\n';
    }
  }

  private shouldLog(level: LogLevel): boolean {
    return level <= this.config.level;
  }

  private log(level: LogLevel, message: string, context?: Record<string, unknown>, error?: Error): void {
    if (!this.shouldLog(level)) return;

    const entry: LogEntry = {
      timestamp: new Date().toISOString(),
      level,
      levelName: LOG_LEVEL_NAMES[level],
      message,
      context,
      error
    };

    const formattedEntry = this.formatLogEntry(entry);

    // Console output
    if (this.config.enableConsole) {
      const consoleMethod = level === LogLevel.ERROR ? 'error' : 
                           level === LogLevel.WARN ? 'warn' : 
                           level === LogLevel.INFO ? 'info' : 'log';
      
      console[consoleMethod](formattedEntry.trim());
    }

    // File output
    if (this.config.enableFile && this.logStream) {
      this.logStream.write(formattedEntry);
    }
  }

  // Public logging methods
  error(message: string, context?: Record<string, unknown>, error?: Error): void {
    this.log(LogLevel.ERROR, message, context, error);
  }

  warn(message: string, context?: Record<string, unknown>): void {
    this.log(LogLevel.WARN, message, context);
  }

  info(message: string, context?: Record<string, unknown>): void {
    this.log(LogLevel.INFO, message, context);
  }

  debug(message: string, context?: Record<string, unknown>): void {
    this.log(LogLevel.DEBUG, message, context);
  }

  trace(message: string, context?: Record<string, unknown>): void {
    this.log(LogLevel.TRACE, message, context);
  }

  // Request-specific logging
  logRequest(req: unknown, res: unknown, duration?: number): void {
    const context: Record<string, unknown> = {
      method: (req as { method?: string }).method,
      url: (req as { url?: string }).url,
      ip: (req as { ip?: string; connection?: { remoteAddress?: string } }).ip || (req as { connection?: { remoteAddress?: string } }).connection?.remoteAddress,
      userAgent: (req as { get?: (header: string) => string }).get?.('User-Agent'),
      duration: duration ? `${duration}ms` : undefined
    };

    if ((req as { user?: { userId?: string } }).user?.userId) {
      context.userId = (req as { user: { userId: string } }).user.userId;
    }

    if ((req as { requestId?: string }).requestId) {
      context.requestId = (req as { requestId: string }).requestId;
    }

    this.info('HTTP Request', context);
  }

  logError(error: Error, req?: unknown): void {
    const context: Record<string, unknown> = {};

    if (req) {
      context.method = (req as { method?: string }).method;
      context.url = (req as { url?: string }).url;
      context.ip = (req as { ip?: string; connection?: { remoteAddress?: string } }).ip || (req as { connection?: { remoteAddress?: string } }).connection?.remoteAddress;
      context.userAgent = (req as { get?: (header: string) => string }).get?.('User-Agent');

      if ((req as { user?: { userId?: string } }).user?.userId) {
        context.userId = (req as { user: { userId: string } }).user.userId;
      }

      if ((req as { requestId?: string }).requestId) {
        context.requestId = (req as { requestId: string }).requestId;
      }
    }

    this.error('Application Error', context, error);
  }

  // Cleanup method
  close(): void {
    if (this.logStream) {
      this.logStream.end();
      this.logStream = null;
    }
  }
}

// Create default logger instance
export const logger = new Logger({
  level: (process.env.LOG_LEVEL as keyof typeof LogLevel) ? LogLevel[process.env.LOG_LEVEL as keyof typeof LogLevel] : LogLevel.INFO,
  enableConsole: process.env.NODE_ENV !== 'test',
  enableFile: process.env.NODE_ENV === 'production',
  format: process.env.NODE_ENV === 'production' ? 'json' : 'text'
});

// Graceful shutdown
process.on('SIGINT', () => {
  logger.info('Shutting down logger...');
  logger.close();
  process.exit(0);
});

process.on('SIGTERM', () => {
  logger.info('Shutting down logger...');
  logger.close();
  process.exit(0);
}); 