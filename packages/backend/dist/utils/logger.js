"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.logger = exports.Logger = exports.LOG_LEVEL_NAMES = exports.LogLevel = void 0;
const fs_1 = __importDefault(require("fs"));
const path_1 = __importDefault(require("path"));
// Log levels
var LogLevel;
(function (LogLevel) {
    LogLevel[LogLevel["ERROR"] = 0] = "ERROR";
    LogLevel[LogLevel["WARN"] = 1] = "WARN";
    LogLevel[LogLevel["INFO"] = 2] = "INFO";
    LogLevel[LogLevel["DEBUG"] = 3] = "DEBUG";
    LogLevel[LogLevel["TRACE"] = 4] = "TRACE";
})(LogLevel = exports.LogLevel || (exports.LogLevel = {}));
// Log level names for display
exports.LOG_LEVEL_NAMES = {
    [LogLevel.ERROR]: 'ERROR',
    [LogLevel.WARN]: 'WARN',
    [LogLevel.INFO]: 'INFO',
    [LogLevel.DEBUG]: 'DEBUG',
    [LogLevel.TRACE]: 'TRACE',
};
// Default configuration
const DEFAULT_CONFIG = {
    level: LogLevel.INFO,
    enableConsole: true,
    enableFile: true,
    logDir: 'logs',
    maxFileSize: 10 * 1024 * 1024,
    maxFiles: 5,
    format: 'json',
};
class Logger {
    constructor(config = {}) {
        this.logStream = null;
        this.currentLogFile = null;
        this.config = { ...DEFAULT_CONFIG, ...config };
        this.initializeLogDirectory();
        this.rotateLogFile();
    }
    initializeLogDirectory() {
        if (this.config.enableFile) {
            const logDir = path_1.default.resolve(this.config.logDir);
            if (!fs_1.default.existsSync(logDir)) {
                fs_1.default.mkdirSync(logDir, { recursive: true });
            }
        }
    }
    rotateLogFile() {
        if (!this.config.enableFile)
            return;
        const timestamp = new Date().toISOString().split('T')[0];
        const logFile = path_1.default.join(this.config.logDir, `robium-${timestamp}.log`);
        // Close existing stream
        if (this.logStream) {
            this.logStream.end();
        }
        // Create new stream
        this.logStream = fs_1.default.createWriteStream(logFile, { flags: 'a' });
        this.currentLogFile = logFile;
        // Check file size and rotate if needed
        this.checkFileSize();
    }
    checkFileSize() {
        if (!this.currentLogFile || !this.logStream)
            return;
        try {
            const stats = fs_1.default.statSync(this.currentLogFile);
            if (stats.size > this.config.maxFileSize) {
                this.rotateLogFile();
            }
        }
        catch (error) {
            // File doesn't exist or other error, continue with current stream
        }
    }
    formatLogEntry(entry) {
        if (this.config.format === 'json') {
            return JSON.stringify(entry) + '\n';
        }
        else {
            // Text format
            const parts = [
                `[${entry.timestamp}]`,
                `${entry.levelName}`,
                entry.message,
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
    shouldLog(level) {
        return level <= this.config.level;
    }
    log(level, message, context, error) {
        if (!this.shouldLog(level))
            return;
        const entry = {
            timestamp: new Date().toISOString(),
            level,
            levelName: exports.LOG_LEVEL_NAMES[level],
            message,
            context,
            error,
        };
        const formattedEntry = this.formatLogEntry(entry);
        // Console output
        if (this.config.enableConsole) {
            const consoleMethod = level === LogLevel.ERROR
                ? 'error'
                : level === LogLevel.WARN
                    ? 'warn'
                    : level === LogLevel.INFO
                        ? 'info'
                        : 'log';
            console[consoleMethod](formattedEntry.trim());
        }
        // File output
        if (this.config.enableFile && this.logStream) {
            this.logStream.write(formattedEntry);
        }
    }
    // Public logging methods
    error(message, context, error) {
        this.log(LogLevel.ERROR, message, context, error);
    }
    warn(message, context) {
        this.log(LogLevel.WARN, message, context);
    }
    info(message, context) {
        this.log(LogLevel.INFO, message, context);
    }
    debug(message, context) {
        this.log(LogLevel.DEBUG, message, context);
    }
    trace(message, context) {
        this.log(LogLevel.TRACE, message, context);
    }
    // Request-specific logging
    logRequest(req, res, duration) {
        const context = {
            method: req.method,
            url: req.url,
            ip: req.ip ||
                req.connection
                    ?.remoteAddress,
            userAgent: req.get?.('User-Agent'),
            duration: duration ? `${duration}ms` : undefined,
        };
        if (req.user?.userId) {
            context.userId = req.user.userId;
        }
        if (req.requestId) {
            context.requestId = req.requestId;
        }
        this.info('HTTP Request', context);
    }
    logError(error, req) {
        const context = {};
        if (req) {
            context.method = req.method;
            context.url = req.url;
            context.ip =
                req.ip ||
                    req.connection
                        ?.remoteAddress;
            context.userAgent = req.get?.('User-Agent');
            if (req.user?.userId) {
                context.userId = req.user.userId;
            }
            if (req.requestId) {
                context.requestId = req.requestId;
            }
        }
        this.error('Application Error', context, error);
    }
    // Cleanup method
    close() {
        if (this.logStream) {
            this.logStream.end();
            this.logStream = null;
        }
    }
}
exports.Logger = Logger;
// Create default logger instance
exports.logger = new Logger({
    level: process.env.LOG_LEVEL
        ? LogLevel[process.env.LOG_LEVEL]
        : LogLevel.INFO,
    enableConsole: process.env.NODE_ENV !== 'test',
    enableFile: process.env.NODE_ENV === 'production',
    format: process.env.NODE_ENV === 'production' ? 'json' : 'text',
});
// Graceful shutdown
process.on('SIGINT', () => {
    exports.logger.info('Shutting down logger...');
    exports.logger.close();
    process.exit(0);
});
process.on('SIGTERM', () => {
    exports.logger.info('Shutting down logger...');
    exports.logger.close();
    process.exit(0);
});
