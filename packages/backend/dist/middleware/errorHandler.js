"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.ErrorHandler = exports.setupErrorHandlers = exports.handleUncaughtException = exports.handleUnhandledRejection = exports.gracefulShutdown = exports.notFoundHandler = exports.globalErrorHandler = exports.asyncHandler = exports.requestTiming = exports.addRequestId = void 0;
const logger_1 = require("../utils/logger");
const errors_1 = require("../utils/errors");
// Request ID middleware to track requests
const addRequestId = (req, res, next) => {
    req.requestId = generateRequestId();
    res.setHeader('X-Request-ID', req.requestId);
    next();
};
exports.addRequestId = addRequestId;
// Generate unique request ID
function generateRequestId() {
    return `req_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
}
// Request timing middleware
const requestTiming = (req, res, next) => {
    const start = Date.now();
    res.on('finish', () => {
        const duration = Date.now() - start;
        logger_1.logger.logRequest(req, res, duration);
    });
    next();
};
exports.requestTiming = requestTiming;
// Async error wrapper to catch async errors
const asyncHandler = (fn) => {
    return (req, res, next) => {
        Promise.resolve(fn(req, res, next)).catch(next);
    };
};
exports.asyncHandler = asyncHandler;
// Global error handler middleware
const globalErrorHandler = (error, req, res, _next // eslint-disable-line @typescript-eslint/no-unused-vars
) => {
    // Log the error
    logger_1.logger.logError(error, req);
    // Handle different types of errors
    if (error instanceof errors_1.ValidationError) {
        handleValidationError(error, req, res);
    }
    else if (error instanceof errors_1.NotFoundError) {
        handleNotFoundError(error, req, res);
    }
    else if (error instanceof errors_1.ConflictError) {
        handleConflictError(error, req, res);
    }
    else if (error instanceof errors_1.UnauthorizedError) {
        handleUnauthorizedError(error, req, res);
    }
    else if (error instanceof errors_1.ForbiddenError) {
        handleForbiddenError(error, req, res);
    }
    else if (error instanceof errors_1.DatabaseError) {
        handleDatabaseError(error, req, res);
    }
    else if (error instanceof errors_1.AppError) {
        handleAppError(error, req, res);
    }
    else {
        handleUnknownError(error, req, res);
    }
};
exports.globalErrorHandler = globalErrorHandler;
// Specific error handlers
function handleValidationError(error, req, res) {
    const response = {
        success: false,
        error: 'Validation Error',
        message: error.message,
        details: error.errors,
        requestId: req.requestId,
        timestamp: new Date().toISOString(),
    };
    res.status(400).json(response);
}
function handleNotFoundError(error, req, res) {
    const response = {
        success: false,
        error: 'Not Found',
        message: error.message,
        requestId: req.requestId,
        timestamp: new Date().toISOString(),
    };
    res.status(404).json(response);
}
function handleConflictError(error, req, res) {
    const response = {
        success: false,
        error: 'Conflict',
        message: error.message,
        requestId: req.requestId,
        timestamp: new Date().toISOString(),
    };
    res.status(409).json(response);
}
function handleUnauthorizedError(error, req, res) {
    const response = {
        success: false,
        error: 'Unauthorized',
        message: error.message,
        requestId: req.requestId,
        timestamp: new Date().toISOString(),
    };
    res.status(401).json(response);
}
function handleForbiddenError(error, req, res) {
    const response = {
        success: false,
        error: 'Forbidden',
        message: error.message,
        requestId: req.requestId,
        timestamp: new Date().toISOString(),
    };
    res.status(403).json(response);
}
function handleDatabaseError(error, req, res) {
    const response = {
        success: false,
        error: 'Database Error',
        message: error.message,
        code: error.code,
        requestId: req.requestId,
        timestamp: new Date().toISOString(),
    };
    res.status(500).json(response);
}
function handleAppError(error, req, res) {
    const response = {
        success: false,
        error: error.name,
        message: error.message,
        requestId: req.requestId,
        timestamp: new Date().toISOString(),
    };
    res.status(error.status).json(response);
}
function handleUnknownError(error, req, res) {
    const isDevelopment = process.env.NODE_ENV === 'development';
    const response = {
        success: false,
        error: 'Internal Server Error',
        message: isDevelopment ? error.message : 'An unexpected error occurred',
        requestId: req.requestId,
        timestamp: new Date().toISOString(),
    };
    // Include stack trace in development
    if (isDevelopment && error.stack) {
        response.stack = error.stack;
    }
    res.status(500).json(response);
}
// 404 handler for unmatched routes
const notFoundHandler = (req, res) => {
    const response = {
        success: false,
        error: 'Route Not Found',
        message: `Cannot ${req.method} ${req.originalUrl}`,
        requestId: req.requestId,
        timestamp: new Date().toISOString(),
    };
    res.status(404).json(response);
};
exports.notFoundHandler = notFoundHandler;
// Graceful shutdown handler
const gracefulShutdown = (server, signal) => {
    logger_1.logger.info(`Received ${signal}. Starting graceful shutdown...`);
    if (typeof server === 'object' &&
        server !== null &&
        'close' in server &&
        typeof server.close === 'function') {
        server.close(() => {
            logger_1.logger.info('HTTP server closed');
            process.exit(0);
        });
    }
    // Force shutdown after 30 seconds
    setTimeout(() => {
        logger_1.logger.error('Forced shutdown after timeout');
        process.exit(1);
    }, 30000);
};
exports.gracefulShutdown = gracefulShutdown;
// Unhandled rejection handler
const handleUnhandledRejection = (reason, promise) => {
    logger_1.logger.error('Unhandled Rejection at:', { reason, promise });
    process.exit(1);
};
exports.handleUnhandledRejection = handleUnhandledRejection;
// Uncaught exception handler
const handleUncaughtException = (error) => {
    logger_1.logger.error('Uncaught Exception:', {}, error);
    process.exit(1);
};
exports.handleUncaughtException = handleUncaughtException;
// Setup global error handlers
const setupErrorHandlers = () => {
    process.on('unhandledRejection', exports.handleUnhandledRejection);
    process.on('uncaughtException', exports.handleUncaughtException);
};
exports.setupErrorHandlers = setupErrorHandlers;
// Export all error handling utilities
exports.ErrorHandler = {
    addRequestId: exports.addRequestId,
    requestTiming: exports.requestTiming,
    asyncHandler: exports.asyncHandler,
    globalErrorHandler: exports.globalErrorHandler,
    notFoundHandler: exports.notFoundHandler,
    gracefulShutdown: exports.gracefulShutdown,
    setupErrorHandlers: exports.setupErrorHandlers,
};
