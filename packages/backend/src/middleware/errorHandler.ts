import { Request, Response, NextFunction } from 'express';
import { logger } from '../utils/logger';
import {
  AppError,
  ValidationError,
  NotFoundError,
  ConflictError,
  UnauthorizedError,
  ForbiddenError,
  DatabaseError,
} from '../utils/errors';

// Request ID middleware to track requests
export const addRequestId = (
  req: Request,
  res: Response,
  next: NextFunction
): void => {
  req.requestId = generateRequestId();
  res.setHeader('X-Request-ID', req.requestId);
  next();
};

// Generate unique request ID
function generateRequestId(): string {
  return `req_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
}

// Request timing middleware
export const requestTiming = (
  req: Request,
  res: Response,
  next: NextFunction
): void => {
  const start = Date.now();

  res.on('finish', () => {
    const duration = Date.now() - start;
    logger.logRequest(req, res, duration);
  });

  next();
};

// Async error wrapper to catch async errors
export const asyncHandler = (
  fn: (req: Request, res: Response, next: NextFunction) => Promise<void> | void
) => {
  return (req: Request, res: Response, next: NextFunction) => {
    Promise.resolve(fn(req, res, next)).catch(next);
  };
};

// Global error handler middleware
export const globalErrorHandler = (
  error: Error,
  req: Request,
  res: Response,
  _next: NextFunction // eslint-disable-line @typescript-eslint/no-unused-vars
): void => {
  // Log the error
  logger.logError(error, req);

  // Handle different types of errors
  if (error instanceof ValidationError) {
    handleValidationError(error, req, res);
  } else if (error instanceof NotFoundError) {
    handleNotFoundError(error, req, res);
  } else if (error instanceof ConflictError) {
    handleConflictError(error, req, res);
  } else if (error instanceof UnauthorizedError) {
    handleUnauthorizedError(error, req, res);
  } else if (error instanceof ForbiddenError) {
    handleForbiddenError(error, req, res);
  } else if (error instanceof DatabaseError) {
    handleDatabaseError(error, req, res);
  } else if (error instanceof AppError) {
    handleAppError(error, req, res);
  } else {
    handleUnknownError(error, req, res);
  }
};

// Specific error handlers
function handleValidationError(
  error: ValidationError,
  req: Request,
  res: Response
): void {
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

function handleNotFoundError(
  error: NotFoundError,
  req: Request,
  res: Response
): void {
  const response = {
    success: false,
    error: 'Not Found',
    message: error.message,
    requestId: req.requestId,
    timestamp: new Date().toISOString(),
  };

  res.status(404).json(response);
}

function handleConflictError(
  error: ConflictError,
  req: Request,
  res: Response
): void {
  const response = {
    success: false,
    error: 'Conflict',
    message: error.message,
    requestId: req.requestId,
    timestamp: new Date().toISOString(),
  };

  res.status(409).json(response);
}

function handleUnauthorizedError(
  error: UnauthorizedError,
  req: Request,
  res: Response
): void {
  const response = {
    success: false,
    error: 'Unauthorized',
    message: error.message,
    requestId: req.requestId,
    timestamp: new Date().toISOString(),
  };

  res.status(401).json(response);
}

function handleForbiddenError(
  error: ForbiddenError,
  req: Request,
  res: Response
): void {
  const response = {
    success: false,
    error: 'Forbidden',
    message: error.message,
    requestId: req.requestId,
    timestamp: new Date().toISOString(),
  };

  res.status(403).json(response);
}

function handleDatabaseError(
  error: DatabaseError,
  req: Request,
  res: Response
): void {
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

function handleAppError(error: AppError, req: Request, res: Response): void {
  const response = {
    success: false,
    error: error.name,
    message: error.message,
    requestId: req.requestId,
    timestamp: new Date().toISOString(),
  };

  res.status(error.status).json(response);
}

function handleUnknownError(error: Error, req: Request, res: Response): void {
  const isDevelopment = process.env.NODE_ENV === 'development';

  const response: Record<string, unknown> = {
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
export const notFoundHandler = (req: Request, res: Response): void => {
  const response = {
    success: false,
    error: 'Route Not Found',
    message: `Cannot ${req.method} ${req.originalUrl}`,
    requestId: req.requestId,
    timestamp: new Date().toISOString(),
  };

  res.status(404).json(response);
};

// Graceful shutdown handler
export const gracefulShutdown = (server: unknown, signal: string) => {
  logger.info(`Received ${signal}. Starting graceful shutdown...`);

  if (
    typeof server === 'object' &&
    server !== null &&
    'close' in server &&
    typeof (server as { close: unknown }).close === 'function'
  ) {
    (server as { close: (callback: () => void) => void }).close(() => {
      logger.info('HTTP server closed');
      process.exit(0);
    });
  }

  // Force shutdown after 30 seconds
  setTimeout(() => {
    logger.error('Forced shutdown after timeout');
    process.exit(1);
  }, 30000);
};

// Unhandled rejection handler
export const handleUnhandledRejection = (
  reason: unknown,
  promise: Promise<unknown>
) => {
  logger.error('Unhandled Rejection at:', { reason, promise });
  process.exit(1);
};

// Uncaught exception handler
export const handleUncaughtException = (error: Error) => {
  logger.error('Uncaught Exception:', {}, error);
  process.exit(1);
};

// Setup global error handlers
export const setupErrorHandlers = () => {
  process.on('unhandledRejection', handleUnhandledRejection);
  process.on('uncaughtException', handleUncaughtException);
};

// Export all error handling utilities
export const ErrorHandler = {
  addRequestId,
  requestTiming,
  asyncHandler,
  globalErrorHandler,
  notFoundHandler,
  gracefulShutdown,
  setupErrorHandlers,
};
