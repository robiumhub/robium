// Custom error classes for the Robium backend application

export class AppError extends Error {
  public readonly status: number;
  public readonly isOperational: boolean;

  constructor(message: string, status: number = 500, isOperational: boolean = true) {
    super(message);
    this.name = this.constructor.name;
    this.status = status;
    this.isOperational = isOperational;

    Error.captureStackTrace(this, this.constructor);
  }
}

export class ValidationError extends AppError {
  public readonly errors?: Array<{ field: string; message: string }>;

  constructor(message: string, errors?: Array<{ field: string; message: string }>) {
    super(message, 400);
    this.errors = errors;
  }
}

export class NotFoundError extends AppError {
  constructor(resource: string, identifier?: string) {
    const message = identifier 
      ? `${resource} with identifier '${identifier}' not found`
      : `${resource} not found`;
    super(message, 404);
  }
}

export class ConflictError extends AppError {
  constructor(message: string) {
    super(message, 409);
  }
}

export class UnauthorizedError extends AppError {
  constructor(message: string = 'Unauthorized') {
    super(message, 401);
  }
}

export class ForbiddenError extends AppError {
  constructor(message: string = 'Forbidden') {
    super(message, 403);
  }
}

export class DatabaseError extends AppError {
  public readonly code?: string;

  constructor(message: string, code?: string) {
    super(message, 500);
    this.code = code;
  }
}

// Helper function to check if error is operational
export const isOperationalError = (error: Error): boolean => {
  if (error instanceof AppError) {
    return error.isOperational;
  }
  return false;
};

// Helper function to handle database constraint errors
export const handleDatabaseError = (error: unknown): AppError => {
  // PostgreSQL error codes
  const dbError = error as { code?: string; constraint?: string; message?: string };
  switch (dbError.code) {
          case '23505': // Unique constraint violation
        if (dbError.constraint?.includes('email')) {
          return new ConflictError('Email address is already registered');
        }
        if (dbError.constraint?.includes('username')) {
          return new ConflictError('Username is already taken');
        }
        return new ConflictError('A record with this data already exists');
    
    case '23503': // Foreign key constraint violation
      return new ValidationError('Referenced record does not exist');
    
    case '23502': // Not null constraint violation
      return new ValidationError('Required field is missing');
    
    case '22001': // String data right truncation
      return new ValidationError('Input data is too long');
    
    case '08006': // Connection failure
    case '08001': // Unable to connect
      return new DatabaseError('Database connection failed');
    
          default:
        return new DatabaseError(`Database operation failed: ${dbError.message || 'Unknown error'}`, dbError.code);
  }
}; 