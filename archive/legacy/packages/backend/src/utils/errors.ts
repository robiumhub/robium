// Custom error classes for the Robium backend application

export class AppError extends Error {
  public readonly status: number;
  public readonly isOperational: boolean;

  constructor(
    message: string,
    status: number = 500,
    isOperational: boolean = true
  ) {
    super(message);
    this.name = this.constructor.name;
    this.status = status;
    this.isOperational = isOperational;

    Error.captureStackTrace(this, this.constructor);
  }
}

export class ValidationError extends AppError {
  public readonly errors?: Array<{ field: string; message: string }>;

  constructor(
    message: string,
    errors?: Array<{ field: string; message: string }>
  ) {
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
  // SQLite error handling
  const dbError = error as {
    code?: string;
    constraint?: string;
    message?: string;
  };

  // SQLite error codes
  if (dbError.message?.includes('UNIQUE constraint failed')) {
    if (dbError.message.includes('email')) {
      return new ConflictError('Email address is already registered');
    }
    if (dbError.message.includes('username')) {
      return new ConflictError('Username is already taken');
    }
    return new ConflictError('A record with this data already exists');
  }

  if (dbError.message?.includes('FOREIGN KEY constraint failed')) {
    return new ValidationError('Referenced record does not exist');
  }

  if (dbError.message?.includes('NOT NULL constraint failed')) {
    return new ValidationError('Required field is missing');
  }

  if (dbError.message?.includes('database is locked')) {
    return new DatabaseError('Database is temporarily unavailable');
  }

  return new DatabaseError(
    `Database operation failed: ${dbError.message || 'Unknown error'}`,
    dbError.code
  );
};
