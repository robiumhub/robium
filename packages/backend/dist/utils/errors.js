"use strict";
// Custom error classes for the Robium backend application
Object.defineProperty(exports, "__esModule", { value: true });
exports.handleDatabaseError = exports.isOperationalError = exports.DatabaseError = exports.ForbiddenError = exports.UnauthorizedError = exports.ConflictError = exports.NotFoundError = exports.ValidationError = exports.AppError = void 0;
class AppError extends Error {
    constructor(message, status = 500, isOperational = true) {
        super(message);
        this.name = this.constructor.name;
        this.status = status;
        this.isOperational = isOperational;
        Error.captureStackTrace(this, this.constructor);
    }
}
exports.AppError = AppError;
class ValidationError extends AppError {
    constructor(message, errors) {
        super(message, 400);
        this.errors = errors;
    }
}
exports.ValidationError = ValidationError;
class NotFoundError extends AppError {
    constructor(resource, identifier) {
        const message = identifier
            ? `${resource} with identifier '${identifier}' not found`
            : `${resource} not found`;
        super(message, 404);
    }
}
exports.NotFoundError = NotFoundError;
class ConflictError extends AppError {
    constructor(message) {
        super(message, 409);
    }
}
exports.ConflictError = ConflictError;
class UnauthorizedError extends AppError {
    constructor(message = 'Unauthorized') {
        super(message, 401);
    }
}
exports.UnauthorizedError = UnauthorizedError;
class ForbiddenError extends AppError {
    constructor(message = 'Forbidden') {
        super(message, 403);
    }
}
exports.ForbiddenError = ForbiddenError;
class DatabaseError extends AppError {
    constructor(message, code) {
        super(message, 500);
        this.code = code;
    }
}
exports.DatabaseError = DatabaseError;
// Helper function to check if error is operational
const isOperationalError = (error) => {
    if (error instanceof AppError) {
        return error.isOperational;
    }
    return false;
};
exports.isOperationalError = isOperationalError;
// Helper function to handle database constraint errors
const handleDatabaseError = (error) => {
    // PostgreSQL error codes
    const dbError = error;
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
exports.handleDatabaseError = handleDatabaseError;
