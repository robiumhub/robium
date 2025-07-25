"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.ValidationMiddleware = exports.sanitizeInput = exports.validateRateLimit = exports.validateFileUpload = exports.validateContentType = exports.validateUUIDParam = exports.validateAdminUserUpdate = exports.validatePagination = exports.validateUserIdParam = exports.validateLoginRequest = exports.validateChangePasswordRequest = exports.validateUpdateUserRequest = exports.validateCreateUserRequest = exports.validateRequest = void 0;
const joi_1 = __importDefault(require("joi"));
const errors_1 = require("../utils/errors");
const validation_1 = require("../utils/validation");
// Generic validation middleware factory
const validateRequest = (schema, location = 'body') => {
    return (req, res, next) => {
        try {
            const data = req[location];
            const validatedData = (0, validation_1.validateData)(data, schema);
            // Replace the original data with validated data
            req[location] = validatedData;
            next();
        }
        catch (error) {
            if (error instanceof errors_1.ValidationError) {
                return res.status(400).json({
                    success: false,
                    error: 'Validation failed',
                    details: error.errors,
                });
            }
            next(error);
        }
    };
};
exports.validateRequest = validateRequest;
// Specific validation middleware for user operations
exports.validateCreateUserRequest = (0, exports.validateRequest)(validation_1.userValidationSchemas.create, 'body');
exports.validateUpdateUserRequest = (0, exports.validateRequest)(validation_1.userValidationSchemas.update, 'body');
exports.validateChangePasswordRequest = (0, exports.validateRequest)(validation_1.userValidationSchemas.changePassword, 'body');
exports.validateLoginRequest = (0, exports.validateRequest)(validation_1.userValidationSchemas.login, 'body');
exports.validateUserIdParam = (0, exports.validateRequest)(joi_1.default.object({ id: validation_1.userValidationSchemas.id }), 'params');
// Pagination validation middleware
const validatePagination = (req, res, next) => {
    const paginationSchema = joi_1.default.object({
        page: joi_1.default.number().integer().min(1).default(1),
        limit: joi_1.default.number().integer().min(1).max(100).default(10),
        sortBy: joi_1.default.string()
            .valid('created_at', 'updated_at', 'email', 'username')
            .default('created_at'),
        sortOrder: joi_1.default.string().valid('asc', 'desc').default('desc'),
        search: joi_1.default.string().max(100).optional(),
    });
    try {
        const validatedQuery = (0, validation_1.validateData)(req.query, paginationSchema);
        req.query = validatedQuery;
        next();
    }
    catch (error) {
        if (error instanceof errors_1.ValidationError) {
            return res.status(400).json({
                success: false,
                error: 'Invalid pagination parameters',
                details: error.errors,
            });
        }
        next(error);
    }
};
exports.validatePagination = validatePagination;
// Admin-specific validation middleware
const validateAdminUserUpdate = (req, res, next) => {
    const adminUpdateSchema = joi_1.default.object({
        email: joi_1.default.string()
            .email({ tlds: { allow: false } })
            .max(255)
            .optional(),
        username: joi_1.default.string().alphanum().min(3).max(50).optional(),
        role: joi_1.default.string().valid('user', 'admin').optional(),
        isActive: joi_1.default.boolean().optional(),
    }).min(1);
    try {
        const validatedData = (0, validation_1.validateData)(req.body, adminUpdateSchema);
        req.body = validatedData;
        next();
    }
    catch (error) {
        if (error instanceof errors_1.ValidationError) {
            return res.status(400).json({
                success: false,
                error: 'Invalid admin update parameters',
                details: error.errors,
            });
        }
        next(error);
    }
};
exports.validateAdminUserUpdate = validateAdminUserUpdate;
// UUID validation middleware for any ID parameter
const validateUUIDParam = (paramName = 'id') => {
    return (req, res, next) => {
        const uuidSchema = joi_1.default.object({
            [paramName]: joi_1.default.string().uuid({ version: 'uuidv4' }).required(),
        });
        try {
            const validatedParams = (0, validation_1.validateData)(req.params, uuidSchema);
            req.params = validatedParams;
            next();
        }
        catch (error) {
            if (error instanceof errors_1.ValidationError) {
                return res.status(400).json({
                    success: false,
                    error: `Invalid ${paramName} format`,
                    details: error.errors,
                });
            }
            next(error);
        }
    };
};
exports.validateUUIDParam = validateUUIDParam;
// Content-Type validation middleware
const validateContentType = (expectedType = 'application/json') => {
    return (req, res, next) => {
        const contentType = req.get('Content-Type');
        if (!contentType || !contentType.includes(expectedType)) {
            return res.status(400).json({
                success: false,
                error: `Content-Type must be ${expectedType}`,
                details: [
                    {
                        field: 'Content-Type',
                        message: `Expected ${expectedType}, got ${contentType}`,
                    },
                ],
            });
        }
        next();
    };
};
exports.validateContentType = validateContentType;
// File upload validation middleware
const validateFileUpload = (options = {}) => {
    const { maxFiles = 1 } = options;
    return (req, res, next) => {
        // This middleware would be used with multer or similar file upload middleware
        // For now, we'll just validate the request structure
        const fileValidationSchema = joi_1.default.object({
            files: joi_1.default.array().max(maxFiles).optional(),
            file: joi_1.default.object().optional(),
        });
        try {
            (0, validation_1.validateData)(req, fileValidationSchema);
            next();
        }
        catch (error) {
            if (error instanceof errors_1.ValidationError) {
                return res.status(400).json({
                    success: false,
                    error: 'Invalid file upload',
                    details: error.errors,
                });
            }
            next(error);
        }
    };
};
exports.validateFileUpload = validateFileUpload;
// Rate limiting validation middleware (basic implementation)
const validateRateLimit = (options = {}) => {
    const { windowMs = 15 * 60 * 1000, maxRequests = 100 } = options; // 15 minutes, 100 requests
    const requestCounts = new Map();
    return (req, res, next) => {
        const clientId = req.ip || req.connection.remoteAddress || 'unknown';
        const now = Date.now();
        const clientData = requestCounts.get(clientId);
        if (!clientData || now > clientData.resetTime) {
            // Reset or initialize client data
            requestCounts.set(clientId, {
                count: 1,
                resetTime: now + windowMs,
            });
            next();
        }
        else if (clientData.count >= maxRequests) {
            return res.status(429).json({
                success: false,
                error: 'Too many requests',
                details: [
                    {
                        field: 'rate_limit',
                        message: `Rate limit exceeded. Try again in ${Math.ceil((clientData.resetTime - now) / 1000)} seconds.`,
                    },
                ],
            });
        }
        else {
            clientData.count++;
            next();
        }
    };
};
exports.validateRateLimit = validateRateLimit;
// Sanitization middleware
const sanitizeInput = (req, res, next) => {
    // Basic XSS protection - remove script tags and dangerous content
    const sanitizeString = (str) => {
        return str
            .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '')
            .replace(/javascript:/gi, '')
            .replace(/on\w+\s*=/gi, '')
            .trim();
    };
    const sanitizeObject = (obj) => {
        if (typeof obj === 'string') {
            return sanitizeString(obj);
        }
        if (Array.isArray(obj)) {
            return obj.map(sanitizeObject);
        }
        if (obj && typeof obj === 'object') {
            const sanitized = {};
            for (const [key, value] of Object.entries(obj)) {
                sanitized[key] = sanitizeObject(value);
            }
            return sanitized;
        }
        return obj;
    };
    // Sanitize body, query, and params
    if (req.body) {
        req.body = sanitizeObject(req.body);
    }
    if (req.query) {
        req.query = sanitizeObject(req.query);
    }
    if (req.params) {
        req.params = sanitizeObject(req.params);
    }
    next();
};
exports.sanitizeInput = sanitizeInput;
// Export all middleware functions
exports.ValidationMiddleware = {
    validateRequest: exports.validateRequest,
    validateCreateUserRequest: exports.validateCreateUserRequest,
    validateUpdateUserRequest: exports.validateUpdateUserRequest,
    validateChangePasswordRequest: exports.validateChangePasswordRequest,
    validateLoginRequest: exports.validateLoginRequest,
    validateUserIdParam: exports.validateUserIdParam,
    validatePagination: exports.validatePagination,
    validateAdminUserUpdate: exports.validateAdminUserUpdate,
    validateUUIDParam: exports.validateUUIDParam,
    validateContentType: exports.validateContentType,
    validateFileUpload: exports.validateFileUpload,
    validateRateLimit: exports.validateRateLimit,
    sanitizeInput: exports.sanitizeInput,
};
