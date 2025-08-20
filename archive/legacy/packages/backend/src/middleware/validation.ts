import { Request, Response, NextFunction } from 'express';
import Joi from 'joi';
import { ValidationError } from '../utils/errors';
import { validateData, userValidationSchemas } from '../utils/validation';

// Generic validation middleware factory
export const validateRequest = (
  schema: Joi.ObjectSchema,
  location: 'body' | 'query' | 'params' = 'body'
) => {
  return (req: Request, res: Response, next: NextFunction) => {
    try {
      const data = req[location];
      const validatedData = validateData(data, schema);

      // Replace the original data with validated data
      req[location] = validatedData as (typeof req)[typeof location];

      next();
    } catch (error) {
      if (error instanceof ValidationError) {
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

// Specific validation middleware for user operations
export const validateCreateUserRequest = validateRequest(
  userValidationSchemas.create,
  'body'
);
export const validateUpdateUserRequest = validateRequest(
  userValidationSchemas.update,
  'body'
);
export const validateChangePasswordRequest = validateRequest(
  userValidationSchemas.changePassword,
  'body'
);
export const validateLoginRequest = validateRequest(
  userValidationSchemas.login,
  'body'
);
export const validateUserIdParam = validateRequest(
  Joi.object({ id: userValidationSchemas.id }),
  'params'
);

// Pagination validation middleware
export const validatePagination = (
  req: Request,
  res: Response,
  next: NextFunction
) => {
  const paginationSchema = Joi.object({
    page: Joi.number().integer().min(1).default(1),
    limit: Joi.number().integer().min(1).max(100).default(10),
    sortBy: Joi.string()
      .valid('created_at', 'updated_at', 'email', 'username')
      .default('created_at'),
    sortOrder: Joi.string().valid('asc', 'desc').default('desc'),
    search: Joi.string().max(100).optional(),
  });

  try {
    const validatedQuery = validateData(req.query, paginationSchema);
    req.query = validatedQuery as typeof req.query;
    next();
  } catch (error) {
    if (error instanceof ValidationError) {
      return res.status(400).json({
        success: false,
        error: 'Invalid pagination parameters',
        details: error.errors,
      });
    }
    next(error);
  }
};

// Admin-specific validation middleware
export const validateAdminUserUpdate = (
  req: Request,
  res: Response,
  next: NextFunction
) => {
  const adminUpdateSchema = Joi.object({
    email: Joi.string()
      .email({ tlds: { allow: false } })
      .max(255)
      .optional(),
    username: Joi.string().alphanum().min(3).max(50).optional(),
    role: Joi.string().valid('user', 'admin').optional(),
    isActive: Joi.boolean().optional(),
  }).min(1);

  try {
    const validatedData = validateData(req.body, adminUpdateSchema);
    req.body = validatedData;
    next();
  } catch (error) {
    if (error instanceof ValidationError) {
      return res.status(400).json({
        success: false,
        error: 'Invalid admin update parameters',
        details: error.errors,
      });
    }
    next(error);
  }
};

// UUID validation middleware for any ID parameter
export const validateUUIDParam = (paramName: string = 'id') => {
  return (req: Request, res: Response, next: NextFunction) => {
    const uuidSchema = Joi.object({
      [paramName]: Joi.string().uuid({ version: 'uuidv4' }).required(),
    });

    try {
      const validatedParams = validateData(req.params, uuidSchema);
      req.params = validatedParams as typeof req.params;
      next();
    } catch (error) {
      if (error instanceof ValidationError) {
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

// Content-Type validation middleware
export const validateContentType = (
  expectedType: string = 'application/json'
) => {
  return (req: Request, res: Response, next: NextFunction) => {
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

// File upload validation middleware
export const validateFileUpload = (
  options: {
    maxFiles?: number;
  } = {}
) => {
  const { maxFiles = 1 } = options;

  return (req: Request, res: Response, next: NextFunction) => {
    // This middleware would be used with multer or similar file upload middleware
    // For now, we'll just validate the request structure
    const fileValidationSchema = Joi.object({
      files: Joi.array().max(maxFiles).optional(),
      file: Joi.object().optional(),
    });

    try {
      validateData(req, fileValidationSchema);
      next();
    } catch (error) {
      if (error instanceof ValidationError) {
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

// Rate limiting validation middleware (basic implementation)
export const validateRateLimit = (
  options: {
    windowMs?: number;
    maxRequests?: number;
  } = {}
) => {
  const { windowMs = 15 * 60 * 1000, maxRequests = 100 } = options; // 15 minutes, 100 requests

  const requestCounts = new Map<string, { count: number; resetTime: number }>();

  return (req: Request, res: Response, next: NextFunction) => {
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
    } else if (clientData.count >= maxRequests) {
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
    } else {
      clientData.count++;
      next();
    }
  };
};

// Sanitization middleware
export const sanitizeInput = (
  req: Request,
  res: Response,
  next: NextFunction
) => {
  // Basic XSS protection - remove script tags and dangerous content
  const sanitizeString = (str: string): string => {
    return str
      .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '')
      .replace(/javascript:/gi, '')
      .replace(/on\w+\s*=/gi, '')
      .trim();
  };

  const sanitizeObject = (obj: unknown): unknown => {
    if (typeof obj === 'string') {
      return sanitizeString(obj);
    }

    if (Array.isArray(obj)) {
      return obj.map(sanitizeObject);
    }

    if (obj && typeof obj === 'object') {
      const sanitized: Record<string, unknown> = {};
      for (const [key, value] of Object.entries(obj)) {
        sanitized[key] = sanitizeObject(value);
      }
      return sanitized;
    }

    return obj;
  };

  // Sanitize body, query, and params
  if (req.body) {
    req.body = sanitizeObject(req.body) as typeof req.body;
  }

  if (req.query) {
    req.query = sanitizeObject(req.query) as typeof req.query;
  }

  if (req.params) {
    req.params = sanitizeObject(req.params) as typeof req.params;
  }

  next();
};

// Export all middleware functions
export const ValidationMiddleware = {
  validateRequest,
  validateCreateUserRequest,
  validateUpdateUserRequest,
  validateChangePasswordRequest,
  validateLoginRequest,
  validateUserIdParam,
  validatePagination,
  validateAdminUserUpdate,
  validateUUIDParam,
  validateContentType,
  validateFileUpload,
  validateRateLimit,
  sanitizeInput,
};
