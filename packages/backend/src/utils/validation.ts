import Joi from 'joi';
import { ValidationError } from './errors';
import { UserRole, CreateUserInput } from '../types';

// User validation schemas
export const userValidationSchemas = {
  // Schema for creating a new user
  create: Joi.object({
    email: Joi.string()
      .email({ tlds: { allow: false } })
      .max(255)
      .required()
      .messages({
        'string.email': 'Please provide a valid email address',
        'string.max': 'Email address cannot exceed 255 characters',
        'any.required': 'Email address is required',
      }),

    username: Joi.string().alphanum().min(3).max(50).required().messages({
      'string.alphanum': 'Username can only contain letters and numbers',
      'string.min': 'Username must be at least 3 characters long',
      'string.max': 'Username cannot exceed 50 characters',
      'any.required': 'Username is required',
    }),

    password: Joi.string().min(6).max(128).required().messages({
      'string.min': 'Password must be at least 6 characters long',
      'string.max': 'Password cannot exceed 128 characters',
      'any.required': 'Password is required',
    }),

    role: Joi.string()
      .valid(...Object.values(UserRole))
      .default(UserRole.USER)
      .messages({
        'any.only': `Role must be one of: ${Object.values(UserRole).join(', ')}`,
      }),
  }),

  // Schema for updating a user (all fields optional except id)
  update: Joi.object({
    email: Joi.string()
      .email({ tlds: { allow: false } })
      .max(255)
      .messages({
        'string.email': 'Please provide a valid email address',
        'string.max': 'Email address cannot exceed 255 characters',
      }),

    username: Joi.string().alphanum().min(3).max(50).messages({
      'string.alphanum': 'Username can only contain letters and numbers',
      'string.min': 'Username must be at least 3 characters long',
      'string.max': 'Username cannot exceed 50 characters',
    }),

    role: Joi.string()
      .valid(...Object.values(UserRole))
      .messages({
        'any.only': `Role must be one of: ${Object.values(UserRole).join(', ')}`,
      }),
  })
    .min(1)
    .messages({
      'object.min': 'At least one field must be provided for update',
    }),

  // Schema for password change
  changePassword: Joi.object({
    currentPassword: Joi.string().required().messages({
      'any.required': 'Current password is required',
    }),

    newPassword: Joi.string().min(6).max(128).required().messages({
      'string.min': 'New password must be at least 6 characters long',
      'string.max': 'New password cannot exceed 128 characters',
      'any.required': 'New password is required',
    }),
  }),

  // Schema for login validation
  login: Joi.object({
    email: Joi.string()
      .email({ tlds: { allow: false } })
      .required()
      .messages({
        'string.email': 'Please provide a valid email address',
        'any.required': 'Email address is required',
      }),

    password: Joi.string().required().messages({
      'any.required': 'Password is required',
    }),
  }),

  // Schema for UUID validation
  id: Joi.string().uuid({ version: 'uuidv4' }).required().messages({
    'string.guid': 'Invalid user ID format',
    'any.required': 'User ID is required',
  }),
};

// Generic validation function
export const validateData = <T>(data: unknown, schema: Joi.ObjectSchema): T => {
  const { error, value } = schema.validate(data, {
    abortEarly: false, // Collect all validation errors
    stripUnknown: true, // Remove unknown fields
    convert: true, // Convert string numbers to numbers, etc.
  });

  if (error) {
    const validationErrors = error.details.map(
      (detail: { path: (string | number)[]; message: string }) => ({
        field: detail.path.map(String).join('.'),
        message: detail.message,
      })
    );

    throw new ValidationError('Validation failed', validationErrors);
  }

  return value as T;
};

// Specific validation functions for common use cases
export const validateCreateUser = (data: unknown): CreateUserInput => {
  return validateData<CreateUserInput>(data, userValidationSchemas.create);
};

export const validateUpdateUser = (data: unknown): Partial<CreateUserInput> => {
  return validateData<Partial<CreateUserInput>>(
    data,
    userValidationSchemas.update
  );
};

export const validateChangePassword = (data: unknown) => {
  return validateData(data, userValidationSchemas.changePassword);
};

export const validateLogin = (data: unknown) => {
  return validateData(data, userValidationSchemas.login);
};

export const validateUserId = (data: { id: string }) => {
  return validateData(data, Joi.object({ id: userValidationSchemas.id }));
};

// Helper function to sanitize user data for responses (remove sensitive fields)
export const sanitizeUser = (user: unknown) => {
  if (!user) return null;

  const userObj = user as { password_hash?: string; [key: string]: unknown };
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  const { password_hash: _password_hash, ...sanitizedUser } = userObj;
  return sanitizedUser;
};

// Helper function to validate email format (simple check)
export const isValidEmail = (email: string): boolean => {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
};

// Helper function to check password strength
export const checkPasswordStrength = (
  password: string
): {
  isStrong: boolean;
  issues: string[];
} => {
  const issues: string[] = [];

  if (password.length < 6) {
    issues.push('Password must be at least 6 characters long');
  }

  return {
    isStrong: issues.length === 0,
    issues,
  };
};
