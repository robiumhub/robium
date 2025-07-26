"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.checkPasswordStrength = exports.isValidEmail = exports.sanitizeUser = exports.validateUserId = exports.validateLogin = exports.validateChangePassword = exports.validateUpdateUser = exports.validateCreateUser = exports.validateData = exports.userValidationSchemas = void 0;
const joi_1 = __importDefault(require("joi"));
const errors_1 = require("./errors");
const types_1 = require("../types");
// User validation schemas
exports.userValidationSchemas = {
    // Schema for creating a new user
    create: joi_1.default.object({
        email: joi_1.default.string()
            .email({ tlds: { allow: false } })
            .max(255)
            .required()
            .messages({
            'string.email': 'Please provide a valid email address',
            'string.max': 'Email address cannot exceed 255 characters',
            'any.required': 'Email address is required',
        }),
        username: joi_1.default.string().alphanum().min(3).max(50).required().messages({
            'string.alphanum': 'Username can only contain letters and numbers',
            'string.min': 'Username must be at least 3 characters long',
            'string.max': 'Username cannot exceed 50 characters',
            'any.required': 'Username is required',
        }),
        password: joi_1.default.string()
            .min(8)
            .max(128)
            .pattern(/^(?=.*[a-z])(?=.*[A-Z])(?=.*\d)(?=.*[@$!%*?&])[A-Za-z\d@$!%*?&]/)
            .required()
            .messages({
            'string.min': 'Password must be at least 8 characters long',
            'string.max': 'Password cannot exceed 128 characters',
            'string.pattern.base': 'Password must contain at least one uppercase letter, one lowercase letter, one number, and one special character',
            'any.required': 'Password is required',
        }),
        role: joi_1.default.string()
            .valid(...Object.values(types_1.UserRole))
            .default(types_1.UserRole.USER)
            .messages({
            'any.only': `Role must be one of: ${Object.values(types_1.UserRole).join(', ')}`,
        }),
    }),
    // Schema for updating a user (all fields optional except id)
    update: joi_1.default.object({
        email: joi_1.default.string()
            .email({ tlds: { allow: false } })
            .max(255)
            .messages({
            'string.email': 'Please provide a valid email address',
            'string.max': 'Email address cannot exceed 255 characters',
        }),
        username: joi_1.default.string().alphanum().min(3).max(50).messages({
            'string.alphanum': 'Username can only contain letters and numbers',
            'string.min': 'Username must be at least 3 characters long',
            'string.max': 'Username cannot exceed 50 characters',
        }),
        role: joi_1.default.string()
            .valid(...Object.values(types_1.UserRole))
            .messages({
            'any.only': `Role must be one of: ${Object.values(types_1.UserRole).join(', ')}`,
        }),
    })
        .min(1)
        .messages({
        'object.min': 'At least one field must be provided for update',
    }),
    // Schema for password change
    changePassword: joi_1.default.object({
        currentPassword: joi_1.default.string().required().messages({
            'any.required': 'Current password is required',
        }),
        newPassword: joi_1.default.string()
            .min(8)
            .max(128)
            .pattern(/^(?=.*[a-z])(?=.*[A-Z])(?=.*\d)(?=.*[@$!%*?&])[A-Za-z\d@$!%*?&]/)
            .required()
            .messages({
            'string.min': 'New password must be at least 8 characters long',
            'string.max': 'New password cannot exceed 128 characters',
            'string.pattern.base': 'New password must contain at least one uppercase letter, one lowercase letter, one number, and one special character',
            'any.required': 'New password is required',
        }),
    }),
    // Schema for login validation
    login: joi_1.default.object({
        email: joi_1.default.string()
            .email({ tlds: { allow: false } })
            .required()
            .messages({
            'string.email': 'Please provide a valid email address',
            'any.required': 'Email address is required',
        }),
        password: joi_1.default.string().required().messages({
            'any.required': 'Password is required',
        }),
    }),
    // Schema for UUID validation
    id: joi_1.default.string().uuid({ version: 'uuidv4' }).required().messages({
        'string.guid': 'Invalid user ID format',
        'any.required': 'User ID is required',
    }),
};
// Generic validation function
const validateData = (data, schema) => {
    const { error, value } = schema.validate(data, {
        abortEarly: false,
        stripUnknown: true,
        convert: true, // Convert string numbers to numbers, etc.
    });
    if (error) {
        const validationErrors = error.details.map((detail) => ({
            field: detail.path.map(String).join('.'),
            message: detail.message,
        }));
        throw new errors_1.ValidationError('Validation failed', validationErrors);
    }
    return value;
};
exports.validateData = validateData;
// Specific validation functions for common use cases
const validateCreateUser = (data) => {
    return (0, exports.validateData)(data, exports.userValidationSchemas.create);
};
exports.validateCreateUser = validateCreateUser;
const validateUpdateUser = (data) => {
    return (0, exports.validateData)(data, exports.userValidationSchemas.update);
};
exports.validateUpdateUser = validateUpdateUser;
const validateChangePassword = (data) => {
    return (0, exports.validateData)(data, exports.userValidationSchemas.changePassword);
};
exports.validateChangePassword = validateChangePassword;
const validateLogin = (data) => {
    return (0, exports.validateData)(data, exports.userValidationSchemas.login);
};
exports.validateLogin = validateLogin;
const validateUserId = (data) => {
    return (0, exports.validateData)(data, joi_1.default.object({ id: exports.userValidationSchemas.id }));
};
exports.validateUserId = validateUserId;
// Helper function to sanitize user data for responses (remove sensitive fields)
const sanitizeUser = (user) => {
    if (!user)
        return null;
    const userObj = user;
    // eslint-disable-next-line @typescript-eslint/no-unused-vars
    const { password_hash: _password_hash, ...sanitizedUser } = userObj;
    return sanitizedUser;
};
exports.sanitizeUser = sanitizeUser;
// Helper function to validate email format (simple check)
const isValidEmail = (email) => {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
};
exports.isValidEmail = isValidEmail;
// Helper function to check password strength
const checkPasswordStrength = (password) => {
    const issues = [];
    if (password.length < 8) {
        issues.push('Password must be at least 8 characters long');
    }
    if (!/[a-z]/.test(password)) {
        issues.push('Password must contain at least one lowercase letter');
    }
    if (!/[A-Z]/.test(password)) {
        issues.push('Password must contain at least one uppercase letter');
    }
    if (!/\d/.test(password)) {
        issues.push('Password must contain at least one number');
    }
    if (!/[@$!%*?&]/.test(password)) {
        issues.push('Password must contain at least one special character (@$!%*?&)');
    }
    return {
        isStrong: issues.length === 0,
        issues,
    };
};
exports.checkPasswordStrength = checkPasswordStrength;
