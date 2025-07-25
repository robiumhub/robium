"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.requireOwnershipOrAdmin = exports.requireAdmin = exports.requireRole = exports.optionalAuth = exports.authenticateToken = void 0;
const jsonwebtoken_1 = __importDefault(require("jsonwebtoken"));
const errors_1 = require("../utils/errors");
const User_1 = require("../models/User");
// Authentication middleware to verify JWT tokens
const authenticateToken = async (req, res, next) => {
    try {
        // Get token from Authorization header
        const authHeader = req.headers.authorization;
        const token = authHeader && authHeader.startsWith('Bearer ')
            ? authHeader.substring(7)
            : null;
        if (!token) {
            throw new errors_1.UnauthorizedError('Access token is required');
        }
        // Verify JWT token
        const jwtSecret = process.env.JWT_SECRET;
        if (!jwtSecret) {
            throw new Error('JWT_SECRET environment variable is not configured');
        }
        const decoded = jsonwebtoken_1.default.verify(token, jwtSecret);
        // Verify user still exists (optional security check)
        const user = await User_1.UserModel.findById(decoded.userId);
        if (!user) {
            throw new errors_1.UnauthorizedError('User not found');
        }
        // Add user info to request object
        req.user = {
            userId: decoded.userId,
            email: decoded.email,
            role: decoded.role,
            iat: decoded.iat,
            exp: decoded.exp,
        };
        next();
    }
    catch (error) {
        if (error instanceof jsonwebtoken_1.default.JsonWebTokenError) {
            next(new errors_1.UnauthorizedError('Invalid token'));
        }
        else if (error instanceof jsonwebtoken_1.default.TokenExpiredError) {
            next(new errors_1.UnauthorizedError('Token has expired'));
        }
        else if (error instanceof errors_1.UnauthorizedError) {
            next(error);
        }
        else {
            next(new errors_1.UnauthorizedError('Authentication failed'));
        }
    }
};
exports.authenticateToken = authenticateToken;
// Optional middleware for routes that work with or without authentication
const optionalAuth = async (req, res, next) => {
    try {
        const authHeader = req.headers.authorization;
        const token = authHeader && authHeader.startsWith('Bearer ')
            ? authHeader.substring(7)
            : null;
        if (token) {
            const jwtSecret = process.env.JWT_SECRET;
            if (jwtSecret) {
                try {
                    const decoded = jsonwebtoken_1.default.verify(token, jwtSecret);
                    // Verify user still exists
                    const user = await User_1.UserModel.findById(decoded.userId);
                    if (user) {
                        req.user = {
                            userId: decoded.userId,
                            email: decoded.email,
                            role: decoded.role,
                            iat: decoded.iat,
                            exp: decoded.exp,
                        };
                    }
                }
                catch (error) {
                    // Silently ignore token errors for optional auth
                    console.log('Optional auth token error:', error);
                }
            }
        }
        next();
    }
    catch (error) {
        // Continue without authentication for optional auth
        next();
    }
};
exports.optionalAuth = optionalAuth;
// Middleware to check if user has specific role
const requireRole = (requiredRole) => {
    return (req, res, next) => {
        if (!req.user) {
            next(new errors_1.UnauthorizedError('Authentication required'));
            return;
        }
        if (req.user.role !== requiredRole) {
            next(new errors_1.UnauthorizedError(`${requiredRole} role required`));
            return;
        }
        next();
    };
};
exports.requireRole = requireRole;
// Middleware to check if user has admin role
exports.requireAdmin = (0, exports.requireRole)('admin');
// Middleware to check if user owns the resource or is admin
const requireOwnershipOrAdmin = (getUserIdFromParams) => {
    return (req, res, next) => {
        if (!req.user) {
            next(new errors_1.UnauthorizedError('Authentication required'));
            return;
        }
        const resourceUserId = getUserIdFromParams(req);
        const isOwner = req.user.userId === resourceUserId;
        const isAdmin = req.user.role === 'admin';
        if (!isOwner && !isAdmin) {
            next(new errors_1.UnauthorizedError('Access denied: insufficient permissions'));
            return;
        }
        next();
    };
};
exports.requireOwnershipOrAdmin = requireOwnershipOrAdmin;
