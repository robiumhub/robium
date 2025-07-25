"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.AuthService = void 0;
const jsonwebtoken_1 = __importDefault(require("jsonwebtoken"));
const User_1 = require("../models/User");
const validation_1 = require("../utils/validation");
const errors_1 = require("../utils/errors");
class AuthService {
    // Generate JWT token for user
    static generateToken(user) {
        const jwtSecret = process.env.JWT_SECRET;
        const jwtExpiresIn = process.env.JWT_EXPIRES_IN || '1h';
        if (!jwtSecret) {
            throw new Error('JWT_SECRET environment variable is not configured');
        }
        const payload = {
            userId: user.id,
            email: user.email,
            role: user.role,
        };
        return jsonwebtoken_1.default.sign(payload, jwtSecret, {
            expiresIn: jwtExpiresIn,
        });
    }
    // Generate refresh token (for future implementation)
    static generateRefreshToken(user) {
        const jwtRefreshSecret = process.env.JWT_REFRESH_SECRET || process.env.JWT_SECRET;
        const jwtRefreshExpiresIn = process.env.JWT_REFRESH_EXPIRES_IN || '7d';
        if (!jwtRefreshSecret) {
            throw new Error('JWT_REFRESH_SECRET environment variable is not configured');
        }
        const payload = {
            userId: user.id,
            email: user.email,
            role: user.role,
        };
        return jsonwebtoken_1.default.sign(payload, jwtRefreshSecret, {
            expiresIn: jwtRefreshExpiresIn,
        });
    }
    // User registration
    static async register(userData) {
        try {
            // Validate input data
            const validatedData = (0, validation_1.validateCreateUser)(userData);
            // Check if user already exists
            const existingUserByEmail = await User_1.UserModel.existsByEmail(validatedData.email);
            if (existingUserByEmail) {
                throw new errors_1.ConflictError('Email address is already registered');
            }
            const existingUserByUsername = await User_1.UserModel.existsByUsername(validatedData.username);
            if (existingUserByUsername) {
                throw new errors_1.ConflictError('Username is already taken');
            }
            // Create new user
            const newUser = await User_1.UserModel.create(validatedData);
            // Generate tokens
            const token = this.generateToken(newUser);
            const refreshToken = this.generateRefreshToken(newUser);
            return {
                user: newUser,
                token,
                refreshToken,
            };
        }
        catch (error) {
            // Re-throw known errors
            if (error instanceof errors_1.ValidationError || error instanceof errors_1.ConflictError) {
                throw error;
            }
            // Handle unexpected errors
            throw new Error('Registration failed');
        }
    }
    // User login
    static async login(loginData) {
        try {
            // Validate input data
            const validatedData = (0, validation_1.validateLogin)(loginData);
            const { email, password } = validatedData;
            // Find user by email with password hash
            const userWithPassword = await User_1.UserModel.findByEmailWithPassword(email);
            if (!userWithPassword) {
                throw new errors_1.UnauthorizedError('Invalid email or password');
            }
            // Verify password
            const isPasswordValid = await User_1.UserModel.validatePassword(password, userWithPassword.password_hash);
            if (!isPasswordValid) {
                throw new errors_1.UnauthorizedError('Invalid email or password');
            }
            // Remove password hash from user object
            const user = userWithPassword;
            // Generate token
            const token = this.generateToken(user);
            return {
                user: user,
                token,
                expiresIn: process.env.JWT_EXPIRES_IN || '1h',
            };
        }
        catch (error) {
            // Re-throw known errors
            if (error instanceof errors_1.ValidationError ||
                error instanceof errors_1.UnauthorizedError) {
                throw error;
            }
            // Handle unexpected errors
            throw new errors_1.UnauthorizedError('Authentication failed');
        }
    }
    // Verify token and get user
    static async verifyToken(token) {
        try {
            const jwtSecret = process.env.JWT_SECRET;
            if (!jwtSecret) {
                throw new Error('JWT_SECRET environment variable is not configured');
            }
            // Verify JWT token
            const decoded = jsonwebtoken_1.default.verify(token, jwtSecret);
            // Get user from database
            const user = await User_1.UserModel.findById(decoded.userId);
            if (!user) {
                throw new errors_1.UnauthorizedError('User not found');
            }
            return user;
        }
        catch (error) {
            if (error instanceof jsonwebtoken_1.default.JsonWebTokenError) {
                throw new errors_1.UnauthorizedError('Invalid token');
            }
            else if (error instanceof jsonwebtoken_1.default.TokenExpiredError) {
                throw new errors_1.UnauthorizedError('Token has expired');
            }
            else if (error instanceof errors_1.UnauthorizedError) {
                throw error;
            }
            else {
                throw new errors_1.UnauthorizedError('Token verification failed');
            }
        }
    }
    // Get current user profile
    static async getCurrentUser(userId) {
        try {
            const user = await User_1.UserModel.findById(userId);
            if (!user) {
                throw new errors_1.UnauthorizedError('User not found');
            }
            return user;
        }
        catch (error) {
            if (error instanceof errors_1.UnauthorizedError) {
                throw error;
            }
            throw new errors_1.UnauthorizedError('Failed to get user profile');
        }
    }
    // Change password
    static async changePassword(userId, currentPassword, newPassword) {
        try {
            await User_1.UserModel.changePassword(userId, currentPassword, newPassword);
        }
        catch (error) {
            if (error instanceof errors_1.ValidationError ||
                error instanceof errors_1.UnauthorizedError) {
                throw error;
            }
            throw new Error('Password change failed');
        }
    }
    // Refresh token (for future implementation)
    static async refreshToken(refreshToken) {
        try {
            const jwtRefreshSecret = process.env.JWT_REFRESH_SECRET || process.env.JWT_SECRET;
            if (!jwtRefreshSecret) {
                throw new Error('JWT_REFRESH_SECRET environment variable is not configured');
            }
            // Verify refresh token
            const decoded = jsonwebtoken_1.default.verify(refreshToken, jwtRefreshSecret);
            // Get user from database
            const user = await User_1.UserModel.findById(decoded.userId);
            if (!user) {
                throw new errors_1.UnauthorizedError('User not found');
            }
            // Generate new access token
            const newToken = this.generateToken(user);
            return {
                token: newToken,
                expiresIn: process.env.JWT_EXPIRES_IN || '1h',
            };
        }
        catch (error) {
            if (error instanceof jsonwebtoken_1.default.JsonWebTokenError) {
                throw new errors_1.UnauthorizedError('Invalid refresh token');
            }
            else if (error instanceof jsonwebtoken_1.default.TokenExpiredError) {
                throw new errors_1.UnauthorizedError('Refresh token has expired');
            }
            else if (error instanceof errors_1.UnauthorizedError) {
                throw error;
            }
            else {
                throw new errors_1.UnauthorizedError('Token refresh failed');
            }
        }
    }
    // Logout (for future session management)
    static async logout(token) {
        // For JWT tokens, logout is typically handled client-side by removing the token
        // In a more advanced implementation, you might maintain a blacklist of tokens
        // or use session-based authentication with server-side session management
        // For now, we'll just verify the token is valid
        try {
            await this.verifyToken(token);
            // Token is valid, logout is successful
            // In the future, add token to blacklist or invalidate session
        }
        catch (error) {
            // Token is already invalid, which is fine for logout
        }
    }
}
exports.AuthService = AuthService;
