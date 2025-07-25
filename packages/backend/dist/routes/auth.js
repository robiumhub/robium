"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const express_1 = require("express");
const AuthService_1 = require("../services/AuthService");
const validation_1 = require("../middleware/validation");
const router = (0, express_1.Router)();
// Apply sanitization to all routes
router.use(validation_1.sanitizeInput);
// Register new user
router.post('/register', validation_1.validateCreateUserRequest, async (req, res, next) => {
    try {
        const result = await AuthService_1.AuthService.register(req.body);
        const response = {
            success: true,
            data: result,
            message: 'User registered successfully',
        };
        res.status(201).json(response);
    }
    catch (error) {
        next(error);
    }
});
// Login user
router.post('/login', validation_1.validateLoginRequest, async (req, res, next) => {
    try {
        const result = await AuthService_1.AuthService.login(req.body);
        const response = {
            success: true,
            data: result,
            message: 'Login successful',
        };
        res.json(response);
    }
    catch (error) {
        next(error);
    }
});
// Change password (requires authentication)
router.post('/change-password', validation_1.validateChangePasswordRequest, async (req, res, next) => {
    try {
        if (!req.user) {
            return res.status(401).json({
                success: false,
                error: 'Authentication required',
            });
        }
        await AuthService_1.AuthService.changePassword(req.user.userId, req.body.currentPassword, req.body.newPassword);
        const response = {
            success: true,
            message: 'Password changed successfully',
        };
        res.json(response);
    }
    catch (error) {
        next(error);
    }
});
// Get current user profile
router.get('/profile', async (req, res, next) => {
    try {
        if (!req.user) {
            return res.status(401).json({
                success: false,
                error: 'Authentication required',
            });
        }
        const user = await AuthService_1.AuthService.getCurrentUser(req.user.userId);
        const response = {
            success: true,
            data: user,
            message: 'Profile retrieved successfully',
        };
        res.json(response);
    }
    catch (error) {
        next(error);
    }
});
// Get user by ID (admin only)
router.get('/users/:id', validation_1.validateUserIdParam, async (req, res, next) => {
    try {
        if (!req.user) {
            return res.status(401).json({
                success: false,
                error: 'Authentication required',
            });
        }
        const user = await AuthService_1.AuthService.getCurrentUser(req.params.id);
        const response = {
            success: true,
            data: user,
            message: 'User retrieved successfully',
        };
        res.json(response);
    }
    catch (error) {
        next(error);
    }
});
exports.default = router;
