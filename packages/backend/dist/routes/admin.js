"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const express_1 = require("express");
const types_1 = require("../types");
const rbac_1 = require("../middleware/rbac");
const auth_1 = require("../middleware/auth");
const permissions_1 = require("../utils/permissions");
const User_1 = require("../models/User");
const validation_1 = require("../middleware/validation");
const router = (0, express_1.Router)();
// Apply authentication and permission middleware
router.use(auth_1.authenticateToken);
router.use(rbac_1.logPermissionChecks);
router.use(rbac_1.addUserPermissions);
router.use(validation_1.sanitizeInput);
// Admin dashboard
router.get('/dashboard', rbac_1.requireAdmin, async (req, res, next) => {
    try {
        const roleCounts = await User_1.UserModel.getCountByRole();
        const userCount = roleCounts[types_1.UserRole.USER] + roleCounts[types_1.UserRole.ADMIN];
        const adminCount = roleCounts[types_1.UserRole.ADMIN];
        const response = {
            success: true,
            data: {
                totalUsers: userCount,
                adminUsers: adminCount,
                regularUsers: roleCounts[types_1.UserRole.USER],
                systemHealth: 'healthy',
                lastUpdated: new Date().toISOString(),
            },
            message: 'Admin dashboard data retrieved successfully',
        };
        res.json(response);
    }
    catch (error) {
        next(error);
    }
});
// List all users with pagination
router.get('/users', validation_1.validatePagination, (0, rbac_1.requirePermission)(permissions_1.Permission.USER_READ), async (req, res, next) => {
    try {
        const { page, limit } = req.query;
        const result = await User_1.UserModel.findAll(Number(page), Number(limit));
        const response = {
            success: true,
            data: result.users,
            message: 'Users retrieved successfully',
        };
        res.json(response);
    }
    catch (error) {
        next(error);
    }
});
// Get user by ID
router.get('/users/:id', (0, validation_1.validateUUIDParam)('id'), (0, rbac_1.requirePermission)(permissions_1.Permission.USER_READ), async (req, res, next) => {
    try {
        const user = await User_1.UserModel.findById(req.params.id);
        if (!user) {
            return res.status(404).json({
                success: false,
                error: 'User not found',
            });
        }
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
// Update user (admin only)
router.put('/users/:id', (0, validation_1.validateUUIDParam)('id'), validation_1.validateAdminUserUpdate, (0, rbac_1.requirePermission)(permissions_1.Permission.USER_UPDATE), async (req, res, next) => {
    try {
        const updatedUser = await User_1.UserModel.update(req.params.id, req.body);
        const response = {
            success: true,
            data: updatedUser,
            message: 'User updated successfully',
        };
        res.json(response);
    }
    catch (error) {
        next(error);
    }
});
// Delete user (admin only)
router.delete('/users/:id', (0, validation_1.validateUUIDParam)('id'), (0, rbac_1.requirePermission)(permissions_1.Permission.USER_DELETE), async (req, res, next) => {
    try {
        await User_1.UserModel.delete(req.params.id);
        const response = {
            success: true,
            message: 'User deleted successfully',
        };
        res.json(response);
    }
    catch (error) {
        next(error);
    }
});
// Get permissions info
router.get('/permissions', (0, rbac_1.requirePermission)(permissions_1.Permission.ADMIN_ALL), async (req, res, next) => {
    try {
        const adminPermissions = permissions_1.PermissionUtils.getRolePermissions(types_1.UserRole.ADMIN);
        const userPermissions = permissions_1.PermissionUtils.getRolePermissions(types_1.UserRole.USER);
        const response = {
            success: true,
            data: {
                adminPermissions,
                userPermissions,
                availableRoles: Object.values(types_1.UserRole),
            },
            message: 'Permissions information retrieved successfully',
        };
        res.json(response);
    }
    catch (error) {
        next(error);
    }
});
// Promote user to admin
router.post('/users/:id/promote', (0, validation_1.validateUUIDParam)('id'), (0, rbac_1.requirePermission)(permissions_1.Permission.USER_UPDATE), async (req, res, next) => {
    try {
        const updatedUser = await User_1.UserModel.update(req.params.id, {
            role: types_1.UserRole.ADMIN,
        });
        const response = {
            success: true,
            data: updatedUser,
            message: 'User promoted to admin successfully',
        };
        res.json(response);
    }
    catch (error) {
        next(error);
    }
});
// Demote admin to user
router.post('/users/:id/demote', (0, validation_1.validateUUIDParam)('id'), (0, rbac_1.requirePermission)(permissions_1.Permission.USER_UPDATE), async (req, res, next) => {
    try {
        const updatedUser = await User_1.UserModel.update(req.params.id, {
            role: types_1.UserRole.USER,
        });
        const response = {
            success: true,
            data: updatedUser,
            message: 'Admin demoted to user successfully',
        };
        res.json(response);
    }
    catch (error) {
        next(error);
    }
});
// System health check
router.get('/system/health', (0, rbac_1.requirePermission)(permissions_1.Permission.ADMIN_ALL), async (req, res, next) => {
    try {
        const response = {
            success: true,
            data: {
                status: 'healthy',
                timestamp: new Date().toISOString(),
                uptime: process.uptime(),
                memory: process.memoryUsage(),
                version: process.version,
            },
            message: 'System health check completed',
        };
        res.json(response);
    }
    catch (error) {
        next(error);
    }
});
// System restart (mock endpoint)
router.post('/system/restart', (0, rbac_1.requirePermission)(permissions_1.Permission.ADMIN_ALL), async (req, res, next) => {
    try {
        const response = {
            success: true,
            message: 'System restart initiated (mock endpoint)',
        };
        res.json(response);
    }
    catch (error) {
        next(error);
    }
});
exports.default = router;
