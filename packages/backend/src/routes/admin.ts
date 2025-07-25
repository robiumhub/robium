import { Router, Request, Response, NextFunction } from 'express';
import { AuthRequest, ApiResponse, UserRole } from '../types';
import {
  requireAdmin,
  requirePermission,
  requireAnyPermission,
  requireAllPermissions,
  logPermissionChecks,
  addUserPermissions,
} from '../middleware/rbac';
import { authenticateToken } from '../middleware/auth';
import {
  Permission,
  PermissionUtils,
  PermissionGroups,
} from '../utils/permissions';
import { UserModel } from '../models/User';

const router = Router();

// Apply authentication and permission logging to all admin routes
router.use(authenticateToken);
router.use(logPermissionChecks);
router.use(addUserPermissions);

// GET /admin/dashboard - Admin dashboard (requires admin role)
router.get(
  '/dashboard',
  requireAdmin,
  async (
    req: AuthRequest,
    res: Response,
    next: NextFunction
  ): Promise<void> => {
    try {
      if (!req.user) {
        throw new Error('User not authenticated');
      }

      // Get system statistics
      const userCount = await UserModel.getCountByRole(UserRole.USER);
      const adminCount = await UserModel.getCountByRole(UserRole.ADMIN);

      const response: ApiResponse = {
        success: true,
        message: 'Admin dashboard data retrieved successfully',
        data: {
          user: req.user,
          permissions: req.userPermissions,
          statistics: {
            totalUsers: (userCount as number) + (adminCount as number),
            regularUsers: userCount as number,
            admins: adminCount as number,
          },
          systemInfo: {
            timestamp: new Date().toISOString(),
            version: process.env.npm_package_version || '1.0.0',
          },
        },
      };

      res.status(200).json(response);
    } catch (error) {
      next(error);
    }
  }
);

// GET /admin/users - List all users (requires USER_READ permission)
router.get(
  '/users',
  requirePermission(Permission.USER_READ),
  async (
    req: AuthRequest,
    res: Response,
    next: NextFunction
  ): Promise<void> => {
    try {
      const page = parseInt(req.query.page as string) || 1;
      const limit = parseInt(req.query.limit as string) || 10;

      const users = await UserModel.findAll(page, limit);

      const response: ApiResponse = {
        success: true,
        message: 'Users retrieved successfully',
        data: {
          users: users.users,
          pagination: {
            page: users.page,
            limit: users.limit,
            total: users.total,
            totalPages: Math.ceil(users.total / users.limit),
          },
        },
      };

      res.status(200).json(response);
    } catch (error) {
      next(error);
    }
  }
);

// GET /admin/users/:id - Get specific user (requires USER_READ permission)
router.get(
  '/users/:id',
  requirePermission(Permission.USER_READ),
  async (
    req: AuthRequest,
    res: Response,
    next: NextFunction
  ): Promise<void> => {
    try {
      const userId = req.params.id;
      const user = await UserModel.findById(userId);

      if (!user) {
        res.status(404).json({
          success: false,
          message: 'User not found',
        });
        return;
      }

      const response: ApiResponse = {
        success: true,
        message: 'User retrieved successfully',
        data: { user },
      };

      res.status(200).json(response);
    } catch (error) {
      next(error);
    }
  }
);

// PUT /admin/users/:id - Update user (requires USER_UPDATE permission)
router.put(
  '/users/:id',
  requirePermission(Permission.USER_UPDATE),
  async (
    req: AuthRequest,
    res: Response,
    next: NextFunction
  ): Promise<void> => {
    try {
      const userId = req.params.id;
      const updates = req.body;

      const updatedUser = await UserModel.update(userId, updates);

      const response: ApiResponse = {
        success: true,
        message: 'User updated successfully',
        data: { user: updatedUser },
      };

      res.status(200).json(response);
    } catch (error) {
      next(error);
    }
  }
);

// DELETE /admin/users/:id - Delete user (requires USER_DELETE permission)
router.delete(
  '/users/:id',
  requirePermission(Permission.USER_DELETE),
  async (
    req: AuthRequest,
    res: Response,
    next: NextFunction
  ): Promise<void> => {
    try {
      const userId = req.params.id;

      // Prevent admin from deleting themselves
      if (req.user && userId === req.user.userId) {
        res.status(400).json({
          success: false,
          message: 'Cannot delete your own account',
        });
        return;
      }

      await UserModel.delete(userId);

      const response: ApiResponse = {
        success: true,
        message: 'User deleted successfully',
      };

      res.status(200).json(response);
    } catch (error) {
      next(error);
    }
  }
);

// GET /admin/permissions - Get user permissions (requires any system permission)
router.get(
  '/permissions',
  requireAnyPermission([Permission.SYSTEM_READ, Permission.SYSTEM_ADMIN]),
  async (
    req: AuthRequest,
    res: Response,
    next: NextFunction
  ): Promise<void> => {
    try {
      if (!req.user) {
        throw new Error('User not authenticated');
      }

      const userPermissions = PermissionUtils.getRolePermissions(req.user.role);
      const permissionDescriptions = userPermissions.map((permission) => ({
        permission,
        description: PermissionUtils.getPermissionDescription(permission),
      }));

      const response: ApiResponse = {
        success: true,
        message: 'User permissions retrieved successfully',
        data: {
          user: {
            id: req.user.userId,
            role: req.user.role,
          },
          permissions: permissionDescriptions,
          permissionGroups: {
            userManagement: PermissionUtils.hasAnyPermission(
              req.user.role,
              PermissionGroups.USER_MANAGEMENT
            ),
            projectManagement: PermissionUtils.hasAnyPermission(
              req.user.role,
              PermissionGroups.PROJECT_MANAGEMENT
            ),
            sessionManagement: PermissionUtils.hasAnyPermission(
              req.user.role,
              PermissionGroups.SESSION_MANAGEMENT
            ),
            profileManagement: PermissionUtils.hasAnyPermission(
              req.user.role,
              PermissionGroups.PROFILE_MANAGEMENT
            ),
            authentication: PermissionUtils.hasAnyPermission(
              req.user.role,
              PermissionGroups.AUTHENTICATION
            ),
            systemAdministration: PermissionUtils.hasAnyPermission(
              req.user.role,
              PermissionGroups.SYSTEM_ADMINISTRATION
            ),
          },
        },
      };

      res.status(200).json(response);
    } catch (error) {
      next(error);
    }
  }
);

// POST /admin/users/:id/promote - Promote user to admin (requires USER_UPDATE and SYSTEM_ADMIN permissions)
router.post(
  '/users/:id/promote',
  requireAllPermissions([Permission.USER_UPDATE, Permission.SYSTEM_ADMIN]),
  async (
    req: AuthRequest,
    res: Response,
    next: NextFunction
  ): Promise<void> => {
    try {
      const userId = req.params.id;

      // Prevent admin from promoting themselves (they're already admin)
      if (req.user && userId === req.user.userId) {
        res.status(400).json({
          success: false,
          message: 'User is already an admin',
        });
        return;
      }

      const updatedUser = await UserModel.update(userId, {
        role: UserRole.ADMIN,
      });

      const response: ApiResponse = {
        success: true,
        message: 'User promoted to admin successfully',
        data: { user: updatedUser },
      };

      res.status(200).json(response);
    } catch (error) {
      next(error);
    }
  }
);

// POST /admin/users/:id/demote - Demote admin to user (requires USER_UPDATE and SYSTEM_ADMIN permissions)
router.post(
  '/users/:id/demote',
  requireAllPermissions([Permission.USER_UPDATE, Permission.SYSTEM_ADMIN]),
  async (
    req: AuthRequest,
    res: Response,
    next: NextFunction
  ): Promise<void> => {
    try {
      const userId = req.params.id;

      // Prevent admin from demoting themselves
      if (req.user && userId === req.user.userId) {
        res.status(400).json({
          success: false,
          message: 'Cannot demote your own account',
        });
        return;
      }

      const updatedUser = await UserModel.update(userId, {
        role: UserRole.USER,
      });

      const response: ApiResponse = {
        success: true,
        message: 'Admin demoted to user successfully',
        data: { user: updatedUser },
      };

      res.status(200).json(response);
    } catch (error) {
      next(error);
    }
  }
);

// GET /admin/system/health - System health check (requires SYSTEM_READ permission)
router.get(
  '/system/health',
  requirePermission(Permission.SYSTEM_READ),
  async (
    req: AuthRequest,
    res: Response,
    next: NextFunction
  ): Promise<void> => {
    try {
      const response: ApiResponse = {
        success: true,
        message: 'System health check completed',
        data: {
          status: 'healthy',
          timestamp: new Date().toISOString(),
          uptime: process.uptime(),
          memory: process.memoryUsage(),
          environment: process.env.NODE_ENV || 'development',
          version: process.env.npm_package_version || '1.0.0',
        },
      };

      res.status(200).json(response);
    } catch (error) {
      next(error);
    }
  }
);

// POST /admin/system/restart - System restart (requires SYSTEM_ADMIN permission)
router.post(
  '/system/restart',
  requirePermission(Permission.SYSTEM_ADMIN),
  async (
    req: AuthRequest,
    res: Response,
    next: NextFunction
  ): Promise<void> => {
    try {
      const response: ApiResponse = {
        success: true,
        message: 'System restart initiated',
        data: {
          timestamp: new Date().toISOString(),
          initiatedBy: req.user?.userId,
        },
      };

      res.status(200).json(response);

      // In a real application, you would implement actual system restart logic here
      // For demo purposes, we just log the restart request
      console.log(`System restart requested by admin: ${req.user?.userId}`);
    } catch (error) {
      next(error);
    }
  }
);

// Error handling middleware specific to admin routes
router.use(
  (error: Error, req: Request, res: Response, next: NextFunction): void => {
    // Log admin route errors for security monitoring
    console.log('Admin route error:', {
      message: error.message,
      stack: error.stack,
      ip: req.ip,
      userAgent: req.get('User-Agent'),
      timestamp: new Date().toISOString(),
      endpoint: req.originalUrl,
      method: req.method,
    });

    // Pass error to global error handler
    next(error);
  }
);

export default router;
