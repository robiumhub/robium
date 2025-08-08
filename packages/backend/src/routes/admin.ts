import { Router, Response, NextFunction } from 'express';
import { AuthRequest, ApiResponse, UserRole } from '../types';
import {
  requireAdmin,
  requirePermission,
  logPermissionChecks,
  addUserPermissions,
} from '../middleware/rbac';
import { authenticateToken } from '../middleware/auth';
import { Permission, PermissionUtils } from '../utils/permissions';
import { UserModel } from '../models/User';
import { Database } from '../utils/database';
import {
  validatePagination,
  validateAdminUserUpdate,
  validateUUIDParam,
  sanitizeInput,
} from '../middleware/validation';

const router = Router();

// Apply authentication and permission middleware
router.use(authenticateToken);
router.use(logPermissionChecks);
router.use(addUserPermissions);
router.use(sanitizeInput);
// Project actions (archive/restore/delete)
router.post(
  '/projects/:id/actions',
  validateUUIDParam('id'),
  requirePermission(Permission.ADMIN_ALL),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const { id } = req.params;
      const { action } = req.body as {
        action: 'archive' | 'restore' | 'delete';
      };

      if (!action) {
        return res
          .status(400)
          .json({ success: false, error: 'Action is required' });
      }

      switch (action) {
        case 'archive':
          await Database.query(
            `UPDATE projects SET is_active = false, updated_at = NOW() WHERE id = $1`,
            [id]
          );
          break;
        case 'restore':
          await Database.query(
            `UPDATE projects SET is_active = true, updated_at = NOW() WHERE id = $1`,
            [id]
          );
          break;
        case 'delete':
          await Database.query(
            `DELETE FROM project_files WHERE project_id = $1`,
            [id]
          );
          await Database.query(
            `DELETE FROM project_packages WHERE project_id = $1`,
            [id]
          );
          await Database.query(
            `DELETE FROM project_module_dependencies WHERE project_id = $1`,
            [id]
          );
          await Database.query(`DELETE FROM projects WHERE id = $1`, [id]);
          break;
        default:
          return res
            .status(400)
            .json({ success: false, error: 'Unsupported action' });
      }

      res.json({ success: true, message: `Project ${action}d successfully` });
    } catch (error) {
      next(error);
    }
  }
);

// User actions (promote/demote/disable/enable)
router.post(
  '/users/:id/actions',
  validateUUIDParam('id'),
  requirePermission(Permission.USER_UPDATE),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const { id } = req.params;
      const { action } = req.body as {
        action: 'promote' | 'demote' | 'disable' | 'enable';
      };

      if (!action) {
        return res
          .status(400)
          .json({ success: false, error: 'Action is required' });
      }

      switch (action) {
        case 'promote':
          await UserModel.update(id, { role: UserRole.ADMIN });
          break;
        case 'demote':
          await UserModel.update(id, { role: UserRole.USER });
          break;
        case 'disable':
          await Database.query(
            `UPDATE users SET is_active = false WHERE id = $1`,
            [id]
          );
          break;
        case 'enable':
          await Database.query(
            `UPDATE users SET is_active = true WHERE id = $1`,
            [id]
          );
          break;
        default:
          return res
            .status(400)
            .json({ success: false, error: 'Unsupported action' });
      }

      res.json({ success: true, message: `User ${action}d successfully` });
    } catch (error) {
      next(error);
    }
  }
);

// Admin dashboard
router.get(
  '/dashboard',
  requireAdmin,
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const roleCounts = await UserModel.getCountByRole();
      const userCount = roleCounts[UserRole.USER] + roleCounts[UserRole.ADMIN];
      const adminCount = roleCounts[UserRole.ADMIN];

      const response: ApiResponse = {
        success: true,
        data: {
          totalUsers: userCount,
          adminUsers: adminCount,
          regularUsers: roleCounts[UserRole.USER],
          systemHealth: 'healthy',
          lastUpdated: new Date().toISOString(),
        },
        message: 'Admin dashboard data retrieved successfully',
      };

      res.json(response);
    } catch (error) {
      next(error);
    }
  }
);

// List all users with pagination
router.get(
  '/users',
  validatePagination,
  requirePermission(Permission.USER_READ),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const { page, limit } = req.query;

      const result = await UserModel.findAll(Number(page), Number(limit));

      const response: ApiResponse = {
        success: true,
        data: result.users,
        message: 'Users retrieved successfully',
      };

      res.json(response);
    } catch (error) {
      next(error);
    }
  }
);

// Get user by ID
router.get(
  '/users/:id',
  validateUUIDParam('id'),
  requirePermission(Permission.USER_READ),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const user = await UserModel.findById(req.params.id);

      if (!user) {
        return res.status(404).json({
          success: false,
          error: 'User not found',
        });
      }

      const response: ApiResponse = {
        success: true,
        data: user,
        message: 'User retrieved successfully',
      };

      res.json(response);
    } catch (error) {
      next(error);
    }
  }
);

// Update user (admin only)
router.put(
  '/users/:id',
  validateUUIDParam('id'),
  validateAdminUserUpdate,
  requirePermission(Permission.USER_UPDATE),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const updatedUser = await UserModel.update(req.params.id, req.body);

      const response: ApiResponse = {
        success: true,
        data: updatedUser,
        message: 'User updated successfully',
      };

      res.json(response);
    } catch (error) {
      next(error);
    }
  }
);

// Delete user (admin only)
router.delete(
  '/users/:id',
  validateUUIDParam('id'),
  requirePermission(Permission.USER_DELETE),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      await UserModel.delete(req.params.id);

      const response: ApiResponse = {
        success: true,
        message: 'User deleted successfully',
      };

      res.json(response);
    } catch (error) {
      next(error);
    }
  }
);

// Get permissions info
router.get(
  '/permissions',
  requirePermission(Permission.ADMIN_ALL),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const adminPermissions = PermissionUtils.getRolePermissions(
        UserRole.ADMIN
      );
      const userPermissions = PermissionUtils.getRolePermissions(UserRole.USER);

      const response: ApiResponse = {
        success: true,
        data: {
          adminPermissions,
          userPermissions,
          availableRoles: Object.values(UserRole),
        },
        message: 'Permissions information retrieved successfully',
      };

      res.json(response);
    } catch (error) {
      next(error);
    }
  }
);

// Promote user to admin
router.post(
  '/users/:id/promote',
  validateUUIDParam('id'),
  requirePermission(Permission.USER_UPDATE),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const updatedUser = await UserModel.update(req.params.id, {
        role: UserRole.ADMIN,
      });

      const response: ApiResponse = {
        success: true,
        data: updatedUser,
        message: 'User promoted to admin successfully',
      };

      res.json(response);
    } catch (error) {
      next(error);
    }
  }
);

// Demote admin to user
router.post(
  '/users/:id/demote',
  validateUUIDParam('id'),
  requirePermission(Permission.USER_UPDATE),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const updatedUser = await UserModel.update(req.params.id, {
        role: UserRole.USER,
      });

      const response: ApiResponse = {
        success: true,
        data: updatedUser,
        message: 'Admin demoted to user successfully',
      };

      res.json(response);
    } catch (error) {
      next(error);
    }
  }
);

// System health check
router.get(
  '/system/health',
  requirePermission(Permission.ADMIN_ALL),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const response: ApiResponse = {
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
    } catch (error) {
      next(error);
    }
  }
);

// System restart (mock endpoint)
router.post(
  '/system/restart',
  requirePermission(Permission.ADMIN_ALL),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const response: ApiResponse = {
        success: true,
        message: 'System restart initiated (mock endpoint)',
      };

      res.json(response);
    } catch (error) {
      next(error);
    }
  }
);

export default router;
