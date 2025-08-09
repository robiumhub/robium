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
import { logger } from '../utils/logger';
import { securityAudit } from '../utils/securityAudit';
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

// Create new user (admin only)
router.post(
  '/users',
  requirePermission(Permission.USER_CREATE),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const { username, email, password, role = 'user' } = req.body;

      // Validate required fields
      if (!username || !email || !password) {
        return res.status(400).json({
          success: false,
          error: 'Username, email, and password are required',
        });
      }

      // Validate role
      if (!Object.values(UserRole).includes(role)) {
        return res.status(400).json({
          success: false,
          error: 'Invalid role. Must be "user" or "admin"',
        });
      }

      // Create the user
      const newUser = await UserModel.create({
        username,
        email,
        password,
        role: role as UserRole,
      });

      const response: ApiResponse = {
        success: true,
        data: newUser,
        message: 'User created successfully',
      };

      res.status(201).json(response);
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

// Project overview statistics
router.get(
  '/projects/overview',
  requirePermission(Permission.ADMIN_ALL),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      // Get project counts by status
      const statusCounts = await Database.query(`
        SELECT 
          is_active,
          is_template,
          COUNT(*) as count
        FROM projects 
        GROUP BY is_active, is_template
      `);

      // Get project counts by type
      const typeCounts = await Database.query(`
        SELECT 
          type,
          COUNT(*) as count
        FROM projects 
        GROUP BY type
      `);

      // Get project creation rate (last 30 days)
      const creationRate = await Database.query(`
        SELECT 
          DATE(created_at) as date,
          COUNT(*) as count
        FROM projects 
        WHERE created_at >= NOW() - INTERVAL '30 days'
        GROUP BY DATE(created_at)
        ORDER BY date DESC
      `);

      // Get most active projects (by update frequency)
      const activeProjects = await Database.query(`
        SELECT 
          p.id,
          p.name,
          p.owner_id,
          u.username as owner_name,
          p.created_at,
          p.updated_at,
          p.is_active,
          p.is_template,
          p.type,
          p.tags,
          EXTRACT(EPOCH FROM (NOW() - p.updated_at))/3600 as hours_since_update
        FROM projects p
        LEFT JOIN users u ON p.owner_id = u.id
        ORDER BY p.updated_at DESC
        LIMIT 10
      `);

      // Get projects by owner distribution
      const ownerDistribution = await Database.query(`
        SELECT 
          u.username,
          u.email,
          COUNT(p.id) as project_count
        FROM users u
        LEFT JOIN projects p ON u.id = p.owner_id
        GROUP BY u.id, u.username, u.email
        ORDER BY project_count DESC
      `);

      // Get total project statistics
      const totalStats = await Database.query(`
        SELECT 
          COUNT(*) as total_projects,
          COUNT(CASE WHEN is_active = true THEN 1 END) as active_projects,
          COUNT(CASE WHEN is_template = true THEN 1 END) as template_projects,
          COUNT(CASE WHEN is_active = false THEN 1 END) as archived_projects,
          AVG(EXTRACT(EPOCH FROM (NOW() - created_at))/86400) as avg_project_age_days
        FROM projects
      `);

      // Get recent activity (last 7 days)
      const recentActivity = await Database.query(`
        SELECT 
          p.id,
          p.name,
          p.updated_at,
          u.username as updated_by
        FROM projects p
        LEFT JOIN users u ON p.updated_by = u.id
        WHERE p.updated_at >= NOW() - INTERVAL '7 days'
        ORDER BY p.updated_at DESC
        LIMIT 20
      `);

      const response: ApiResponse = {
        success: true,
        data: {
          summary: {
            totalProjects: parseInt(
              (totalStats as any).rows[0]?.total_projects || '0'
            ),
            activeProjects: parseInt(
              (totalStats as any).rows[0]?.active_projects || '0'
            ),
            templateProjects: parseInt(
              (totalStats as any).rows[0]?.template_projects || '0'
            ),
            archivedProjects: parseInt(
              (totalStats as any).rows[0]?.archived_projects || '0'
            ),
            avgProjectAgeDays: parseFloat(
              (totalStats as any).rows[0]?.avg_project_age_days || '0'
            ),
          },
          statusDistribution: (statusCounts as any).rows,
          typeDistribution: (typeCounts as any).rows,
          creationTrend: (creationRate as any).rows,
          mostActiveProjects: (activeProjects as any).rows,
          ownerDistribution: (ownerDistribution as any).rows,
          recentActivity: (recentActivity as any).rows,
          lastUpdated: new Date().toISOString(),
        },
        message: 'Project overview data retrieved successfully',
      };

      res.json(response);
    } catch (error) {
      next(error);
    }
  }
);

// Module overview statistics
router.get(
  '/modules/overview',
  requirePermission(Permission.ADMIN_ALL),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      // Get module counts by status and type
      const statusCounts = await Database.query(`
        SELECT 
          is_active,
          is_public,
          is_default,
          COUNT(*) as count
        FROM modules 
        GROUP BY is_active, is_public, is_default
      `);

      // Get module counts by type
      const typeCounts = await Database.query(`
        SELECT 
          type,
          COUNT(*) as count
        FROM modules 
        GROUP BY type
      `);

      // Get module counts by category
      const categoryCounts = await Database.query(`
        SELECT 
          category,
          COUNT(*) as count
        FROM modules 
        WHERE category IS NOT NULL
        GROUP BY category
        ORDER BY count DESC
      `);

      // Get most popular modules (by usage in projects)
      const popularModules = await Database.query(`
        SELECT 
          m.id,
          m.name,
          m.type,
          m.category,
          m.is_active,
          m.is_public,
          m.is_default,
          m.created_at,
          m.updated_at,
          COUNT(pmd.project_id) as usage_count
        FROM modules m
        LEFT JOIN project_module_dependencies pmd ON m.id = pmd.module_id
        GROUP BY m.id, m.name, m.type, m.category, m.is_active, m.is_public, m.is_default, m.created_at, m.updated_at
        ORDER BY usage_count DESC, m.name
        LIMIT 10
      `);

      // Get modules by supported robots
      const robotSupport = await Database.query(`
        SELECT 
          unnest(supported_robots) as robot_type,
          COUNT(*) as module_count
        FROM modules 
        WHERE supported_robots IS NOT NULL AND array_length(supported_robots, 1) > 0
        GROUP BY robot_type
        ORDER BY module_count DESC
      `);

      // Get recent module activity
      const recentActivity = await Database.query(`
        SELECT 
          m.id,
          m.name,
          m.updated_at,
          m.type,
          m.category
        FROM modules m
        WHERE m.updated_at >= NOW() - INTERVAL '30 days'
        ORDER BY m.updated_at DESC
        LIMIT 20
      `);

      // Get total module statistics
      const totalStats = await Database.query(`
        SELECT 
          COUNT(*) as total_modules,
          COUNT(CASE WHEN is_active = true THEN 1 END) as active_modules,
          COUNT(CASE WHEN is_public = true THEN 1 END) as public_modules,
          COUNT(CASE WHEN is_default = true THEN 1 END) as default_modules,
          AVG(EXTRACT(EPOCH FROM (NOW() - created_at))/86400) as avg_module_age_days
        FROM modules
      `);

      // Get module usage distribution
      const usageDistribution = await Database.query(`
        SELECT 
          usage_bucket,
          COUNT(*) as module_count
        FROM (
          SELECT 
            CASE 
              WHEN usage_count = 0 THEN 'Not Used'
              WHEN usage_count = 1 THEN 'Used in 1 Project'
              WHEN usage_count <= 5 THEN 'Used in 2-5 Projects'
              WHEN usage_count <= 10 THEN 'Used in 6-10 Projects'
              ELSE 'Used in 10+ Projects'
            END as usage_bucket
          FROM (
            SELECT COUNT(pmd.project_id) as usage_count
            FROM modules m
            LEFT JOIN project_module_dependencies pmd ON m.id = pmd.module_id
            GROUP BY m.id
          ) usage_counts
        ) buckets
        GROUP BY usage_bucket
        ORDER BY 
          CASE usage_bucket
            WHEN 'Not Used' THEN 1
            WHEN 'Used in 1 Project' THEN 2
            WHEN 'Used in 2-5 Projects' THEN 3
            WHEN 'Used in 6-10 Projects' THEN 4
            ELSE 5
          END
      `);

      const response: ApiResponse = {
        success: true,
        data: {
          summary: {
            totalModules: parseInt(
              (totalStats as any).rows[0]?.total_modules || '0'
            ),
            activeModules: parseInt(
              (totalStats as any).rows[0]?.active_modules || '0'
            ),
            publicModules: parseInt(
              (totalStats as any).rows[0]?.public_modules || '0'
            ),
            defaultModules: parseInt(
              (totalStats as any).rows[0]?.default_modules || '0'
            ),
            avgModuleAgeDays: parseFloat(
              (totalStats as any).rows[0]?.avg_module_age_days || '0'
            ),
          },
          statusDistribution: (statusCounts as any).rows,
          typeDistribution: (typeCounts as any).rows,
          categoryDistribution: (categoryCounts as any).rows,
          popularModules: (popularModules as any).rows,
          robotSupport: (robotSupport as any).rows,
          recentActivity: (recentActivity as any).rows,
          usageDistribution: (usageDistribution as any).rows,
          lastUpdated: new Date().toISOString(),
        },
        message: 'Module overview data retrieved successfully',
      };

      res.json(response);
    } catch (error) {
      next(error);
    }
  }
);

// Template overview statistics
router.get(
  '/templates/overview',
  requirePermission(Permission.ADMIN_ALL),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      // Get template counts by status
      const statusCounts = await Database.query(`
        SELECT 
          is_active,
          COUNT(*) as count
        FROM projects 
        WHERE is_template = true
        GROUP BY is_active
      `);

      // Get template counts by type
      const typeCounts = await Database.query(`
        SELECT 
          COALESCE(type, 'custom') as type,
          COUNT(*) as count
        FROM projects 
        WHERE is_template = true
        GROUP BY type
        ORDER BY count DESC
      `);

      // Get most popular templates (by usage count - this would need a separate table in a real implementation)
      const popularTemplates = await Database.query(`
        SELECT 
          p.id,
          p.name,
          p.type,
          p.is_active,
          p.created_at,
          p.updated_at,
          p.owner_id,
          u.username as owner_name,
          p.tags,
          EXTRACT(EPOCH FROM (NOW() - p.updated_at))/3600 as hours_since_update
        FROM projects p
        LEFT JOIN users u ON p.owner_id = u.id
        WHERE p.is_template = true
        ORDER BY p.updated_at DESC
        LIMIT 10
      `);

      // Get recent template activity
      const recentActivity = await Database.query(`
        SELECT 
          p.id,
          p.name,
          p.updated_at,
          p.type,
          u.username as updated_by
        FROM projects p
        LEFT JOIN users u ON p.owner_id = u.id
        WHERE p.is_template = true 
        AND p.updated_at >= NOW() - INTERVAL '30 days'
        ORDER BY p.updated_at DESC
        LIMIT 20
      `);

      // Get total template statistics
      const totalStats = await Database.query(`
        SELECT 
          COUNT(*) as total_templates,
          COUNT(CASE WHEN is_active = true THEN 1 END) as active_templates,
          AVG(EXTRACT(EPOCH FROM (NOW() - created_at))/86400) as avg_template_age_days
        FROM projects
        WHERE is_template = true
      `);

      // Get template creation trend
      const creationTrend = await Database.query(`
        SELECT 
          DATE(created_at) as date,
          COUNT(*) as count
        FROM projects 
        WHERE is_template = true 
        AND created_at >= NOW() - INTERVAL '30 days'
        GROUP BY DATE(created_at)
        ORDER BY date DESC
      `);

      // Get template owner distribution
      const ownerDistribution = await Database.query(`
        SELECT 
          u.username,
          u.email,
          COUNT(p.id) as template_count
        FROM users u
        LEFT JOIN projects p ON u.id = p.owner_id AND p.is_template = true
        GROUP BY u.id, u.username, u.email
        HAVING COUNT(p.id) > 0
        ORDER BY template_count DESC
      `);

      const response: ApiResponse = {
        success: true,
        data: {
          summary: {
            totalTemplates: parseInt(
              (totalStats as any).rows[0]?.total_templates || '0'
            ),
            activeTemplates: parseInt(
              (totalStats as any).rows[0]?.active_templates || '0'
            ),
            avgTemplateAgeDays: parseFloat(
              (totalStats as any).rows[0]?.avg_template_age_days || '0'
            ),
          },
          statusDistribution: (statusCounts as any).rows,
          typeDistribution: (typeCounts as any).rows,
          popularTemplates: (popularTemplates as any).rows,
          recentActivity: (recentActivity as any).rows,
          creationTrend: (creationTrend as any).rows,
          ownerDistribution: (ownerDistribution as any).rows,
          lastUpdated: new Date().toISOString(),
        },
        message: 'Template overview data retrieved successfully',
      };

      res.json(response);
    } catch (error) {
      next(error);
    }
  }
);

// Robot overview statistics
router.get(
  '/robots/overview',
  requirePermission(Permission.ADMIN_ALL),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    const startTime = Date.now();
    const requestId =
      req.requestId ||
      `req_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;

    try {
      // Log access to robots oversight endpoint
      securityAudit.logAccess(
        req,
        'ROBOTS_OVERSIGHT',
        'VIEW_ROBOTS_OVERVIEW',
        true,
        { endpoint: '/admin/robots/overview', method: 'GET' }
      );
      // Get robot counts by type (derived from supported_robots in modules)
      const robotTypeCounts = await Database.query(`
        SELECT 
          robot_type,
          COUNT(*) as module_count
        FROM (
          SELECT DISTINCT unnest(supported_robots) AS robot_type
          FROM modules 
          WHERE is_active = true AND supported_robots IS NOT NULL AND array_length(supported_robots, 1) > 0
        ) AS robot_types
        GROUP BY robot_type
        ORDER BY module_count DESC
      `);

      // Get robot utilization by project (how many projects use each robot type)
      const robotUtilization = await Database.query(`
        SELECT 
          robot_type,
          COUNT(DISTINCT p.id) as project_count
        FROM (
          SELECT DISTINCT unnest(supported_robots) AS robot_type
          FROM modules 
          WHERE is_active = true AND supported_robots IS NOT NULL AND array_length(supported_robots, 1) > 0
        ) AS robot_types
        LEFT JOIN project_module_dependencies pmd ON pmd.module_id IN (
          SELECT id FROM modules 
          WHERE is_active = true AND robot_type = ANY(supported_robots)
        )
        LEFT JOIN projects p ON pmd.project_id = p.id AND p.is_active = true
        GROUP BY robot_type
        ORDER BY project_count DESC
      `);

      // Get most popular robot types (by module count)
      const popularRobots = await Database.query(`
        SELECT 
          robot_type,
          COUNT(m.id) as module_count,
          COUNT(DISTINCT p.id) as project_count
        FROM (
          SELECT DISTINCT unnest(supported_robots) AS robot_type
          FROM modules 
          WHERE is_active = true AND supported_robots IS NOT NULL AND array_length(supported_robots, 1) > 0
        ) AS robot_types
        LEFT JOIN modules m ON robot_type = ANY(m.supported_robots) AND m.is_active = true
        LEFT JOIN project_module_dependencies pmd ON m.id = pmd.module_id
        LEFT JOIN projects p ON pmd.project_id = p.id AND p.is_active = true
        GROUP BY robot_type
        ORDER BY module_count DESC, project_count DESC
        LIMIT 10
      `);

      // Get robot deployment distribution (how many projects use each robot)
      const deploymentDistribution = await Database.query(`
        WITH robot_deployments AS (
          SELECT 
            robot_type,
            COUNT(DISTINCT p.id) as project_count
          FROM (
            SELECT DISTINCT unnest(supported_robots) AS robot_type
            FROM modules 
            WHERE is_active = true AND supported_robots IS NOT NULL AND array_length(supported_robots, 1) > 0
          ) AS robot_types
          LEFT JOIN modules m ON robot_type = ANY(m.supported_robots) AND m.is_active = true
          LEFT JOIN project_module_dependencies pmd ON m.id = pmd.module_id
          LEFT JOIN projects p ON pmd.project_id = p.id AND p.is_active = true
          GROUP BY robot_type
        ),
        deployment_buckets AS (
          SELECT 
            CASE 
              WHEN project_count = 0 THEN 'Not Deployed'
              WHEN project_count = 1 THEN 'Deployed in 1 Project'
              WHEN project_count <= 3 THEN 'Deployed in 2-3 Projects'
              WHEN project_count <= 5 THEN 'Deployed in 4-5 Projects'
              ELSE 'Deployed in 5+ Projects'
            END as deployment_bucket,
            COUNT(*) as robot_count
          FROM robot_deployments
          GROUP BY 
            CASE 
              WHEN project_count = 0 THEN 'Not Deployed'
              WHEN project_count = 1 THEN 'Deployed in 1 Project'
              WHEN project_count <= 3 THEN 'Deployed in 2-3 Projects'
              WHEN project_count <= 5 THEN 'Deployed in 4-5 Projects'
              ELSE 'Deployed in 5+ Projects'
            END
        )
        SELECT deployment_bucket, robot_count
        FROM deployment_buckets
        ORDER BY 
          CASE deployment_bucket
            WHEN 'Not Deployed' THEN 1
            WHEN 'Deployed in 1 Project' THEN 2
            WHEN 'Deployed in 2-3 Projects' THEN 3
            WHEN 'Deployed in 4-5 Projects' THEN 4
            ELSE 5
          END
      `);

      // Get recent robot activity (modules updated recently that support robots)
      const recentActivity = await Database.query(`
        SELECT 
          m.id,
          m.name,
          m.type,
          m.category,
          m.updated_at,
          array_agg(DISTINCT robot_type) as supported_robots
        FROM modules m,
        LATERAL unnest(m.supported_robots) AS robot_type
        WHERE m.is_active = true 
        AND m.supported_robots IS NOT NULL 
        AND array_length(m.supported_robots, 1) > 0
        AND m.updated_at >= NOW() - INTERVAL '30 days'
        GROUP BY m.id, m.name, m.type, m.category, m.updated_at
        ORDER BY m.updated_at DESC
        LIMIT 20
      `);

      // Get total robot statistics
      const totalStats = await Database.query(`
        SELECT 
          COUNT(DISTINCT rt.robot_type) as total_robot_types,
          COUNT(DISTINCT m.id) as total_modules_with_robots,
          COUNT(DISTINCT p.id) as total_projects_using_robots,
          AVG(rmc.module_count) as avg_modules_per_robot
        FROM (
          SELECT DISTINCT unnest(supported_robots) AS robot_type
          FROM modules 
          WHERE is_active = true AND supported_robots IS NOT NULL AND array_length(supported_robots, 1) > 0
        ) AS rt
        LEFT JOIN (
          SELECT 
            rmc_rt.robot_type,
            COUNT(rmc_m.id) as module_count
          FROM (
            SELECT DISTINCT unnest(supported_robots) AS robot_type
            FROM modules 
            WHERE is_active = true AND supported_robots IS NOT NULL AND array_length(supported_robots, 1) > 0
          ) AS rmc_rt
          LEFT JOIN modules rmc_m ON rmc_rt.robot_type = ANY(rmc_m.supported_robots) AND rmc_m.is_active = true
          GROUP BY rmc_rt.robot_type
        ) rmc ON rt.robot_type = rmc.robot_type
        LEFT JOIN modules m ON rt.robot_type = ANY(m.supported_robots) AND m.is_active = true
        LEFT JOIN project_module_dependencies pmd ON m.id = pmd.module_id
        LEFT JOIN projects p ON pmd.project_id = p.id AND p.is_active = true
      `);

      // Get robot category distribution (based on module categories)
      const categoryDistribution = await Database.query(`
        SELECT 
          m.category,
          COUNT(DISTINCT robot_type) as robot_count
        FROM modules m,
        LATERAL unnest(m.supported_robots) AS robot_type
        WHERE m.is_active = true 
        AND m.supported_robots IS NOT NULL 
        AND array_length(m.supported_robots, 1) > 0
        AND m.category IS NOT NULL
        GROUP BY m.category
        ORDER BY robot_count DESC
      `);

      const response: ApiResponse = {
        success: true,
        data: {
          summary: {
            totalRobotTypes: parseInt(
              (totalStats as any).rows[0]?.total_robot_types || '0'
            ),
            totalModulesWithRobots: parseInt(
              (totalStats as any).rows[0]?.total_modules_with_robots || '0'
            ),
            totalProjectsUsingRobots: parseInt(
              (totalStats as any).rows[0]?.total_projects_using_robots || '0'
            ),
            avgModulesPerRobot: parseFloat(
              (totalStats as any).rows[0]?.avg_modules_per_robot || '0'
            ),
          },
          typeDistribution: (robotTypeCounts as any).rows,
          utilizationMetrics: (robotUtilization as any).rows,
          popularRobots: (popularRobots as any).rows,
          deploymentDistribution: (deploymentDistribution as any).rows,
          recentActivity: (recentActivity as any).rows,
          categoryDistribution: (categoryDistribution as any).rows,
          lastUpdated: new Date().toISOString(),
        },
        message: 'Robot overview data retrieved successfully',
      };

      const duration = Date.now() - startTime;

      // Log successful robots oversight data access
      logger.info('Robots oversight data retrieved successfully', {
        userId: req.user?.userId,
        userEmail: req.user?.email,
        requestId,
        duration: `${duration}ms`,
        dataPoints: {
          robotTypes: response.data.typeDistribution.length,
          categories: response.data.categoryDistribution.length,
          recentActivity: response.data.recentActivity.length,
        },
        timestamp: new Date().toISOString(),
      });

      res.json(response);
    } catch (error) {
      const duration = Date.now() - startTime;

      // Log error in robots oversight data access
      logger.error('Failed to retrieve robots oversight data', {
        userId: req.user?.userId,
        userEmail: req.user?.email,
        requestId,
        duration: `${duration}ms`,
        error: error instanceof Error ? error.message : 'Unknown error',
        stack: error instanceof Error ? error.stack : undefined,
        timestamp: new Date().toISOString(),
      });

      next(error);
    }
  }
);

// Module actions (activate/deactivate/delete)
router.post(
  '/modules/:id/actions',
  validateUUIDParam('id'),
  requirePermission(Permission.ADMIN_ALL),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const { id } = req.params;
      const { action } = req.body as {
        action: 'activate' | 'deactivate' | 'delete';
      };

      if (!action) {
        return res
          .status(400)
          .json({ success: false, error: 'Action is required' });
      }

      switch (action) {
        case 'activate':
          await Database.query(
            `UPDATE modules SET is_active = true, updated_at = NOW() WHERE id = $1`,
            [id]
          );
          break;
        case 'deactivate':
          await Database.query(
            `UPDATE modules SET is_active = false, updated_at = NOW() WHERE id = $1`,
            [id]
          );
          break;
        case 'delete':
          // First delete related data
          await Database.query(
            `DELETE FROM module_packages WHERE module_id = $1`,
            [id]
          );
          await Database.query(
            `DELETE FROM module_dependencies WHERE module_id = $1 OR dependency_module_id = $1`,
            [id]
          );
          await Database.query(
            `DELETE FROM project_module_dependencies WHERE module_id = $1`,
            [id]
          );
          // Then delete the module
          await Database.query(`DELETE FROM modules WHERE id = $1`, [id]);
          break;
        default:
          return res
            .status(400)
            .json({ success: false, error: 'Unsupported action' });
      }

      res.json({ success: true, message: `Module ${action}d successfully` });
    } catch (error) {
      next(error);
    }
  }
);

// Robot management endpoints
router.get(
  '/robots',
  requirePermission(Permission.ADMIN_ALL),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const result = await Database.query(`
      SELECT 
        robot_type as code,
        CASE robot_type
          WHEN 'turtlebot3' THEN 'TurtleBot 3'
          WHEN 'pioneer3at' THEN 'Pioneer 3-AT'
          WHEN 'kobuki' THEN 'Kobuki Base'
          WHEN 'create3' THEN 'iRobot Create 3'
          WHEN 'jackal' THEN 'Clearpath Jackal'
          WHEN 'husky' THEN 'Clearpath Husky'
          WHEN 'clearpath_robots' THEN 'Clearpath Robots'
          WHEN 'pr2' THEN 'Willow Garage PR2'
          WHEN 'fetch' THEN 'Fetch Robotics'
          WHEN 'baxter' THEN 'Rethink Robotics Baxter'
          ELSE INITCAP(REPLACE(robot_type, '_', ' '))
        END AS name,
        COUNT(m.id) AS module_count,
        COUNT(DISTINCT p.id) AS project_count,
        MIN(m.created_at) as first_used,
        MAX(m.updated_at) as last_used
      FROM (
        SELECT DISTINCT unnest(supported_robots) AS robot_type
        FROM modules 
        WHERE is_active = true 
        AND supported_robots IS NOT NULL 
        AND array_length(supported_robots, 1) > 0
      ) AS robot_types
      LEFT JOIN modules m ON robot_type = ANY(m.supported_robots) AND m.is_active = true
      LEFT JOIN project_module_dependencies pmd ON m.id = pmd.module_id
      LEFT JOIN projects p ON pmd.project_id = p.id AND p.is_active = true
      GROUP BY robot_type
      ORDER BY module_count DESC, name ASC
    `);

      res.json({ success: true, data: result.rows });
    } catch (error) {
      next(error);
    }
  }
);

router.post(
  '/robots',
  requirePermission(Permission.ADMIN_ALL),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const { code, name, description } = req.body;

      if (!code || !name) {
        return res
          .status(400)
          .json({ success: false, error: 'Code and name are required' });
      }

      // Check if robot already exists
      const existing = await Database.query(
        `SELECT robot_type FROM (
        SELECT DISTINCT unnest(supported_robots) AS robot_type
        FROM modules 
        WHERE is_active = true 
        AND supported_robots IS NOT NULL 
        AND array_length(supported_robots, 1) > 0
      ) AS robot_types WHERE robot_type = $1`,
        [code]
      );

      if (existing.rows.length > 0) {
        return res
          .status(400)
          .json({ success: false, error: 'Robot already exists' });
      }

      // Add robot to a default module or create a placeholder
      await Database.query(
        `UPDATE modules SET supported_robots = array_append(supported_robots, $1), updated_at = NOW() 
       WHERE name = 'localization' AND is_active = true`,
        [code]
      );

      res.json({ success: true, data: { code, name, description } });
    } catch (error) {
      next(error);
    }
  }
);

router.delete(
  '/robots/:code',
  requirePermission(Permission.ADMIN_ALL),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const { code } = req.params;

      // Remove robot from all modules
      await Database.query(
        `UPDATE modules SET supported_robots = array_remove(supported_robots, $1), updated_at = NOW() 
       WHERE $1 = ANY(supported_robots)`,
        [code]
      );

      res.json({ success: true, message: 'Robot deleted successfully' });
    } catch (error) {
      next(error);
    }
  }
);

router.put(
  '/robots/:code',
  requirePermission(Permission.ADMIN_ALL),
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      const { code } = req.params;
      const { name, description } = req.body;

      if (!name) {
        return res
          .status(400)
          .json({ success: false, error: 'Name is required' });
      }

      // Note: Since robot names are hardcoded in the CASE statement,
      // we can only update the description or add it to a metadata table
      // For now, we'll just return success as the name mapping is in the query
      res.json({ success: true, data: { code, name, description } });
    } catch (error) {
      next(error);
    }
  }
);

export default router;
