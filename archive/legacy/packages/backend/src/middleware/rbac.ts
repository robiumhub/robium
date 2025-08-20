import { Request, Response, NextFunction } from 'express';
import { AuthRequest } from '../types';
import { UnauthorizedError, ForbiddenError } from '../utils/errors';
import {
  Permission,
  ResourceType,
  PermissionUtils,
} from '../utils/permissions';

// Middleware to require specific permission
export const requirePermission = (permission: Permission) => {
  return (req: AuthRequest, res: Response, next: NextFunction): void => {
    if (!req.user) {
      next(new UnauthorizedError('Authentication required'));
      return;
    }

    if (!PermissionUtils.hasPermission(req.user.role, permission)) {
      const description = PermissionUtils.getPermissionDescription(permission);
      next(new ForbiddenError(`Insufficient permissions: ${description}`));
      return;
    }

    // Log permission check for auditing
    console.log(
      `Permission check passed: ${req.user.userId} has ${permission} for ${req.originalUrl}`
    );

    next();
  };
};

// Middleware to require any of the specified permissions
export const requireAnyPermission = (permissions: Permission[]) => {
  return (req: AuthRequest, res: Response, next: NextFunction): void => {
    if (!req.user) {
      next(new UnauthorizedError('Authentication required'));
      return;
    }

    if (!PermissionUtils.hasAnyPermission(req.user.role, permissions)) {
      const descriptions = permissions.map((p) =>
        PermissionUtils.getPermissionDescription(p)
      );
      next(
        new ForbiddenError(
          `Insufficient permissions. Required: ${descriptions.join(' OR ')}`
        )
      );
      return;
    }

    // Log permission check for auditing
    console.log(
      `Permission check passed: ${req.user.userId} has any of ${permissions.join(', ')} for ${req.originalUrl}`
    );

    next();
  };
};

// Middleware to require all of the specified permissions
export const requireAllPermissions = (permissions: Permission[]) => {
  return (req: AuthRequest, res: Response, next: NextFunction): void => {
    if (!req.user) {
      next(new UnauthorizedError('Authentication required'));
      return;
    }

    if (!PermissionUtils.hasAllPermissions(req.user.role, permissions)) {
      const descriptions = permissions.map((p) =>
        PermissionUtils.getPermissionDescription(p)
      );
      next(
        new ForbiddenError(
          `Insufficient permissions. Required: ${descriptions.join(' AND ')}`
        )
      );
      return;
    }

    // Log permission check for auditing
    console.log(
      `Permission check passed: ${req.user.userId} has all of ${permissions.join(', ')} for ${req.originalUrl}`
    );

    next();
  };
};

// Middleware to require resource ownership or admin access
export const requireResourceOwnership = (
  resourceType: ResourceType,
  getResourceOwnerId: (req: Request) => string | Promise<string>
) => {
  return async (
    req: AuthRequest,
    res: Response,
    next: NextFunction
  ): Promise<void> => {
    if (!req.user) {
      next(new UnauthorizedError('Authentication required'));
      return;
    }

    try {
      const resourceOwnerId = await getResourceOwnerId(req);

      if (
        !PermissionUtils.canAccessResource(
          req.user.role,
          resourceType,
          resourceOwnerId,
          req.user.userId,
          Permission.PROFILE_READ // Default permission, can be overridden
        )
      ) {
        next(
          new ForbiddenError(
            `Access denied: insufficient permissions for ${resourceType}`
          )
        );
        return;
      }

      // Log resource access for auditing
      console.log(
        `Resource access granted: ${req.user.userId} accessed ${resourceType} owned by ${resourceOwnerId}`
      );

      next();
    } catch (error) {
      next(new ForbiddenError(`Failed to validate resource ownership`));
    }
  };
};

// Middleware for project member management
export const requireProjectMemberManagement = (
  getProjectOwnerId: (req: Request) => string | Promise<string>,
  getIsProjectMember: (req: Request) => boolean | Promise<boolean> = () => false
) => {
  return async (
    req: AuthRequest,
    res: Response,
    next: NextFunction
  ): Promise<void> => {
    if (!req.user) {
      next(new UnauthorizedError('Authentication required'));
      return;
    }

    try {
      const projectOwnerId = await getProjectOwnerId(req);
      const isProjectMember = await getIsProjectMember(req);

      if (
        !PermissionUtils.canManageProjectMembers(
          req.user.role,
          projectOwnerId,
          req.user.userId,
          isProjectMember
        )
      ) {
        next(
          new ForbiddenError(
            'Access denied: insufficient permissions for project member management'
          )
        );
        return;
      }

      // Log project member management for auditing
      console.log(
        `Project member management granted: ${req.user.userId} managing members for project owned by ${projectOwnerId}`
      );

      next();
    } catch (error) {
      next(
        new ForbiddenError(
          'Failed to validate project member management permissions'
        )
      );
    }
  };
};

// Middleware for admin-only routes
export const requireAdmin = (
  req: AuthRequest,
  res: Response,
  next: NextFunction
): void => {
  if (!req.user) {
    next(new UnauthorizedError('Authentication required'));
    return;
  }

  if (!PermissionUtils.hasPermission(req.user.role, Permission.ADMIN_ALL)) {
    next(new ForbiddenError('Access denied: admin privileges required'));
    return;
  }

  // Log admin access for auditing
  console.log(
    `Admin access granted: ${req.user.userId} accessed admin route ${req.originalUrl}`
  );

  next();
};

// Middleware for system administration
export const requireSystemAdmin = (
  req: AuthRequest,
  res: Response,
  next: NextFunction
): void => {
  if (!req.user) {
    next(new UnauthorizedError('Authentication required'));
    return;
  }

  if (!PermissionUtils.hasPermission(req.user.role, Permission.SYSTEM_ADMIN)) {
    next(
      new ForbiddenError(
        'Access denied: system administration privileges required'
      )
    );
    return;
  }

  // Log system admin access for auditing
  console.log(
    `System admin access granted: ${req.user.userId} accessed system admin route ${req.originalUrl}`
  );

  next();
};

// Middleware for user management (admin or self-management)
export const requireUserManagement = (
  getTargetUserId: (req: Request) => string
) => {
  return (req: AuthRequest, res: Response, next: NextFunction): void => {
    if (!req.user) {
      next(new UnauthorizedError('Authentication required'));
      return;
    }

    const targetUserId = getTargetUserId(req);

    // Admin can manage all users
    if (PermissionUtils.hasPermission(req.user.role, Permission.ADMIN_ALL)) {
      console.log(
        `User management granted (admin): ${req.user.userId} managing user ${targetUserId}`
      );
      next();
      return;
    }

    // Users can only manage their own profiles
    if (targetUserId === req.user.userId) {
      if (
        PermissionUtils.hasPermission(req.user.role, Permission.PROFILE_UPDATE)
      ) {
        console.log(
          `User management granted (self): ${req.user.userId} managing own profile`
        );
        next();
        return;
      }
    }

    next(
      new ForbiddenError(
        'Access denied: insufficient permissions for user management'
      )
    );
  };
};

// Middleware for project access (owner, member, or admin)
export const requireProjectAccess = (
  getProjectId: (req: Request) => string,
  getProjectOwnerId: (req: Request) => string | Promise<string>,
  getIsProjectMember: (req: Request) => boolean | Promise<boolean> = () => false
) => {
  return async (
    req: AuthRequest,
    res: Response,
    next: NextFunction
  ): Promise<void> => {
    if (!req.user) {
      next(new UnauthorizedError('Authentication required'));
      return;
    }

    try {
      const projectId = getProjectId(req);
      const projectOwnerId = await getProjectOwnerId(req);
      const isProjectMember = await getIsProjectMember(req);

      // Admin can access all projects
      if (PermissionUtils.hasPermission(req.user.role, Permission.ADMIN_ALL)) {
        console.log(
          `Project access granted (admin): ${req.user.userId} accessing project ${projectId}`
        );
        next();
        return;
      }

      // Project owner can access their projects
      if (projectOwnerId === req.user.userId) {
        if (
          PermissionUtils.hasPermission(req.user.role, Permission.PROJECT_READ)
        ) {
          console.log(
            `Project access granted (owner): ${req.user.userId} accessing own project ${projectId}`
          );
          next();
          return;
        }
      }

      // Project members can access projects they're part of
      if (isProjectMember) {
        if (
          PermissionUtils.hasPermission(req.user.role, Permission.PROJECT_READ)
        ) {
          console.log(
            `Project access granted (member): ${req.user.userId} accessing project ${projectId} as member`
          );
          next();
          return;
        }
      }

      next(
        new ForbiddenError(
          'Access denied: insufficient permissions for project access'
        )
      );
    } catch (error) {
      next(new ForbiddenError('Failed to validate project access permissions'));
    }
  };
};

// Utility middleware to log all permission checks
export const logPermissionChecks = (
  req: AuthRequest,
  res: Response,
  next: NextFunction
): void => {
  if (req.user) {
    console.log(
      `Permission check initiated: ${req.user.userId} (${req.user.role}) accessing ${req.method} ${req.originalUrl}`
    );
  }
  next();
};

// Middleware to add user permissions to request for debugging
export const addUserPermissions = (
  req: AuthRequest,
  res: Response,
  next: NextFunction
): void => {
  if (req.user) {
    const permissions = PermissionUtils.getRolePermissions(req.user.role);
    req.userPermissions = permissions;
  }
  next();
};

// Type augmentation for AuthRequest to include permissions
declare module '../types' {
  interface AuthRequest {
    userPermissions?: string[];
  }
}
