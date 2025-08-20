import { UserRole } from '../types';

// Permission enum for granular access control
export enum Permission {
  // User management permissions
  USER_READ = 'user:read',
  USER_CREATE = 'user:create',
  USER_UPDATE = 'user:update',
  USER_DELETE = 'user:delete',

  // Project permissions
  PROJECT_READ = 'project:read',
  PROJECT_CREATE = 'project:create',
  PROJECT_UPDATE = 'project:update',
  PROJECT_DELETE = 'project:delete',
  PROJECT_MANAGE_MEMBERS = 'project:manage_members',

  // Session permissions
  SESSION_READ = 'session:read',
  SESSION_CREATE = 'session:create',
  SESSION_UPDATE = 'session:update',
  SESSION_DELETE = 'session:delete',

  // System permissions
  SYSTEM_READ = 'system:read',
  SYSTEM_WRITE = 'system:write',
  SYSTEM_ADMIN = 'system:admin',

  // Profile permissions (users can manage their own profiles)
  PROFILE_READ = 'profile:read',
  PROFILE_UPDATE = 'profile:update',
  PROFILE_DELETE = 'profile:delete',

  // Authentication permissions
  AUTH_LOGIN = 'auth:login',
  AUTH_LOGOUT = 'auth:logout',
  AUTH_REFRESH = 'auth:refresh',

  // Admin permissions (superuser capabilities)
  ADMIN_ALL = 'admin:all',
}

// Resource types for ownership validation
export enum ResourceType {
  USER = 'user',
  PROJECT = 'project',
  SESSION = 'session',
  PROFILE = 'profile',
}

// Role-permission mapping
export const ROLE_PERMISSIONS: Record<UserRole, Permission[]> = {
  [UserRole.ADMIN]: [
    // Admin has all permissions
    ...Object.values(Permission),
  ],

  [UserRole.USER]: [
    // User management - users can only read their own data
    Permission.USER_READ,

    // Project permissions - users can manage projects they own or are members of
    Permission.PROJECT_READ,
    Permission.PROJECT_CREATE,
    Permission.PROJECT_UPDATE,
    Permission.PROJECT_DELETE,
    Permission.PROJECT_MANAGE_MEMBERS,

    // Session permissions - users can manage their own sessions
    Permission.SESSION_READ,
    Permission.SESSION_CREATE,
    Permission.SESSION_UPDATE,
    Permission.SESSION_DELETE,

    // Profile permissions - users can manage their own profiles
    Permission.PROFILE_READ,
    Permission.PROFILE_UPDATE,
    Permission.PROFILE_DELETE,

    // Authentication permissions
    Permission.AUTH_LOGIN,
    Permission.AUTH_LOGOUT,
    Permission.AUTH_REFRESH,
  ],
};

// Permission checking utilities
export class PermissionUtils {
  // Check if a user has a specific permission
  static hasPermission(userRole: UserRole, permission: Permission): boolean {
    const userPermissions = ROLE_PERMISSIONS[userRole] || [];
    return (
      userPermissions.includes(permission) ||
      userPermissions.includes(Permission.ADMIN_ALL)
    );
  }

  // Check if a user has any of the specified permissions
  static hasAnyPermission(
    userRole: UserRole,
    permissions: Permission[]
  ): boolean {
    return permissions.some((permission) =>
      this.hasPermission(userRole, permission)
    );
  }

  // Check if a user has all of the specified permissions
  static hasAllPermissions(
    userRole: UserRole,
    permissions: Permission[]
  ): boolean {
    return permissions.every((permission) =>
      this.hasPermission(userRole, permission)
    );
  }

  // Get all permissions for a role
  static getRolePermissions(userRole: UserRole): Permission[] {
    return ROLE_PERMISSIONS[userRole] || [];
  }

  // Check if user can access a resource (ownership validation)
  static canAccessResource(
    userRole: UserRole,
    resourceType: ResourceType,
    resourceOwnerId: string,
    userId: string,
    permission: Permission
  ): boolean {
    // Admin can access all resources
    if (this.hasPermission(userRole, Permission.ADMIN_ALL)) {
      return true;
    }

    // Check if user has the required permission
    if (!this.hasPermission(userRole, permission)) {
      return false;
    }

    // For profile resources, users can only access their own
    if (resourceType === ResourceType.PROFILE) {
      return resourceOwnerId === userId;
    }

    // For other resources, check ownership or membership
    // This can be extended based on business logic
    return resourceOwnerId === userId;
  }

  // Check if user can manage project members
  static canManageProjectMembers(
    userRole: UserRole,
    projectOwnerId: string,
    userId: string,
    isProjectMember: boolean = false
  ): boolean {
    // Admin can manage all project members
    if (this.hasPermission(userRole, Permission.ADMIN_ALL)) {
      return true;
    }

    // Project owner can manage members
    if (projectOwnerId === userId) {
      return true;
    }

    // Project members with manage permission can manage other members
    if (
      isProjectMember &&
      this.hasPermission(userRole, Permission.PROJECT_MANAGE_MEMBERS)
    ) {
      return true;
    }

    return false;
  }

  // Validate permission hierarchy (admin > user)
  static validatePermissionHierarchy(
    requiredRole: UserRole,
    userRole: UserRole
  ): boolean {
    const roleHierarchy = {
      [UserRole.ADMIN]: 2,
      [UserRole.USER]: 1,
    };

    const requiredLevel = roleHierarchy[requiredRole] || 0;
    const userLevel = roleHierarchy[userRole] || 0;

    return userLevel >= requiredLevel;
  }

  // Get permission description for logging/auditing
  static getPermissionDescription(permission: Permission): string {
    const descriptions: Record<Permission, string> = {
      [Permission.USER_READ]: 'Read user information',
      [Permission.USER_CREATE]: 'Create new users',
      [Permission.USER_UPDATE]: 'Update user information',
      [Permission.USER_DELETE]: 'Delete users',
      [Permission.PROJECT_READ]: 'Read project information',
      [Permission.PROJECT_CREATE]: 'Create new projects',
      [Permission.PROJECT_UPDATE]: 'Update project information',
      [Permission.PROJECT_DELETE]: 'Delete projects',
      [Permission.PROJECT_MANAGE_MEMBERS]: 'Manage project members',
      [Permission.SESSION_READ]: 'Read session information',
      [Permission.SESSION_CREATE]: 'Create new sessions',
      [Permission.SESSION_UPDATE]: 'Update session information',
      [Permission.SESSION_DELETE]: 'Delete sessions',
      [Permission.SYSTEM_READ]: 'Read system information',
      [Permission.SYSTEM_WRITE]: 'Write system information',
      [Permission.SYSTEM_ADMIN]: 'Administer system',
      [Permission.PROFILE_READ]: 'Read profile information',
      [Permission.PROFILE_UPDATE]: 'Update profile information',
      [Permission.PROFILE_DELETE]: 'Delete profile',
      [Permission.AUTH_LOGIN]: 'User login',
      [Permission.AUTH_LOGOUT]: 'User logout',
      [Permission.AUTH_REFRESH]: 'Refresh authentication token',
      [Permission.ADMIN_ALL]: 'Full administrative access',
    };

    return descriptions[permission] || 'Unknown permission';
  }
}

// Permission groups for common operations
export const PermissionGroups = {
  USER_MANAGEMENT: [
    Permission.USER_READ,
    Permission.USER_CREATE,
    Permission.USER_UPDATE,
    Permission.USER_DELETE,
  ],

  PROJECT_MANAGEMENT: [
    Permission.PROJECT_READ,
    Permission.PROJECT_CREATE,
    Permission.PROJECT_UPDATE,
    Permission.PROJECT_DELETE,
    Permission.PROJECT_MANAGE_MEMBERS,
  ],

  SESSION_MANAGEMENT: [
    Permission.SESSION_READ,
    Permission.SESSION_CREATE,
    Permission.SESSION_UPDATE,
    Permission.SESSION_DELETE,
  ],

  PROFILE_MANAGEMENT: [
    Permission.PROFILE_READ,
    Permission.PROFILE_UPDATE,
    Permission.PROFILE_DELETE,
  ],

  AUTHENTICATION: [
    Permission.AUTH_LOGIN,
    Permission.AUTH_LOGOUT,
    Permission.AUTH_REFRESH,
  ],

  SYSTEM_ADMINISTRATION: [
    Permission.SYSTEM_READ,
    Permission.SYSTEM_WRITE,
    Permission.SYSTEM_ADMIN,
  ],
};
