"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.PermissionGroups = exports.PermissionUtils = exports.ROLE_PERMISSIONS = exports.ResourceType = exports.Permission = void 0;
const types_1 = require("../types");
// Permission enum for granular access control
var Permission;
(function (Permission) {
    // User management permissions
    Permission["USER_READ"] = "user:read";
    Permission["USER_CREATE"] = "user:create";
    Permission["USER_UPDATE"] = "user:update";
    Permission["USER_DELETE"] = "user:delete";
    // Project permissions
    Permission["PROJECT_READ"] = "project:read";
    Permission["PROJECT_CREATE"] = "project:create";
    Permission["PROJECT_UPDATE"] = "project:update";
    Permission["PROJECT_DELETE"] = "project:delete";
    Permission["PROJECT_MANAGE_MEMBERS"] = "project:manage_members";
    // Session permissions
    Permission["SESSION_READ"] = "session:read";
    Permission["SESSION_CREATE"] = "session:create";
    Permission["SESSION_UPDATE"] = "session:update";
    Permission["SESSION_DELETE"] = "session:delete";
    // System permissions
    Permission["SYSTEM_READ"] = "system:read";
    Permission["SYSTEM_WRITE"] = "system:write";
    Permission["SYSTEM_ADMIN"] = "system:admin";
    // Profile permissions (users can manage their own profiles)
    Permission["PROFILE_READ"] = "profile:read";
    Permission["PROFILE_UPDATE"] = "profile:update";
    Permission["PROFILE_DELETE"] = "profile:delete";
    // Authentication permissions
    Permission["AUTH_LOGIN"] = "auth:login";
    Permission["AUTH_LOGOUT"] = "auth:logout";
    Permission["AUTH_REFRESH"] = "auth:refresh";
    // Admin permissions (superuser capabilities)
    Permission["ADMIN_ALL"] = "admin:all";
})(Permission = exports.Permission || (exports.Permission = {}));
// Resource types for ownership validation
var ResourceType;
(function (ResourceType) {
    ResourceType["USER"] = "user";
    ResourceType["PROJECT"] = "project";
    ResourceType["SESSION"] = "session";
    ResourceType["PROFILE"] = "profile";
})(ResourceType = exports.ResourceType || (exports.ResourceType = {}));
// Role-permission mapping
exports.ROLE_PERMISSIONS = {
    [types_1.UserRole.ADMIN]: [
        // Admin has all permissions
        ...Object.values(Permission),
    ],
    [types_1.UserRole.USER]: [
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
class PermissionUtils {
    // Check if a user has a specific permission
    static hasPermission(userRole, permission) {
        const userPermissions = exports.ROLE_PERMISSIONS[userRole] || [];
        return (userPermissions.includes(permission) ||
            userPermissions.includes(Permission.ADMIN_ALL));
    }
    // Check if a user has any of the specified permissions
    static hasAnyPermission(userRole, permissions) {
        return permissions.some((permission) => this.hasPermission(userRole, permission));
    }
    // Check if a user has all of the specified permissions
    static hasAllPermissions(userRole, permissions) {
        return permissions.every((permission) => this.hasPermission(userRole, permission));
    }
    // Get all permissions for a role
    static getRolePermissions(userRole) {
        return exports.ROLE_PERMISSIONS[userRole] || [];
    }
    // Check if user can access a resource (ownership validation)
    static canAccessResource(userRole, resourceType, resourceOwnerId, userId, permission) {
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
    static canManageProjectMembers(userRole, projectOwnerId, userId, isProjectMember = false) {
        // Admin can manage all project members
        if (this.hasPermission(userRole, Permission.ADMIN_ALL)) {
            return true;
        }
        // Project owner can manage members
        if (projectOwnerId === userId) {
            return true;
        }
        // Project members with manage permission can manage other members
        if (isProjectMember &&
            this.hasPermission(userRole, Permission.PROJECT_MANAGE_MEMBERS)) {
            return true;
        }
        return false;
    }
    // Validate permission hierarchy (admin > user)
    static validatePermissionHierarchy(requiredRole, userRole) {
        const roleHierarchy = {
            [types_1.UserRole.ADMIN]: 2,
            [types_1.UserRole.USER]: 1,
        };
        const requiredLevel = roleHierarchy[requiredRole] || 0;
        const userLevel = roleHierarchy[userRole] || 0;
        return userLevel >= requiredLevel;
    }
    // Get permission description for logging/auditing
    static getPermissionDescription(permission) {
        const descriptions = {
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
exports.PermissionUtils = PermissionUtils;
// Permission groups for common operations
exports.PermissionGroups = {
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
