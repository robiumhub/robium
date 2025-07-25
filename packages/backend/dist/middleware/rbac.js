"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.addUserPermissions = exports.logPermissionChecks = exports.requireProjectAccess = exports.requireUserManagement = exports.requireSystemAdmin = exports.requireAdmin = exports.requireProjectMemberManagement = exports.requireResourceOwnership = exports.requireAllPermissions = exports.requireAnyPermission = exports.requirePermission = void 0;
const errors_1 = require("../utils/errors");
const permissions_1 = require("../utils/permissions");
// Middleware to require specific permission
const requirePermission = (permission) => {
    return (req, res, next) => {
        if (!req.user) {
            next(new errors_1.UnauthorizedError('Authentication required'));
            return;
        }
        if (!permissions_1.PermissionUtils.hasPermission(req.user.role, permission)) {
            const description = permissions_1.PermissionUtils.getPermissionDescription(permission);
            next(new errors_1.ForbiddenError(`Insufficient permissions: ${description}`));
            return;
        }
        // Log permission check for auditing
        console.log(`Permission check passed: ${req.user.userId} has ${permission} for ${req.originalUrl}`);
        next();
    };
};
exports.requirePermission = requirePermission;
// Middleware to require any of the specified permissions
const requireAnyPermission = (permissions) => {
    return (req, res, next) => {
        if (!req.user) {
            next(new errors_1.UnauthorizedError('Authentication required'));
            return;
        }
        if (!permissions_1.PermissionUtils.hasAnyPermission(req.user.role, permissions)) {
            const descriptions = permissions.map((p) => permissions_1.PermissionUtils.getPermissionDescription(p));
            next(new errors_1.ForbiddenError(`Insufficient permissions. Required: ${descriptions.join(' OR ')}`));
            return;
        }
        // Log permission check for auditing
        console.log(`Permission check passed: ${req.user.userId} has any of ${permissions.join(', ')} for ${req.originalUrl}`);
        next();
    };
};
exports.requireAnyPermission = requireAnyPermission;
// Middleware to require all of the specified permissions
const requireAllPermissions = (permissions) => {
    return (req, res, next) => {
        if (!req.user) {
            next(new errors_1.UnauthorizedError('Authentication required'));
            return;
        }
        if (!permissions_1.PermissionUtils.hasAllPermissions(req.user.role, permissions)) {
            const descriptions = permissions.map((p) => permissions_1.PermissionUtils.getPermissionDescription(p));
            next(new errors_1.ForbiddenError(`Insufficient permissions. Required: ${descriptions.join(' AND ')}`));
            return;
        }
        // Log permission check for auditing
        console.log(`Permission check passed: ${req.user.userId} has all of ${permissions.join(', ')} for ${req.originalUrl}`);
        next();
    };
};
exports.requireAllPermissions = requireAllPermissions;
// Middleware to require resource ownership or admin access
const requireResourceOwnership = (resourceType, getResourceOwnerId) => {
    return async (req, res, next) => {
        if (!req.user) {
            next(new errors_1.UnauthorizedError('Authentication required'));
            return;
        }
        try {
            const resourceOwnerId = await getResourceOwnerId(req);
            if (!permissions_1.PermissionUtils.canAccessResource(req.user.role, resourceType, resourceOwnerId, req.user.userId, permissions_1.Permission.PROFILE_READ // Default permission, can be overridden
            )) {
                next(new errors_1.ForbiddenError(`Access denied: insufficient permissions for ${resourceType}`));
                return;
            }
            // Log resource access for auditing
            console.log(`Resource access granted: ${req.user.userId} accessed ${resourceType} owned by ${resourceOwnerId}`);
            next();
        }
        catch (error) {
            next(new errors_1.ForbiddenError(`Failed to validate resource ownership`));
        }
    };
};
exports.requireResourceOwnership = requireResourceOwnership;
// Middleware for project member management
const requireProjectMemberManagement = (getProjectOwnerId, getIsProjectMember = () => false) => {
    return async (req, res, next) => {
        if (!req.user) {
            next(new errors_1.UnauthorizedError('Authentication required'));
            return;
        }
        try {
            const projectOwnerId = await getProjectOwnerId(req);
            const isProjectMember = await getIsProjectMember(req);
            if (!permissions_1.PermissionUtils.canManageProjectMembers(req.user.role, projectOwnerId, req.user.userId, isProjectMember)) {
                next(new errors_1.ForbiddenError('Access denied: insufficient permissions for project member management'));
                return;
            }
            // Log project member management for auditing
            console.log(`Project member management granted: ${req.user.userId} managing members for project owned by ${projectOwnerId}`);
            next();
        }
        catch (error) {
            next(new errors_1.ForbiddenError('Failed to validate project member management permissions'));
        }
    };
};
exports.requireProjectMemberManagement = requireProjectMemberManagement;
// Middleware for admin-only routes
const requireAdmin = (req, res, next) => {
    if (!req.user) {
        next(new errors_1.UnauthorizedError('Authentication required'));
        return;
    }
    if (!permissions_1.PermissionUtils.hasPermission(req.user.role, permissions_1.Permission.ADMIN_ALL)) {
        next(new errors_1.ForbiddenError('Access denied: admin privileges required'));
        return;
    }
    // Log admin access for auditing
    console.log(`Admin access granted: ${req.user.userId} accessed admin route ${req.originalUrl}`);
    next();
};
exports.requireAdmin = requireAdmin;
// Middleware for system administration
const requireSystemAdmin = (req, res, next) => {
    if (!req.user) {
        next(new errors_1.UnauthorizedError('Authentication required'));
        return;
    }
    if (!permissions_1.PermissionUtils.hasPermission(req.user.role, permissions_1.Permission.SYSTEM_ADMIN)) {
        next(new errors_1.ForbiddenError('Access denied: system administration privileges required'));
        return;
    }
    // Log system admin access for auditing
    console.log(`System admin access granted: ${req.user.userId} accessed system admin route ${req.originalUrl}`);
    next();
};
exports.requireSystemAdmin = requireSystemAdmin;
// Middleware for user management (admin or self-management)
const requireUserManagement = (getTargetUserId) => {
    return (req, res, next) => {
        if (!req.user) {
            next(new errors_1.UnauthorizedError('Authentication required'));
            return;
        }
        const targetUserId = getTargetUserId(req);
        // Admin can manage all users
        if (permissions_1.PermissionUtils.hasPermission(req.user.role, permissions_1.Permission.ADMIN_ALL)) {
            console.log(`User management granted (admin): ${req.user.userId} managing user ${targetUserId}`);
            next();
            return;
        }
        // Users can only manage their own profiles
        if (targetUserId === req.user.userId) {
            if (permissions_1.PermissionUtils.hasPermission(req.user.role, permissions_1.Permission.PROFILE_UPDATE)) {
                console.log(`User management granted (self): ${req.user.userId} managing own profile`);
                next();
                return;
            }
        }
        next(new errors_1.ForbiddenError('Access denied: insufficient permissions for user management'));
    };
};
exports.requireUserManagement = requireUserManagement;
// Middleware for project access (owner, member, or admin)
const requireProjectAccess = (getProjectId, getProjectOwnerId, getIsProjectMember = () => false) => {
    return async (req, res, next) => {
        if (!req.user) {
            next(new errors_1.UnauthorizedError('Authentication required'));
            return;
        }
        try {
            const projectId = getProjectId(req);
            const projectOwnerId = await getProjectOwnerId(req);
            const isProjectMember = await getIsProjectMember(req);
            // Admin can access all projects
            if (permissions_1.PermissionUtils.hasPermission(req.user.role, permissions_1.Permission.ADMIN_ALL)) {
                console.log(`Project access granted (admin): ${req.user.userId} accessing project ${projectId}`);
                next();
                return;
            }
            // Project owner can access their projects
            if (projectOwnerId === req.user.userId) {
                if (permissions_1.PermissionUtils.hasPermission(req.user.role, permissions_1.Permission.PROJECT_READ)) {
                    console.log(`Project access granted (owner): ${req.user.userId} accessing own project ${projectId}`);
                    next();
                    return;
                }
            }
            // Project members can access projects they're part of
            if (isProjectMember) {
                if (permissions_1.PermissionUtils.hasPermission(req.user.role, permissions_1.Permission.PROJECT_READ)) {
                    console.log(`Project access granted (member): ${req.user.userId} accessing project ${projectId} as member`);
                    next();
                    return;
                }
            }
            next(new errors_1.ForbiddenError('Access denied: insufficient permissions for project access'));
        }
        catch (error) {
            next(new errors_1.ForbiddenError('Failed to validate project access permissions'));
        }
    };
};
exports.requireProjectAccess = requireProjectAccess;
// Utility middleware to log all permission checks
const logPermissionChecks = (req, res, next) => {
    if (req.user) {
        console.log(`Permission check initiated: ${req.user.userId} (${req.user.role}) accessing ${req.method} ${req.originalUrl}`);
    }
    next();
};
exports.logPermissionChecks = logPermissionChecks;
// Middleware to add user permissions to request for debugging
const addUserPermissions = (req, res, next) => {
    if (req.user) {
        const permissions = permissions_1.PermissionUtils.getRolePermissions(req.user.role);
        req.userPermissions = permissions;
    }
    next();
};
exports.addUserPermissions = addUserPermissions;
