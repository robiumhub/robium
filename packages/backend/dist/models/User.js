"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.UserModel = void 0;
const bcryptjs_1 = __importDefault(require("bcryptjs"));
const database_1 = require("../utils/database");
const types_1 = require("../types");
const validation_1 = require("../utils/validation");
const errors_1 = require("../utils/errors");
class UserModel {
    // Create a new user
    static async create(userData) {
        try {
            // Validate input data
            const validatedData = (0, validation_1.validateCreateUser)(userData);
            const { email, username, password, role = types_1.UserRole.USER } = validatedData;
            // Hash the password
            const passwordHash = await bcryptjs_1.default.hash(password, 12);
            // Insert user into database
            const query = `
        INSERT INTO users (email, username, password_hash, role) 
        VALUES ($1, $2, $3, $4) 
        RETURNING id, email, username, role, created_at, updated_at
      `;
            const result = (await database_1.Database.query(query, [
                email,
                username,
                passwordHash,
                role,
            ]));
            if (!result.rows || result.rows.length === 0) {
                throw new Error('Failed to create user');
            }
            return result.rows[0];
        }
        catch (error) {
            // Handle database constraint errors
            if (error && typeof error === 'object' && 'code' in error) {
                throw (0, errors_1.handleDatabaseError)(error);
            }
            // Re-throw validation errors and other operational errors
            throw error;
        }
    }
    // Find user by ID
    static async findById(id) {
        try {
            // Validate ID format
            (0, validation_1.validateUserId)({ id });
            const query = `
        SELECT id, email, username, role, created_at, updated_at 
        FROM users 
        WHERE id = $1
      `;
            const result = (await database_1.Database.query(query, [id]));
            return result.rows[0] || null;
        }
        catch (error) {
            if (error instanceof errors_1.ValidationError) {
                throw error;
            }
            throw (0, errors_1.handleDatabaseError)(error);
        }
    }
    // Find user by email (for authentication)
    static async findByEmail(email) {
        try {
            if (!email || typeof email !== 'string') {
                throw new errors_1.ValidationError('Valid email is required');
            }
            const query = `
        SELECT id, email, username, role, created_at, updated_at 
        FROM users 
        WHERE email = $1
      `;
            const result = (await database_1.Database.query(query, [email.toLowerCase()]));
            return result.rows[0] || null;
        }
        catch (error) {
            if (error instanceof errors_1.ValidationError) {
                throw error;
            }
            throw (0, errors_1.handleDatabaseError)(error);
        }
    }
    // Find user by email with password hash (for login)
    static async findByEmailWithPassword(email) {
        try {
            if (!email || typeof email !== 'string') {
                throw new errors_1.ValidationError('Valid email is required');
            }
            const query = `
        SELECT id, email, username, password_hash, role, created_at, updated_at 
        FROM users 
        WHERE email = $1
      `;
            const result = (await database_1.Database.query(query, [email.toLowerCase()]));
            return result.rows[0] || null;
        }
        catch (error) {
            if (error instanceof errors_1.ValidationError) {
                throw error;
            }
            throw (0, errors_1.handleDatabaseError)(error);
        }
    }
    // Update user
    static async update(id, updates) {
        try {
            // Validate ID and updates
            (0, validation_1.validateUserId)({ id });
            const validatedUpdates = (0, validation_1.validateUpdateUser)(updates);
            // Check if user exists
            const existingUser = await this.findById(id);
            if (!existingUser) {
                throw new errors_1.NotFoundError('User', id);
            }
            // Build dynamic update query
            const updateFields = [];
            const values = [];
            let paramIndex = 1;
            if (validatedUpdates.email) {
                updateFields.push(`email = $${paramIndex++}`);
                values.push(validatedUpdates.email.toLowerCase());
            }
            if (validatedUpdates.username) {
                updateFields.push(`username = $${paramIndex++}`);
                values.push(validatedUpdates.username);
            }
            if (validatedUpdates.role) {
                updateFields.push(`role = $${paramIndex++}`);
                values.push(validatedUpdates.role);
            }
            if (updateFields.length === 0) {
                throw new errors_1.ValidationError('No valid fields provided for update');
            }
            // Add updated_at timestamp
            updateFields.push(`updated_at = CURRENT_TIMESTAMP`);
            // Add WHERE clause parameter
            values.push(id);
            const query = `
        UPDATE users 
        SET ${updateFields.join(', ')} 
        WHERE id = $${paramIndex}
        RETURNING id, email, username, role, created_at, updated_at
      `;
            const result = (await database_1.Database.query(query, values));
            if (!result.rows || result.rows.length === 0) {
                throw new errors_1.NotFoundError('User', id);
            }
            return result.rows[0];
        }
        catch (error) {
            // Handle database constraint errors
            if (error && typeof error === 'object' && 'code' in error) {
                throw (0, errors_1.handleDatabaseError)(error);
            }
            // Re-throw validation and other operational errors
            throw error;
        }
    }
    // Change user password
    static async changePassword(id, currentPassword, newPassword) {
        try {
            // Validate ID
            (0, validation_1.validateUserId)({ id });
            // Get user with current password hash
            const query = `
        SELECT id, password_hash 
        FROM users 
        WHERE id = $1
      `;
            const result = (await database_1.Database.query(query, [id]));
            if (!result.rows || result.rows.length === 0) {
                throw new errors_1.NotFoundError('User', id);
            }
            const user = result.rows[0];
            // Verify current password
            const isCurrentPasswordValid = await bcryptjs_1.default.compare(currentPassword, user.password_hash);
            if (!isCurrentPasswordValid) {
                throw new errors_1.ValidationError('Current password is incorrect');
            }
            // Hash new password
            const newPasswordHash = await bcryptjs_1.default.hash(newPassword, 12);
            // Update password
            const updateQuery = `
        UPDATE users 
        SET password_hash = $1, updated_at = CURRENT_TIMESTAMP 
        WHERE id = $2
      `;
            await database_1.Database.query(updateQuery, [newPasswordHash, id]);
        }
        catch (error) {
            if (error instanceof errors_1.ValidationError || error instanceof errors_1.NotFoundError) {
                throw error;
            }
            throw (0, errors_1.handleDatabaseError)(error);
        }
    }
    // Delete user (soft delete - you could implement hard delete if needed)
    static async delete(id) {
        try {
            // Validate ID
            (0, validation_1.validateUserId)({ id });
            // Check if user exists
            const existingUser = await this.findById(id);
            if (!existingUser) {
                throw new errors_1.NotFoundError('User', id);
            }
            // For now, we'll do a hard delete. In production, you might want soft delete
            const query = `DELETE FROM users WHERE id = $1`;
            await database_1.Database.query(query, [id]);
        }
        catch (error) {
            if (error instanceof errors_1.ValidationError || error instanceof errors_1.NotFoundError) {
                throw error;
            }
            throw (0, errors_1.handleDatabaseError)(error);
        }
    }
    // Validate password against hash
    static async validatePassword(password, hash) {
        try {
            return await bcryptjs_1.default.compare(password, hash);
        }
        catch (error) {
            return false;
        }
    }
    // Get all users (with pagination)
    static async findAll(page = 1, limit = 10) {
        try {
            // Validate pagination parameters
            const validPage = Math.max(1, Math.floor(page));
            const validLimit = Math.max(1, Math.min(100, Math.floor(limit))); // Max 100 per page
            const offset = (validPage - 1) * validLimit;
            // Get total count
            const countQuery = `SELECT COUNT(*) as total FROM users`;
            const countResult = (await database_1.Database.query(countQuery));
            const total = parseInt(countResult.rows[0].total);
            // Get users with pagination
            const query = `
        SELECT id, email, username, role, created_at, updated_at 
        FROM users 
        ORDER BY created_at DESC 
        LIMIT $1 OFFSET $2
      `;
            const result = (await database_1.Database.query(query, [validLimit, offset]));
            return {
                users: result.rows,
                pagination: {
                    page: validPage,
                    limit: validLimit,
                    total,
                    totalPages: Math.ceil(total / validLimit),
                },
            };
        }
        catch (error) {
            throw (0, errors_1.handleDatabaseError)(error);
        }
    }
    // Check if user exists by email
    static async existsByEmail(email) {
        try {
            const query = `SELECT 1 FROM users WHERE email = $1 LIMIT 1`;
            const result = (await database_1.Database.query(query, [email.toLowerCase()]));
            return result.rows.length > 0;
        }
        catch (error) {
            throw (0, errors_1.handleDatabaseError)(error);
        }
    }
    // Check if user exists by username
    static async existsByUsername(username) {
        try {
            const query = `SELECT 1 FROM users WHERE username = $1 LIMIT 1`;
            const result = (await database_1.Database.query(query, [username]));
            return result.rows.length > 0;
        }
        catch (error) {
            throw (0, errors_1.handleDatabaseError)(error);
        }
    }
    // Get user count by role
    static async getCountByRole() {
        try {
            const query = `
        SELECT role, COUNT(*) as count 
        FROM users 
        GROUP BY role
      `;
            const result = (await database_1.Database.query(query));
            // Initialize counts for all roles
            const counts = {
                [types_1.UserRole.ADMIN]: 0,
                [types_1.UserRole.USER]: 0,
            };
            // Fill in actual counts
            result.rows.forEach((row) => {
                counts[row.role] = parseInt(row.count);
            });
            return counts;
        }
        catch (error) {
            throw (0, errors_1.handleDatabaseError)(error);
        }
    }
}
exports.UserModel = UserModel;
