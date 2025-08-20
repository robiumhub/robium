import bcrypt from 'bcryptjs';
import { v4 as uuidv4 } from 'uuid';
import { Database } from '../utils/database';
import { User, CreateUserInput, UserRole } from '../types';
import {
  validateCreateUser,
  validateUpdateUser,
  validateUserId,
} from '../utils/validation';
import {
  NotFoundError,
  ValidationError,
  handleDatabaseError,
} from '../utils/errors';

export class UserModel {
  // Create a new user
  static async create(userData: CreateUserInput): Promise<User> {
    try {
      // Validate input data
      const validatedData = validateCreateUser(userData);
      const { email, username, password, role = UserRole.USER } = validatedData;

      // Hash the password
      const passwordHash = await bcrypt.hash(password, 12);

      // Generate id explicitly to support SQLite (which lacks gen_random_uuid())
      const id = uuidv4();

      // Insert user into database
      const insertQuery = `
        INSERT INTO users (id, email, username, password_hash, role) 
        VALUES ($1, $2, $3, $4, $5)
      `;

      await Database.query(insertQuery, [id, email, username, passwordHash, role]);

      // Fetch created user
      const selectQuery = `
        SELECT id, email, username, role, created_at, updated_at 
        FROM users 
        WHERE id = $1
      `;
      const result = (await Database.query(selectQuery, [id])) as { rows: User[] };
      if (!result.rows || result.rows.length === 0) {
        throw new Error('Failed to create user');
      }
      return result.rows[0];
    } catch (error: unknown) {
      // Handle database constraint errors
      if (error && typeof error === 'object' && 'code' in error) {
        throw handleDatabaseError(error);
      }

      // Re-throw validation errors and other operational errors
      throw error;
    }
  }

  // Find user by ID
  static async findById(id: string): Promise<User | null> {
    try {
      // Validate ID format
      validateUserId({ id });

      const query = `
        SELECT id, email, username, role, created_at, updated_at 
        FROM users 
        WHERE id = $1
      `;

      const result = (await Database.query(query, [id])) as { rows: User[] };
      return result.rows[0] || null;
    } catch (error: unknown) {
      if (error instanceof ValidationError) {
        throw error;
      }
      throw handleDatabaseError(error);
    }
  }

  // Find user by email (for authentication)
  static async findByEmail(email: string): Promise<User | null> {
    try {
      if (!email || typeof email !== 'string') {
        throw new ValidationError('Valid email is required');
      }

      const query = `
        SELECT id, email, username, role, created_at, updated_at 
        FROM users 
        WHERE email = $1
      `;

      const result = (await Database.query(query, [email.toLowerCase()])) as {
        rows: User[];
      };
      return result.rows[0] || null;
    } catch (error: unknown) {
      if (error instanceof ValidationError) {
        throw error;
      }
      throw handleDatabaseError(error);
    }
  }

  // Find user by email with password hash (for login)
  static async findByEmailWithPassword(
    email: string
  ): Promise<(User & { password_hash: string }) | null> {
    try {
      if (!email || typeof email !== 'string') {
        throw new ValidationError('Valid email is required');
      }

      const query = `
        SELECT id, email, username, password_hash, role, created_at, updated_at 
        FROM users 
        WHERE email = $1
      `;

      const result = (await Database.query(query, [email.toLowerCase()])) as {
        rows: (User & { password_hash: string })[];
      };
      return result.rows[0] || null;
    } catch (error: unknown) {
      if (error instanceof ValidationError) {
        throw error;
      }
      throw handleDatabaseError(error);
    }
  }

  // Update user
  static async update(
    id: string,
    updates: Partial<CreateUserInput>
  ): Promise<User> {
    try {
      // Validate ID and updates
      validateUserId({ id });
      const validatedUpdates = validateUpdateUser(updates);

      // Check if user exists
      const existingUser = await this.findById(id);
      if (!existingUser) {
        throw new NotFoundError('User', id);
      }

      // Build dynamic update query
      const updateFields: string[] = [];
      const values: (string | UserRole)[] = [];
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
        throw new ValidationError('No valid fields provided for update');
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

      const result = (await Database.query(query, values)) as { rows: User[] };

      if (!result.rows || result.rows.length === 0) {
        throw new NotFoundError('User', id);
      }

      return result.rows[0];
    } catch (error: unknown) {
      // Handle database constraint errors
      if (error && typeof error === 'object' && 'code' in error) {
        throw handleDatabaseError(error);
      }

      // Re-throw validation and other operational errors
      throw error;
    }
  }

  // Change user password
  static async changePassword(
    id: string,
    currentPassword: string,
    newPassword: string
  ): Promise<void> {
    try {
      // Validate ID
      validateUserId({ id });

      // Get user with current password hash
      const query = `
        SELECT id, password_hash 
        FROM users 
        WHERE id = $1
      `;

      const result = (await Database.query(query, [id])) as {
        rows: { id: string; password_hash: string }[];
      };

      if (!result.rows || result.rows.length === 0) {
        throw new NotFoundError('User', id);
      }

      const user = result.rows[0];

      // Verify current password
      const isCurrentPasswordValid = await bcrypt.compare(
        currentPassword,
        user.password_hash
      );
      if (!isCurrentPasswordValid) {
        throw new ValidationError('Current password is incorrect');
      }

      // Hash new password
      const newPasswordHash = await bcrypt.hash(newPassword, 12);

      // Update password
      const updateQuery = `
        UPDATE users 
        SET password_hash = $1, updated_at = CURRENT_TIMESTAMP 
        WHERE id = $2
      `;

      await Database.query(updateQuery, [newPasswordHash, id]);
    } catch (error: unknown) {
      if (error instanceof ValidationError || error instanceof NotFoundError) {
        throw error;
      }
      throw handleDatabaseError(error);
    }
  }

  // Delete user (soft delete - you could implement hard delete if needed)
  static async delete(id: string): Promise<void> {
    try {
      // Validate ID
      validateUserId({ id });

      // Check if user exists
      const existingUser = await this.findById(id);
      if (!existingUser) {
        throw new NotFoundError('User', id);
      }

      // For now, we'll do a hard delete. In production, you might want soft delete
      const query = `DELETE FROM users WHERE id = $1`;
      await Database.query(query, [id]);
    } catch (error: unknown) {
      if (error instanceof ValidationError || error instanceof NotFoundError) {
        throw error;
      }
      throw handleDatabaseError(error);
    }
  }

  // Validate password against hash
  static async validatePassword(
    password: string,
    hash: string
  ): Promise<boolean> {
    try {
      return await bcrypt.compare(password, hash);
    } catch (error) {
      return false;
    }
  }

  // Get all users (with pagination)
  static async findAll(
    page: number = 1,
    limit: number = 10
  ): Promise<{
    users: User[];
    pagination: {
      page: number;
      limit: number;
      total: number;
      totalPages: number;
    };
  }> {
    try {
      // Validate pagination parameters
      const validPage = Math.max(1, Math.floor(page));
      const validLimit = Math.max(1, Math.min(100, Math.floor(limit))); // Max 100 per page
      const offset = (validPage - 1) * validLimit;

      // Get total count
      const countQuery = `SELECT COUNT(*) as total FROM users`;
      const countResult = (await Database.query(countQuery)) as {
        rows: { total: string }[];
      };
      const total = parseInt(countResult.rows[0].total);

      // Get users with pagination
      const query = `
        SELECT id, email, username, role, created_at, updated_at 
        FROM users 
        ORDER BY created_at DESC 
        LIMIT $1 OFFSET $2
      `;

      const result = (await Database.query(query, [validLimit, offset])) as {
        rows: User[];
      };

      return {
        users: result.rows,
        pagination: {
          page: validPage,
          limit: validLimit,
          total,
          totalPages: Math.ceil(total / validLimit),
        },
      };
    } catch (error: unknown) {
      throw handleDatabaseError(error);
    }
  }

  // Check if user exists by email
  static async existsByEmail(email: string): Promise<boolean> {
    try {
      const query = `SELECT 1 FROM users WHERE email = $1 LIMIT 1`;
      const result = (await Database.query(query, [email.toLowerCase()])) as {
        rows: Record<string, unknown>[];
      };
      return result.rows.length > 0;
    } catch (error: unknown) {
      throw handleDatabaseError(error);
    }
  }

  // Check if user exists by username
  static async existsByUsername(username: string): Promise<boolean> {
    try {
      const query = `SELECT 1 FROM users WHERE username = $1 LIMIT 1`;
      const result = (await Database.query(query, [username])) as {
        rows: Record<string, unknown>[];
      };
      return result.rows.length > 0;
    } catch (error: unknown) {
      throw handleDatabaseError(error);
    }
  }

  // Get user count by role
  static async getCountByRole(): Promise<Record<UserRole, number>> {
    try {
      const query = `
        SELECT role, COUNT(*) as count 
        FROM users 
        GROUP BY role
      `;

      const result = (await Database.query(query)) as {
        rows: { role: UserRole; count: string }[];
      };

      // Initialize counts for all roles
      const counts: Record<UserRole, number> = {
        [UserRole.ADMIN]: 0,
        [UserRole.USER]: 0,
      };

      // Fill in actual counts
      result.rows.forEach((row) => {
        counts[row.role] = parseInt(row.count);
      });

      return counts;
    } catch (error: unknown) {
      throw handleDatabaseError(error);
    }
  }
}
