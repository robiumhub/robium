import { Router } from 'express';
import { authMiddleware, adminMiddleware, AuthRequest } from '../middleware/auth';
import bcrypt from 'bcryptjs';
import { Database } from '../utils/database';

const router = Router();

// GET /api/users - Get all users (admin only)
router.get('/', adminMiddleware, async (req, res) => {
  try {
    const db = Database.getDatabase();

    const users = await new Promise<any[]>((resolve, reject) => {
      db.all(
        'SELECT id, email, username, role, is_active, created_at, updated_at FROM users ORDER BY created_at DESC',
        (err, rows) => {
          if (err) reject(err);
          else resolve(rows || []);
        }
      );
    });

    res.json({
      success: true,
      data: {
        users: users.map((user) => ({
          ...user,
          createdAt: new Date(user.created_at),
          updatedAt: new Date(user.updated_at),
          isActive: Boolean(user.is_active),
        })),
      },
    });
  } catch (error) {
    console.error('Error fetching users:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch users',
    });
  }
});

// GET /api/users/:id - Get user by ID (admin only)
router.get('/:id', adminMiddleware, async (req, res) => {
  try {
    const { id } = req.params;
    const db = Database.getDatabase();

    const users = await new Promise<any[]>((resolve, reject) => {
      db.all(
        'SELECT id, email, username, role, is_active, created_at, updated_at FROM users WHERE id = ?',
        [id],
        (err, rows) => {
          if (err) reject(err);
          else resolve(rows || []);
        }
      );
    });

    if (users.length === 0) {
      return res.status(404).json({
        success: false,
        error: 'User not found',
      });
    }

    const user = users[0];
    res.json({
      success: true,
      data: {
        ...user,
        createdAt: new Date(user.created_at),
        updatedAt: new Date(user.updated_at),
        isActive: Boolean(user.is_active),
      },
    });
  } catch (error) {
    console.error('Error fetching user:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch user',
    });
  }
});

// POST /api/users - Create new user (admin only)
router.post('/', adminMiddleware, async (req, res) => {
  try {
    const { email, username, password, role = 'user' } = req.body;

    // Validation
    if (!email || !username || !password) {
      return res.status(400).json({
        success: false,
        error: 'Email, username, and password are required',
      });
    }

    if (!['user', 'admin'].includes(role)) {
      return res.status(400).json({
        success: false,
        error: 'Role must be either "user" or "admin"',
      });
    }

    // Check if user already exists
    const db = Database.getDatabase();
    const existingUsers = await new Promise<any[]>((resolve, reject) => {
      db.all(
        'SELECT id FROM users WHERE email = ? OR username = ?',
        [email, username],
        (err, rows) => {
          if (err) reject(err);
          else resolve(rows || []);
        }
      );
    });

    if (existingUsers.length > 0) {
      return res.status(409).json({
        success: false,
        error: 'User with this email or username already exists',
      });
    }

    // Hash password
    const passwordHash = await bcrypt.hash(password, 12);

    // Create user
    const userId = `user_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    const now = new Date().toISOString();

    await new Promise<void>((resolve, reject) => {
      db.run(
        `INSERT INTO users (id, email, username, password_hash, role, is_active, created_at, updated_at) 
         VALUES (?, ?, ?, ?, ?, ?, ?, ?)`,
        [userId, email, username, passwordHash, role, 1, now, now],
        function (err) {
          if (err) reject(err);
          else resolve();
        }
      );
    });

    // Get the created user
    const newUsers = await new Promise<any[]>((resolve, reject) => {
      db.all(
        'SELECT id, email, username, role, is_active, created_at, updated_at FROM users WHERE id = ?',
        [userId],
        (err, rows) => {
          if (err) reject(err);
          else resolve(rows || []);
        }
      );
    });

    const newUser = newUsers[0];
    res.status(201).json({
      success: true,
      data: {
        ...newUser,
        createdAt: new Date(newUser.created_at),
        updatedAt: new Date(newUser.updated_at),
        isActive: Boolean(newUser.is_active),
      },
      message: 'User created successfully',
    });
  } catch (error) {
    console.error('Error creating user:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to create user',
    });
  }
});

// PUT /api/users/:id - Update user (admin only)
router.put('/:id', adminMiddleware, async (req, res) => {
  try {
    const { id } = req.params;
    const { email, username, password, role, isActive } = req.body;

    // Check if user exists
    const db = Database.getDatabase();
    const existingUsers = await new Promise<any[]>((resolve, reject) => {
      db.all('SELECT id FROM users WHERE id = ?', [id], (err, rows) => {
        if (err) reject(err);
        else resolve(rows || []);
      });
    });

    if (existingUsers.length === 0) {
      return res.status(404).json({
        success: false,
        error: 'User not found',
      });
    }

    // Validation
    if (role && !['user', 'admin'].includes(role)) {
      return res.status(400).json({
        success: false,
        error: 'Role must be either "user" or "admin"',
      });
    }

    // Check for email/username conflicts
    if (email || username) {
      const conflictUsers = await new Promise<any[]>((resolve, reject) => {
        db.all(
          'SELECT id FROM users WHERE (email = ? OR username = ?) AND id != ?',
          [email, username, id],
          (err, rows) => {
            if (err) reject(err);
            else resolve(rows || []);
          }
        );
      });

      if (conflictUsers.length > 0) {
        return res.status(409).json({
          success: false,
          error: 'User with this email or username already exists',
        });
      }
    }

    // Build update query dynamically
    const updates: string[] = [];
    const values: any[] = [];

    if (email !== undefined) {
      updates.push('email = ?');
      values.push(email);
    }

    if (username !== undefined) {
      updates.push('username = ?');
      values.push(username);
    }

    if (password !== undefined) {
      const passwordHash = await bcrypt.hash(password, 12);
      updates.push('password_hash = ?');
      values.push(passwordHash);
    }

    if (role !== undefined) {
      updates.push('role = ?');
      values.push(role);
    }

    if (isActive !== undefined) {
      updates.push('is_active = ?');
      values.push(isActive ? 1 : 0);
    }

    if (updates.length === 0) {
      return res.status(400).json({
        success: false,
        error: 'No fields to update',
      });
    }

    updates.push('updated_at = ?');
    values.push(new Date().toISOString());
    values.push(id);

    await new Promise<void>((resolve, reject) => {
      db.run(`UPDATE users SET ${updates.join(', ')} WHERE id = ?`, values, function (err) {
        if (err) reject(err);
        else resolve();
      });
    });

    // Get the updated user
    const updatedUsers = await new Promise<any[]>((resolve, reject) => {
      db.all(
        'SELECT id, email, username, role, is_active, created_at, updated_at FROM users WHERE id = ?',
        [id],
        (err, rows) => {
          if (err) reject(err);
          else resolve(rows || []);
        }
      );
    });

    const updatedUser = updatedUsers[0];
    res.json({
      success: true,
      data: {
        ...updatedUser,
        createdAt: new Date(updatedUser.created_at),
        updatedAt: new Date(updatedUser.updated_at),
        isActive: Boolean(updatedUser.is_active),
      },
      message: 'User updated successfully',
    });
  } catch (error) {
    console.error('Error updating user:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to update user',
    });
  }
});

// DELETE /api/users/:id - Delete user (admin only)
router.delete('/:id', adminMiddleware, async (req, res) => {
  try {
    const { id } = req.params;

    // Check if user exists
    const db = Database.getDatabase();
    const existingUsers = await new Promise<any[]>((resolve, reject) => {
      db.all('SELECT id FROM users WHERE id = ?', [id], (err, rows) => {
        if (err) reject(err);
        else resolve(rows || []);
      });
    });

    if (existingUsers.length === 0) {
      return res.status(404).json({
        success: false,
        error: 'User not found',
      });
    }

    // Prevent deleting the current user
    if (id === (req as AuthRequest).user?.id) {
      return res.status(400).json({
        success: false,
        error: 'You cannot delete your own account',
      });
    }

    // Delete user
    await new Promise<void>((resolve, reject) => {
      db.run('DELETE FROM users WHERE id = ?', [id], function (err) {
        if (err) reject(err);
        else resolve();
      });
    });

    res.json({
      success: true,
      message: 'User deleted successfully',
    });
  } catch (error) {
    console.error('Error deleting user:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to delete user',
    });
  }
});

export default router;
