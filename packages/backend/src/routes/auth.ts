import { Router } from 'express';
import { authMiddleware, AuthRequest } from '../middleware/auth';
import { Database } from '../utils/database';
import bcrypt from 'bcryptjs';

const router = Router();

// POST /api/auth/signup
router.post('/signup', async (req, res) => {
  try {
    // TODO: Implement signup logic
    res.json({
      success: true,
      data: {
        user: { id: '1', email: req.body.email, username: req.body.username },
        token: 'dummy-token',
        expiresIn: '1h',
      },
    });
  } catch (error) {
    res.status(500).json({
      success: false,
      error: 'Signup failed',
    });
  }
});

// POST /api/auth/login
router.post('/login', async (req, res) => {
  try {
    const { email, password } = req.body;

    if (!email || !password) {
      return res.status(400).json({
        success: false,
        error: 'Email and password are required',
      });
    }

    const db = Database.getDatabase();
    const user = await new Promise<any>((resolve, reject) => {
      db.get(
        'SELECT id, email, username, password_hash, role, is_active FROM users WHERE email = ?',
        [email],
        (err, row) => {
          if (err) reject(err);
          else resolve(row);
        }
      );
    });

    if (!user) {
      return res.status(401).json({
        success: false,
        error: 'Invalid credentials',
      });
    }

    if (user.is_active !== 1) {
      return res.status(401).json({
        success: false,
        error: 'Account is deactivated',
      });
    }

    // Use bcrypt to compare passwords
    const isValidPassword = await bcrypt.compare(password, user.password_hash);

    if (!isValidPassword) {
      return res.status(401).json({
        success: false,
        error: 'Invalid credentials',
      });
    }

    // Generate a simple token (in production, use JWT)
    const token = `${user.role}-token-${Date.now()}`;

    res.json({
      success: true,
      data: {
        user: {
          id: user.id,
          email: user.email,
          username: user.username,
          role: user.role,
          isActive: user.is_active === 1,
        },
        token,
        expiresIn: '1h',
      },
    });
  } catch (error) {
    console.error('Login error:', error);
    res.status(500).json({
      success: false,
      error: 'Login failed',
    });
  }
});

// GET /api/auth/me
router.get('/me', authMiddleware, async (req: AuthRequest, res) => {
  try {
    const userId = req.user?.id;
    if (!userId) {
      return res.status(401).json({
        success: false,
        error: 'User not authenticated',
      });
    }

    const db = Database.getDatabase();
    const user = await new Promise<any>((resolve, reject) => {
      db.get(
        'SELECT id, email, username, role, is_active, created_at, updated_at FROM users WHERE id = ?',
        [userId],
        (err, row) => {
          if (err) reject(err);
          else resolve(row);
        }
      );
    });

    if (!user) {
      return res.status(404).json({
        success: false,
        error: 'User not found',
      });
    }

    res.json({
      success: true,
      data: {
        id: user.id,
        email: user.email,
        username: user.username,
        role: user.role,
        isActive: user.is_active === 1,
        createdAt: user.created_at,
        updatedAt: user.updated_at,
      },
    });
  } catch (error) {
    console.error('Error getting user info:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to get user info',
    });
  }
});

// POST /api/auth/change-password
router.post('/change-password', authMiddleware, async (req, res) => {
  try {
    // TODO: Implement change password logic
    res.json({
      success: true,
      message: 'Password changed successfully',
    });
  } catch (error) {
    res.status(500).json({
      success: false,
      error: 'Failed to change password',
    });
  }
});

// PUT /api/auth/profile
router.put('/profile', authMiddleware, async (req: AuthRequest, res) => {
  try {
    const { username, email } = req.body;
    const userId = req.user?.id;

    // TODO: Implement profile update logic with database
    // For now, return a mock updated user
    const updatedUser = {
      id: userId || '1',
      email: email || 'test@example.com',
      username: username || 'testuser',
      role: 'user' as const,
      createdAt: new Date(),
      updatedAt: new Date(),
    };

    res.json({
      success: true,
      data: updatedUser,
      message: 'Profile updated successfully',
    });
  } catch (error) {
    res.status(500).json({
      success: false,
      error: 'Failed to update profile',
    });
  }
});

// POST /api/auth/refresh
router.post('/refresh', async (req, res) => {
  try {
    // TODO: Implement refresh logic
    res.json({
      success: true,
      data: {
        token: 'new-dummy-token',
        expiresIn: '1h',
      },
    });
  } catch (error) {
    res.status(500).json({
      success: false,
      error: 'Failed to refresh token',
    });
  }
});

// POST /api/auth/logout
router.post('/logout', authMiddleware, async (req, res) => {
  try {
    // TODO: Implement logout logic
    res.json({
      success: true,
      message: 'Logged out successfully',
    });
  } catch (error) {
    res.status(500).json({
      success: false,
      error: 'Failed to logout',
    });
  }
});

export default router;
