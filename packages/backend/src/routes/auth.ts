import { Router, Request, Response, NextFunction } from 'express';
import { AuthService } from '../services/AuthService';
import { ApiResponse, AuthRequest } from '../types';
import {
  validateCreateUserRequest,
  validateLoginRequest,
  validateChangePasswordRequest,
  validateUserIdParam,
  sanitizeInput,
} from '../middleware/validation';

const router = Router();

// Apply sanitization to all routes
router.use(sanitizeInput);

// Register new user
router.post(
  '/register',
  validateCreateUserRequest,
  async (req: Request, res: Response, next: NextFunction) => {
    try {
      const result = await AuthService.register(req.body);

      const response: ApiResponse = {
        success: true,
        data: result,
        message: 'User registered successfully',
      };

      res.status(201).json(response);
    } catch (error) {
      next(error);
    }
  }
);

// Login user
router.post(
  '/login',
  validateLoginRequest,
  async (req: Request, res: Response, next: NextFunction) => {
    try {
      const result = await AuthService.login(req.body);

      const response: ApiResponse = {
        success: true,
        data: result,
        message: 'Login successful',
      };

      res.json(response);
    } catch (error) {
      next(error);
    }
  }
);

// Change password (requires authentication)
router.post(
  '/change-password',
  validateChangePasswordRequest,
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      if (!req.user) {
        return res.status(401).json({
          success: false,
          error: 'Authentication required',
        });
      }

      await AuthService.changePassword(
        req.user.userId,
        req.body.currentPassword,
        req.body.newPassword
      );

      const response: ApiResponse = {
        success: true,
        message: 'Password changed successfully',
      };

      res.json(response);
    } catch (error) {
      next(error);
    }
  }
);

// Get current user profile
router.get(
  '/profile',
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      if (!req.user) {
        return res.status(401).json({
          success: false,
          error: 'Authentication required',
        });
      }

      const user = await AuthService.getCurrentUser(req.user.userId);

      const response: ApiResponse = {
        success: true,
        data: user,
        message: 'Profile retrieved successfully',
      };

      res.json(response);
    } catch (error) {
      next(error);
    }
  }
);

// Get user by ID (admin only)
router.get(
  '/users/:id',
  validateUserIdParam,
  async (req: AuthRequest, res: Response, next: NextFunction) => {
    try {
      if (!req.user) {
        return res.status(401).json({
          success: false,
          error: 'Authentication required',
        });
      }

      const user = await AuthService.getCurrentUser(req.params.id);

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

export default router;
