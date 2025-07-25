import { Router, Request, Response, NextFunction } from 'express';
import { AuthService } from '../services/AuthService';
import { authenticateToken } from '../middleware/auth';
import { 
  CreateUserInput, 
  LoginInput, 
  ApiResponse, 
  AuthRequest 
} from '../types';
import { 
  ValidationError, 
  UnauthorizedError, 
  AppError 
} from '../utils/errors';

const router = Router();

// POST /auth/signup - User registration
router.post('/signup', async (req: Request, res: Response, next: NextFunction): Promise<void> => {
  try {
    const userData: CreateUserInput = req.body;
    
    const result = await AuthService.register(userData);

    const response: ApiResponse<typeof result> = {
      success: true,
      message: 'User registered successfully',
      data: result
    };

    res.status(201).json(response);
  } catch (error) {
    next(error);
  }
});

// POST /auth/login - User authentication
router.post('/login', async (req: Request, res: Response, next: NextFunction): Promise<void> => {
  try {
    const loginData: LoginInput = req.body;
    
    const result = await AuthService.login(loginData);

    const response: ApiResponse<typeof result> = {
      success: true,
      message: 'Login successful',
      data: result
    };

    res.status(200).json(response);
  } catch (error) {
    next(error);
  }
});

// POST /auth/logout - User logout
router.post('/logout', authenticateToken, async (req: AuthRequest, res: Response, next: NextFunction): Promise<void> => {
  try {
    // Get token from Authorization header
    const authHeader = req.headers.authorization;
    const token = authHeader && authHeader.startsWith('Bearer ') ? authHeader.substring(7) : '';

    if (token) {
      await AuthService.logout(token);
    }

    const response: ApiResponse = {
      success: true,
      message: 'Logout successful'
    };

    res.status(200).json(response);
  } catch (error) {
    next(error);
  }
});

// GET /auth/me - Get current user profile
router.get('/me', authenticateToken, async (req: AuthRequest, res: Response, next: NextFunction): Promise<void> => {
  try {
    if (!req.user) {
      throw new UnauthorizedError('User not authenticated');
    }

    const user = await AuthService.getCurrentUser(req.user.userId);

    const response: ApiResponse<typeof user> = {
      success: true,
      message: 'User profile retrieved successfully',
      data: user
    };

    res.status(200).json(response);
  } catch (error) {
    next(error);
  }
});

// POST /auth/change-password - Change user password
router.post('/change-password', authenticateToken, async (req: AuthRequest, res: Response, next: NextFunction): Promise<void> => {
  try {
    if (!req.user) {
      throw new UnauthorizedError('User not authenticated');
    }

    const { currentPassword, newPassword } = req.body;

    if (!currentPassword || !newPassword) {
      throw new ValidationError('Current password and new password are required');
    }

    await AuthService.changePassword(req.user.userId, currentPassword, newPassword);

    const response: ApiResponse = {
      success: true,
      message: 'Password changed successfully'
    };

    res.status(200).json(response);
  } catch (error) {
    next(error);
  }
});

// POST /auth/refresh - Refresh access token
router.post('/refresh', async (req: Request, res: Response, next: NextFunction): Promise<void> => {
  try {
    const { refreshToken } = req.body;

    if (!refreshToken) {
      throw new ValidationError('Refresh token is required');
    }

    const result = await AuthService.refreshToken(refreshToken);

    const response: ApiResponse<typeof result> = {
      success: true,
      message: 'Token refreshed successfully',
      data: result
    };

    res.status(200).json(response);
  } catch (error) {
    next(error);
  }
});

// GET /auth/verify - Verify token validity
router.get('/verify', authenticateToken, async (req: AuthRequest, res: Response, next: NextFunction): Promise<void> => {
  try {
    if (!req.user) {
      throw new UnauthorizedError('User not authenticated');
    }

    const response: ApiResponse<{ valid: boolean; user: typeof req.user }> = {
      success: true,
      message: 'Token is valid',
      data: {
        valid: true,
        user: req.user
      }
    };

    res.status(200).json(response);
  } catch (error) {
    next(error);
  }
});

// Error handling middleware specific to auth routes
router.use((error: AppError, req: Request, res: Response, next: NextFunction): void => {
  // Log authentication errors for security monitoring
  if (error instanceof UnauthorizedError) {
    console.log('Authentication error:', {
      message: error.message,
      ip: req.ip,
      userAgent: req.get('User-Agent'),
      timestamp: new Date().toISOString(),
      endpoint: req.originalUrl
    });
  }

  // Pass error to global error handler
  next(error);
});

export default router; 