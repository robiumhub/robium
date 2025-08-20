import { Request, Response, NextFunction } from 'express';
import jwt from 'jsonwebtoken';

export interface AuthRequest extends Request {
  user?: {
    id: string;
    email: string;
    username: string;
    role: string;
  };
}

export const authMiddleware = (req: AuthRequest, res: Response, next: NextFunction): void => {
  try {
    const token = req.headers.authorization?.replace('Bearer ', '');

    if (!token) {
      res.status(401).json({
        success: false,
        error: 'No token provided',
      });
      return;
    }

    // Simple token-based authentication for testing
    if (token === 'admin-token') {
      req.user = {
        id: '1',
        email: 'admin@robium.com',
        username: 'admin',
        role: 'admin',
      };
      next();
    } else if (token === 'user-token') {
      req.user = {
        id: '2',
        email: 'user@robium.com',
        username: 'user',
        role: 'user',
      };
      next();
    } else if (token === 'dummy-token') {
      // Fallback for backward compatibility
      req.user = {
        id: '1',
        email: 'admin@robium.com',
        username: 'admin',
        role: 'admin',
      };
      next();
    } else {
      res.status(401).json({
        success: false,
        error: 'Invalid token',
      });
    }
  } catch (error) {
    res.status(401).json({
      success: false,
      error: 'Invalid token',
    });
  }
};

export const adminMiddleware = (req: AuthRequest, res: Response, next: NextFunction): void => {
  authMiddleware(req, res, () => {
    if (req.user?.role === 'admin') {
      next();
    } else {
      res.status(403).json({
        success: false,
        error: 'Admin access required',
      });
    }
  });
};
