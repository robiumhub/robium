import { Request, Response, NextFunction } from 'express';
import jwt from 'jsonwebtoken';
import { AuthRequest, JWTPayload } from '../types';
import { UnauthorizedError } from '../utils/errors';
import { UserModel } from '../models/User';

// Authentication middleware to verify JWT tokens
export const authenticateToken = async (req: AuthRequest, res: Response, next: NextFunction): Promise<void> => {
  try {
    // Get token from Authorization header
    const authHeader = req.headers.authorization;
    const token = authHeader && authHeader.startsWith('Bearer ') ? authHeader.substring(7) : null;

    if (!token) {
      throw new UnauthorizedError('Access token is required');
    }

    // Verify JWT token
    const jwtSecret = process.env.JWT_SECRET;
    if (!jwtSecret) {
      throw new Error('JWT_SECRET environment variable is not configured');
    }

    const decoded = jwt.verify(token, jwtSecret) as JWTPayload;
    
    // Verify user still exists (optional security check)
    const user = await UserModel.findById(decoded.userId);
    if (!user) {
      throw new UnauthorizedError('User not found');
    }

    // Add user info to request object
    req.user = {
      userId: decoded.userId,
      email: decoded.email,
      role: decoded.role,
      iat: decoded.iat,
      exp: decoded.exp
    };

    next();
  } catch (error) {
    if (error instanceof jwt.JsonWebTokenError) {
      next(new UnauthorizedError('Invalid token'));
    } else if (error instanceof jwt.TokenExpiredError) {
      next(new UnauthorizedError('Token has expired'));
    } else if (error instanceof UnauthorizedError) {
      next(error);
    } else {
      next(new UnauthorizedError('Authentication failed'));
    }
  }
};

// Optional middleware for routes that work with or without authentication
export const optionalAuth = async (req: AuthRequest, res: Response, next: NextFunction): Promise<void> => {
  try {
    const authHeader = req.headers.authorization;
    const token = authHeader && authHeader.startsWith('Bearer ') ? authHeader.substring(7) : null;

    if (token) {
      const jwtSecret = process.env.JWT_SECRET;
      if (jwtSecret) {
        try {
          const decoded = jwt.verify(token, jwtSecret) as JWTPayload;
          
          // Verify user still exists
          const user = await UserModel.findById(decoded.userId);
          if (user) {
            req.user = {
              userId: decoded.userId,
              email: decoded.email,
              role: decoded.role,
              iat: decoded.iat,
              exp: decoded.exp
            };
          }
        } catch (error) {
          // Silently ignore token errors for optional auth
          console.log('Optional auth token error:', error);
        }
      }
    }

    next();
  } catch (error) {
    // Continue without authentication for optional auth
    next();
  }
};

// Middleware to check if user has specific role
export const requireRole = (requiredRole: string) => {
  return (req: AuthRequest, res: Response, next: NextFunction): void => {
    if (!req.user) {
      next(new UnauthorizedError('Authentication required'));
      return;
    }

    if (req.user.role !== requiredRole) {
      next(new UnauthorizedError(`${requiredRole} role required`));
      return;
    }

    next();
  };
};

// Middleware to check if user has admin role
export const requireAdmin = requireRole('admin');

// Middleware to check if user owns the resource or is admin
export const requireOwnershipOrAdmin = (getUserIdFromParams: (req: Request) => string) => {
  return (req: AuthRequest, res: Response, next: NextFunction): void => {
    if (!req.user) {
      next(new UnauthorizedError('Authentication required'));
      return;
    }

    const resourceUserId = getUserIdFromParams(req);
    const isOwner = req.user.userId === resourceUserId;
    const isAdmin = req.user.role === 'admin';

    if (!isOwner && !isAdmin) {
      next(new UnauthorizedError('Access denied: insufficient permissions'));
      return;
    }

    next();
  };
}; 