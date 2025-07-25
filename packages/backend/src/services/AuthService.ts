import jwt from 'jsonwebtoken';
import { UserModel } from '../models/User';
import { 
  CreateUserInput, 
  LoginInput, 
  User, 
  JWTPayload
} from '../types';
import { 
  validateCreateUser, 
  validateLogin 
} from '../utils/validation';
import { 
  ValidationError, 
  UnauthorizedError, 
  ConflictError 
} from '../utils/errors';

export interface AuthResponse {
  user: User;
  token: string;
  refreshToken?: string;
}

export interface LoginResponse {
  user: User;
  token: string;
  expiresIn: string;
}

export class AuthService {
  // Generate JWT token for user
  static generateToken(user: User): string {
    const jwtSecret = process.env.JWT_SECRET;
    const jwtExpiresIn = process.env.JWT_EXPIRES_IN || '1h';

    if (!jwtSecret) {
      throw new Error('JWT_SECRET environment variable is not configured');
    }

    const payload: JWTPayload = {
      userId: user.id,
      email: user.email,
      role: user.role
    };

    return jwt.sign(payload, jwtSecret, { 
      expiresIn: jwtExpiresIn 
    });
  }

  // Generate refresh token (for future implementation)
  static generateRefreshToken(user: User): string {
    const jwtRefreshSecret = process.env.JWT_REFRESH_SECRET || process.env.JWT_SECRET;
    const jwtRefreshExpiresIn = process.env.JWT_REFRESH_EXPIRES_IN || '7d';

    if (!jwtRefreshSecret) {
      throw new Error('JWT_REFRESH_SECRET environment variable is not configured');
    }

    const payload: JWTPayload = {
      userId: user.id,
      email: user.email,
      role: user.role
    };

    return jwt.sign(payload, jwtRefreshSecret, { 
      expiresIn: jwtRefreshExpiresIn 
    });
  }

  // User registration
  static async register(userData: CreateUserInput): Promise<AuthResponse> {
    try {
      // Validate input data
      const validatedData = validateCreateUser(userData);

      // Check if user already exists
      const existingUserByEmail = await UserModel.existsByEmail(validatedData.email);
      if (existingUserByEmail) {
        throw new ConflictError('Email address is already registered');
      }

      const existingUserByUsername = await UserModel.existsByUsername(validatedData.username);
      if (existingUserByUsername) {
        throw new ConflictError('Username is already taken');
      }

      // Create new user
      const newUser = await UserModel.create(validatedData);

      // Generate tokens
      const token = this.generateToken(newUser);
      const refreshToken = this.generateRefreshToken(newUser);

      return {
        user: newUser,
        token,
        refreshToken
      };
    } catch (error) {
      // Re-throw known errors
      if (error instanceof ValidationError || error instanceof ConflictError) {
        throw error;
      }
      
      // Handle unexpected errors
      throw new Error('Registration failed');
    }
  }

  // User login
  static async login(loginData: LoginInput): Promise<LoginResponse> {
    try {
      // Validate input data
      const validatedData = validateLogin(loginData);
      const { email, password } = validatedData as LoginInput;

      // Find user by email with password hash
      const userWithPassword = await UserModel.findByEmailWithPassword(email);
      if (!userWithPassword) {
        throw new UnauthorizedError('Invalid email or password');
      }

      // Verify password
      const isPasswordValid = await UserModel.validatePassword(password, userWithPassword.password_hash);
      if (!isPasswordValid) {
        throw new UnauthorizedError('Invalid email or password');
      }

      // Remove password hash from user object
      const { password_hash, ...user } = userWithPassword;

      // Generate token
      const token = this.generateToken(user as User);

      return {
        user: user as User,
        token,
        expiresIn: process.env.JWT_EXPIRES_IN || '1h'
      };
    } catch (error) {
      // Re-throw known errors
      if (error instanceof ValidationError || error instanceof UnauthorizedError) {
        throw error;
      }
      
      // Handle unexpected errors
      throw new UnauthorizedError('Authentication failed');
    }
  }

  // Verify token and get user
  static async verifyToken(token: string): Promise<User> {
    try {
      const jwtSecret = process.env.JWT_SECRET;
      if (!jwtSecret) {
        throw new Error('JWT_SECRET environment variable is not configured');
      }

      // Verify JWT token
      const decoded = jwt.verify(token, jwtSecret) as JWTPayload;
      
      // Get user from database
      const user = await UserModel.findById(decoded.userId);
      if (!user) {
        throw new UnauthorizedError('User not found');
      }

      return user;
    } catch (error) {
      if (error instanceof jwt.JsonWebTokenError) {
        throw new UnauthorizedError('Invalid token');
      } else if (error instanceof jwt.TokenExpiredError) {
        throw new UnauthorizedError('Token has expired');
      } else if (error instanceof UnauthorizedError) {
        throw error;
      } else {
        throw new UnauthorizedError('Token verification failed');
      }
    }
  }

  // Get current user profile
  static async getCurrentUser(userId: string): Promise<User> {
    try {
      const user = await UserModel.findById(userId);
      if (!user) {
        throw new UnauthorizedError('User not found');
      }

      return user;
    } catch (error) {
      if (error instanceof UnauthorizedError) {
        throw error;
      }
      throw new UnauthorizedError('Failed to get user profile');
    }
  }

  // Change password
  static async changePassword(userId: string, currentPassword: string, newPassword: string): Promise<void> {
    try {
      await UserModel.changePassword(userId, currentPassword, newPassword);
    } catch (error) {
      if (error instanceof ValidationError || error instanceof UnauthorizedError) {
        throw error;
      }
      throw new Error('Password change failed');
    }
  }

  // Refresh token (for future implementation)
  static async refreshToken(refreshToken: string): Promise<{ token: string; expiresIn: string }> {
    try {
      const jwtRefreshSecret = process.env.JWT_REFRESH_SECRET || process.env.JWT_SECRET;
      if (!jwtRefreshSecret) {
        throw new Error('JWT_REFRESH_SECRET environment variable is not configured');
      }

      // Verify refresh token
      const decoded = jwt.verify(refreshToken, jwtRefreshSecret) as JWTPayload;
      
      // Get user from database
      const user = await UserModel.findById(decoded.userId);
      if (!user) {
        throw new UnauthorizedError('User not found');
      }

      // Generate new access token
      const newToken = this.generateToken(user);

      return {
        token: newToken,
        expiresIn: process.env.JWT_EXPIRES_IN || '1h'
      };
    } catch (error) {
      if (error instanceof jwt.JsonWebTokenError) {
        throw new UnauthorizedError('Invalid refresh token');
      } else if (error instanceof jwt.TokenExpiredError) {
        throw new UnauthorizedError('Refresh token has expired');
      } else if (error instanceof UnauthorizedError) {
        throw error;
      } else {
        throw new UnauthorizedError('Token refresh failed');
      }
    }
  }

  // Logout (for future session management)
  static async logout(token: string): Promise<void> {
    // For JWT tokens, logout is typically handled client-side by removing the token
    // In a more advanced implementation, you might maintain a blacklist of tokens
    // or use session-based authentication with server-side session management
    
    // For now, we'll just verify the token is valid
    try {
      await this.verifyToken(token);
      // Token is valid, logout is successful
      // In the future, add token to blacklist or invalidate session
    } catch (error) {
      // Token is already invalid, which is fine for logout
    }
  }
} 