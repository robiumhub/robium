import { Request } from 'express';

// User related types
export interface User {
  id: string;
  email: string;
  username: string;
  password?: string; // Optional for responses
  role: UserRole;
  createdAt: Date;
  updatedAt: Date;
}

export enum UserRole {
  ADMIN = 'admin',
  USER = 'user'
}

export interface CreateUserInput {
  email: string;
  username: string;
  password: string;
  role?: UserRole;
}

export interface LoginInput {
  email: string;
  password: string;
}

// Project related types
export interface Project {
  id: string;
  name: string;
  description?: string;
  ownerId: string;
  createdAt: Date;
  updatedAt: Date;
}

export interface CreateProjectInput {
  name: string;
  description?: string;
}

// JWT and Auth types
export interface JWTPayload {
  userId: string;
  email: string;
  role: UserRole;
  iat?: number;
  exp?: number;
}

export interface AuthRequest extends Request {
  user?: JWTPayload;
}

// API Response types
export interface ApiResponse<T = unknown> {
  success: boolean;
  message?: string;
  data?: T;
  error?: string;
}

export interface PaginatedResponse<T> extends ApiResponse<T[]> {
  pagination: {
    page: number;
    limit: number;
    total: number;
    totalPages: number;
  };
}

// Database types
export interface DatabaseConfig {
  host: string;
  port: number;
  database: string;
  username: string;
  password: string;
}

// WebSocket types
export interface WebSocketMessage {
  type: string;
  payload: unknown;
  userId?: string;
  timestamp: Date;
}

// Validation error type
export interface ValidationError {
  field: string;
  message: string;
}

export interface ApiError extends Error {
  status: number;
  errors?: ValidationError[];
} 