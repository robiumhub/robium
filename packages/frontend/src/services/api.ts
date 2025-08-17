import axios, { AxiosInstance, AxiosResponse } from 'axios';

// Types
export interface User {
  id: string;
  email: string;
  username: string;
  role: 'user' | 'admin';
}

export interface LoginRequest {
  email: string;
  password: string;
}

export interface RegisterRequest {
  username: string;
  email: string;
  password: string;
}

export interface AuthResponse {
  user: User;
  token: string;
  expiresIn: string;
}

export interface ApiResponse<T = any> {
  success: boolean;
  message?: string;
  data?: T;
  error?: string;
}

// API Configuration
// Use proxy in development, direct URL in production
const API_BASE_URL =
  process.env.NODE_ENV === 'development'
    ? '/api'
    : process.env.REACT_APP_API_URL || '/api';

// Create axios instance
const api: AxiosInstance = axios.create({
  baseURL: API_BASE_URL,
  headers: {
    'Content-Type': 'application/json',
  },
  timeout: 60000,
});

// Request interceptor to add auth token
api.interceptors.request.use(
  (config) => {
    const token = localStorage.getItem('token');
    if (token) {
      config.headers.Authorization = `Bearer ${token}`;
    }
    return config;
  },
  (error) => {
    return Promise.reject(error);
  }
);

// Response interceptor to handle auth errors
api.interceptors.response.use(
  (response: AxiosResponse) => response,
  (error) => {
    if (error.response?.status === 401) {
      // Token expired or invalid
      localStorage.removeItem('token');
      localStorage.removeItem('user');
      window.location.href = '/login';
    }
    return Promise.reject(error);
  }
);

// API Service Class
export class ApiService {
  // Helper method to handle API errors
  private static handleApiError(error: any): never {
    if (error.response) {
      // Server responded with error status
      const { data, status } = error.response;

      // Extract detailed error information
      let errorMessage = 'An error occurred';

      if (data) {
        // Handle validation errors with details
        if (
          data.error === 'Validation failed' &&
          data.details &&
          Array.isArray(data.details)
        ) {
          const validationErrors = data.details
            .map((detail: any) => `${detail.field}: ${detail.message}`)
            .join(', ');
          errorMessage = `Validation failed: ${validationErrors}`;
        }
        // Handle conflict errors
        else if (data.error === 'Conflict' && data.message) {
          errorMessage = data.message;
        }
        // Handle other errors with message
        else if (data.message) {
          errorMessage = data.message;
        }
        // Handle errors with just error field
        else if (data.error) {
          errorMessage = data.error;
        }
      }

      throw new Error(errorMessage);
    } else if (error.request) {
      // Network error
      throw new Error('Network error: Unable to connect to the server');
    } else {
      // Other error
      throw new Error(error.message || 'An unexpected error occurred');
    }
  }

  // Authentication endpoints
  static async login(credentials: LoginRequest): Promise<AuthResponse> {
    try {
      const response = await api.post<ApiResponse<AuthResponse>>(
        '/auth/login',
        credentials
      );

      if (response.data.success) {
        return response.data.data!;
      } else {
        throw new Error(response.data.message || 'Login failed');
      }
    } catch (error) {
      throw ApiService.handleApiError(error);
    }
  }

  static async register(userData: RegisterRequest): Promise<AuthResponse> {
    try {
      const response = await api.post<ApiResponse<AuthResponse>>(
        '/auth/signup',
        userData
      );

      if (response.data.success) {
        return response.data.data!;
      } else {
        throw new Error(response.data.message || 'Registration failed');
      }
    } catch (error) {
      throw ApiService.handleApiError(error);
    }
  }

  static async getCurrentUser(): Promise<User> {
    try {
      const response = await api.get<ApiResponse<User>>('/auth/me');

      if (response.data.success) {
        return response.data.data!;
      } else {
        throw new Error(response.data.message || 'Failed to get user data');
      }
    } catch (error: any) {
      // For getCurrentUser, handle network errors more gracefully
      if (error.response) {
        // Server responded with error status
        const { data, status } = error.response;
        if (status === 401) {
          throw new Error('Token expired or invalid');
        }
        throw new Error(
          data?.message || data?.error || 'Failed to get user data'
        );
      } else if (error.request) {
        // Network error - don't throw, just return null or throw a specific error
        throw new Error('Network unavailable');
      } else {
        throw new Error(error.message || 'Failed to get user data');
      }
    }
  }

  static async refreshToken(): Promise<AuthResponse> {
    try {
      const response = await api.post<ApiResponse<AuthResponse>>(
        '/auth/refresh'
      );

      if (response.data.success) {
        return response.data.data!;
      } else {
        throw new Error(response.data.message || 'Token refresh failed');
      }
    } catch (error) {
      throw ApiService.handleApiError(error);
    }
  }

  static async logout(): Promise<void> {
    try {
      await api.post('/auth/logout');
    } catch (error) {
      // Don't throw on logout errors, just log them
      console.warn('Logout error:', error);
    }
  }

  // User management endpoints
  static async updateProfile(userData: Partial<User>): Promise<User> {
    try {
      const response = await api.put<ApiResponse<User>>(
        '/auth/profile',
        userData
      );

      if (!response.data.success) {
        throw new Error(response.data.error || 'Profile update failed');
      }

      return response.data.data!;
    } catch (error: any) {
      if (error.response?.data?.error) {
        throw new Error(error.response.data.error);
      }
      throw new Error(error.message || 'Profile update failed');
    }
  }

  static async changePassword(passwords: {
    currentPassword: string;
    newPassword: string;
  }): Promise<void> {
    try {
      const response = await api.post<ApiResponse>(
        '/auth/change-password',
        passwords
      );

      if (!response.data.success) {
        throw new Error(response.data.error || 'Password change failed');
      }
    } catch (error: any) {
      if (error.response?.data?.error) {
        throw new Error(error.response.data.error);
      }
      throw new Error(error.message || 'Password change failed');
    }
  }

  // Project management endpoints
  static async createProject(payload: {
    name: string;
    description?: string;
    config: any;
  }): Promise<any> {
    try {
      const response = await api.post<ApiResponse<any>>('/projects', payload);

      if (!response.data.success) {
        throw new Error(response.data.error || 'Project creation failed');
      }

      return response.data.data!;
    } catch (error: any) {
      if (error.response?.data?.error) {
        throw new Error(error.response.data.error);
      }
      throw new Error(error.message || 'Project creation failed');
    }
  }

  static async getProjects(): Promise<any[]> {
    try {
      const response = await api.get<ApiResponse<any[]>>('/projects');

      if (!response.data.success) {
        throw new Error(response.data.error || 'Failed to fetch projects');
      }

      return response.data.data!;
    } catch (error: any) {
      if (error.response?.data?.error) {
        throw new Error(error.response.data.error);
      }
      throw new Error(error.message || 'Failed to fetch projects');
    }
  }

  // Dockerfile generation endpoints
  static async generateDockerfile(projectId: string): Promise<any> {
    try {
      const response = await api.post<ApiResponse<any>>(
        `/dockerfiles/${projectId}/generate`
      );

      if (!response.data.success) {
        throw new Error(response.data.error || 'Dockerfile generation failed');
      }

      return response.data.data!;
    } catch (error: any) {
      if (error.response?.data?.error) {
        throw new Error(error.response.data.error);
      }
      throw new Error(error.message || 'Dockerfile generation failed');
    }
  }

  // Generic API methods
  static async get<T>(endpoint: string): Promise<T> {
    try {
      const response = await api.get<ApiResponse<T>>(endpoint);

      if (!response.data.success) {
        throw new Error(response.data.error || 'Request failed');
      }

      return response.data.data!;
    } catch (error: any) {
      if (error.response?.data?.error) {
        throw new Error(error.response.data.error);
      }
      throw new Error(error.message || 'Request failed');
    }
  }

  static async post<T>(endpoint: string, data?: any): Promise<T> {
    try {
      const response = await api.post<ApiResponse<T>>(endpoint, data);

      if (!response.data.success) {
        throw new Error(response.data.error || 'Request failed');
      }

      return response.data.data!;
    } catch (error: any) {
      if (error.response?.data?.error) {
        throw new Error(error.response.data.error);
      }
      throw new Error(error.message || 'Request failed');
    }
  }

  static async put<T>(endpoint: string, data?: any): Promise<T> {
    try {
      const response = await api.put<ApiResponse<T>>(endpoint, data);

      if (!response.data.success) {
        throw new Error(response.data.error || 'Request failed');
      }

      return response.data.data!;
    } catch (error: any) {
      if (error.response?.data?.error) {
        throw new Error(error.response.data.error);
      }
      throw new Error(error.message || 'Request failed');
    }
  }

  static async delete<T>(endpoint: string): Promise<T> {
    try {
      const response = await api.delete<ApiResponse<T>>(endpoint);

      if (!response.data.success) {
        throw new Error(response.data.error || 'Request failed');
      }

      return response.data.data!;
    } catch (error: any) {
      if (error.response?.data?.error) {
        throw new Error(error.response.data.error);
      }
      throw new Error(error.message || 'Request failed');
    }
  }
}

export default ApiService;
