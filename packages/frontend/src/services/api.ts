import axios, { AxiosResponse } from 'axios';
import {
  ApiResponse,
  User,
  Project,
  FilterCategory,
  FilterValue,
  DockerfileResponse,
  GitHubStatus,
  GitHubRepo,
  GitHubCreateRepoOptions,
  AdminDashboard,
} from '@robium/shared';

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

// Create axios instance with base configuration
const apiClient = axios.create({
  baseURL: API_BASE_URL,
  headers: {
    'Content-Type': 'application/json',
  },
});

// Add request interceptor to include auth token
apiClient.interceptors.request.use((config) => {
  const token = localStorage.getItem('authToken');
  if (token) {
    config.headers.Authorization = `Bearer ${token}`;
  }
  return config;
});

// Add response interceptor to handle auth errors
apiClient.interceptors.response.use(
  (response) => response,
  (error) => {
    if (error.response?.status === 401) {
      localStorage.removeItem('authToken');
      window.location.href = '/login';
    }
    return Promise.reject(error);
  }
);

export class ApiService {
  // Auth endpoints
  static async signup(userData: {
    email: string;
    username: string;
    password: string;
  }): Promise<ApiResponse<{ user: User; token: string }>> {
    const response: AxiosResponse<ApiResponse<{ user: User; token: string }>> =
      await apiClient.post('/api/auth/signup', userData);
    return response.data;
  }

  static async login(credentials: {
    email: string;
    password: string;
  }): Promise<ApiResponse<{ user: User; token: string; expiresIn: string }>> {
    const response: AxiosResponse<ApiResponse<{ user: User; token: string; expiresIn: string }>> =
      await apiClient.post('/api/auth/login', credentials);
    return response.data;
  }

  static async getCurrentUser(): Promise<ApiResponse<User>> {
    const response: AxiosResponse<ApiResponse<User>> = await apiClient.get('/api/auth/me');
    return response.data;
  }

  static async changePassword(passwords: {
    currentPassword: string;
    newPassword: string;
  }): Promise<ApiResponse<void>> {
    const response: AxiosResponse<ApiResponse<void>> = await apiClient.post(
      '/api/auth/change-password',
      passwords
    );
    return response.data;
  }

  static async updateProfile(updates: {
    username?: string;
    email?: string;
  }): Promise<ApiResponse<User>> {
    const response: AxiosResponse<ApiResponse<User>> = await apiClient.put(
      '/api/auth/profile',
      updates
    );
    return response.data;
  }

  static async refreshToken(): Promise<ApiResponse<{ token: string; expiresIn: string }>> {
    const response: AxiosResponse<ApiResponse<{ token: string; expiresIn: string }>> =
      await apiClient.post('/api/auth/refresh');
    return response.data;
  }

  static async logout(): Promise<ApiResponse<void>> {
    const response: AxiosResponse<ApiResponse<void>> = await apiClient.post('/api/auth/logout');
    return response.data;
  }

  // Project endpoints
  static async getProjects(): Promise<ApiResponse<{ projects: Project[] }>> {
    const response: AxiosResponse<ApiResponse<{ projects: Project[] }>> =
      await apiClient.get('/api/projects');
    return response.data;
  }

  static async getProject(id: string): Promise<ApiResponse<{ project: Project }>> {
    const response: AxiosResponse<ApiResponse<{ project: Project }>> = await apiClient.get(
      `/api/projects/${id}`
    );
    return response.data;
  }

  static async createProject(projectData: {
    name: string;
    description?: string;
    tags?: string[];
    isTemplate?: boolean;
    config?: Record<string, any>;
    metadata?: Record<string, any>;
    github?: GitHubCreateRepoOptions;
  }): Promise<ApiResponse<{ project: Project; githubRepo?: GitHubRepo }>> {
    const response: AxiosResponse<ApiResponse<{ project: Project; githubRepo?: GitHubRepo }>> =
      await apiClient.post('/api/projects', projectData);
    return response.data;
  }

  static async updateProject(
    id: string,
    updates: Partial<Project>
  ): Promise<ApiResponse<{ project: Project }>> {
    const response: AxiosResponse<ApiResponse<{ project: Project }>> = await apiClient.put(
      `/api/projects/${id}`,
      updates
    );
    return response.data;
  }

  static async deleteProject(id: string): Promise<ApiResponse<void>> {
    const response: AxiosResponse<ApiResponse<void>> = await apiClient.delete(
      `/api/projects/${id}`
    );
    return response.data;
  }

  // Template endpoints
  static async getTemplates(): Promise<ApiResponse<{ projects: Project[] }>> {
    const response: AxiosResponse<ApiResponse<{ projects: Project[] }>> =
      await apiClient.get('/api/projects/templates');
    return response.data;
  }

  static async cloneProject(
    id: string,
    name: string,
    github?: GitHubCreateRepoOptions
  ): Promise<ApiResponse<{ project: Project; githubRepo?: GitHubRepo }>> {
    const response: AxiosResponse<ApiResponse<{ project: Project; githubRepo?: GitHubRepo }>> =
      await apiClient.post(`/api/projects/${id}/clone`, { name, github });
    return response.data;
  }

  static async convertToTemplate(
    id: string,
    visibility: 'public' | 'private' = 'public',
    version: string = '1.0.0'
  ): Promise<ApiResponse<{ project: Project }>> {
    const response: AxiosResponse<ApiResponse<{ project: Project }>> = await apiClient.post(
      `/api/projects/${id}/convert-to-template`,
      { visibility, version }
    );
    return response.data;
  }

  // Filter endpoints
  static async getFilterCategories(): Promise<ApiResponse<{ categories: FilterCategory[] }>> {
    const response: AxiosResponse<ApiResponse<{ categories: FilterCategory[] }>> =
      await apiClient.get('/api/projects/filters/categories');
    return response.data;
  }

  static async createFilterCategory(categoryData: {
    name: string;
    displayName: string;
    description?: string;
    isActive?: boolean;
    sortOrder?: number;
  }): Promise<ApiResponse<FilterCategory>> {
    const response: AxiosResponse<ApiResponse<FilterCategory>> = await apiClient.post(
      '/api/projects/filters/categories',
      categoryData
    );
    return response.data;
  }

  static async updateFilterCategory(
    id: string,
    updates: Partial<FilterCategory>
  ): Promise<ApiResponse<FilterCategory>> {
    const response: AxiosResponse<ApiResponse<FilterCategory>> = await apiClient.put(
      `/api/projects/filters/categories/${id}`,
      updates
    );
    return response.data;
  }

  static async deleteFilterCategory(id: string): Promise<ApiResponse<void>> {
    const response: AxiosResponse<ApiResponse<void>> = await apiClient.delete(
      `/api/projects/filters/categories/${id}`
    );
    return response.data;
  }

  static async getFilterValues(): Promise<ApiResponse<{ values: FilterValue[] }>> {
    const response: AxiosResponse<ApiResponse<{ values: FilterValue[] }>> = await apiClient.get(
      '/api/projects/filters/values'
    );
    return response.data;
  }

  static async getCategoryValues(
    categoryId: string
  ): Promise<ApiResponse<{ values: FilterValue[] }>> {
    const response: AxiosResponse<ApiResponse<{ values: FilterValue[] }>> = await apiClient.get(
      `/api/projects/filters/categories/${categoryId}/values`
    );
    return response.data;
  }

  static async createFilterValue(valueData: {
    categoryId: string;
    value: string;
    displayName: string;
    description?: string;
    isActive?: boolean;
    sortOrder?: number;
  }): Promise<ApiResponse<FilterValue>> {
    const response: AxiosResponse<ApiResponse<FilterValue>> = await apiClient.post(
      '/api/projects/filters/values',
      valueData
    );
    return response.data;
  }

  static async updateFilterValue(
    id: string,
    updates: Partial<FilterValue>
  ): Promise<ApiResponse<FilterValue>> {
    const response: AxiosResponse<ApiResponse<FilterValue>> = await apiClient.put(
      `/api/projects/filters/values/${id}`,
      updates
    );
    return response.data;
  }

  static async deleteFilterValue(id: string): Promise<ApiResponse<void>> {
    const response: AxiosResponse<ApiResponse<void>> = await apiClient.delete(
      `/api/projects/filters/values/${id}`
    );
    return response.data;
  }

  static async getFilterStats(
    isTemplate: boolean = false
  ): Promise<ApiResponse<{ stats: Record<string, Record<string, number>> }>> {
    const response: AxiosResponse<ApiResponse<{ stats: Record<string, Record<string, number>> }>> =
      await apiClient.get(`/api/projects/filters/stats?isTemplate=${isTemplate}`);
    return response.data;
  }

  // Dockerfile endpoints
  static async generateDockerfile(
    projectId: string,
    options?: Record<string, any>
  ): Promise<ApiResponse<DockerfileResponse>> {
    const response: AxiosResponse<ApiResponse<DockerfileResponse>> = await apiClient.post(
      `/api/dockerfiles/${projectId}/generate`,
      { options }
    );
    return response.data;
  }

  static async getDockerfile(projectId: string): Promise<ApiResponse<DockerfileResponse>> {
    const response: AxiosResponse<ApiResponse<DockerfileResponse>> = await apiClient.get(
      `/api/dockerfiles/${projectId}`
    );
    return response.data;
  }

  // GitHub integration endpoints
  static async getGitHubStatus(): Promise<ApiResponse<GitHubStatus>> {
    const response: AxiosResponse<ApiResponse<GitHubStatus>> = await apiClient.get(
      '/api/integrations/github/status'
    );
    return response.data;
  }

  static async connectGitHub(token: string): Promise<ApiResponse<GitHubStatus>> {
    const response: AxiosResponse<ApiResponse<GitHubStatus>> = await apiClient.post(
      '/api/integrations/github/connect',
      { token }
    );
    return response.data;
  }

  static async disconnectGitHub(): Promise<ApiResponse<void>> {
    const response: AxiosResponse<ApiResponse<void>> = await apiClient.post(
      '/api/integrations/github/disconnect'
    );
    return response.data;
  }

  static async syncGitHub(): Promise<ApiResponse<void>> {
    const response: AxiosResponse<ApiResponse<void>> = await apiClient.post(
      '/api/integrations/github/sync'
    );
    return response.data;
  }

  // User management endpoints
  static async getUsers(): Promise<ApiResponse<{ users: User[] }>> {
    const response: AxiosResponse<ApiResponse<{ users: User[] }>> =
      await apiClient.get('/api/users');
    return response.data;
  }

  static async getUser(id: string): Promise<ApiResponse<User>> {
    const response: AxiosResponse<ApiResponse<User>> = await apiClient.get(`/api/users/${id}`);
    return response.data;
  }

  static async createUser(userData: {
    email: string;
    username: string;
    password: string;
    role?: 'user' | 'admin';
  }): Promise<ApiResponse<User>> {
    const response: AxiosResponse<ApiResponse<User>> = await apiClient.post('/api/users', userData);
    return response.data;
  }

  static async updateUser(
    id: string,
    updates: {
      email?: string;
      username?: string;
      password?: string;
      role?: 'user' | 'admin';
      isActive?: boolean;
    }
  ): Promise<ApiResponse<User>> {
    const response: AxiosResponse<ApiResponse<User>> = await apiClient.put(
      `/api/users/${id}`,
      updates
    );
    return response.data;
  }

  static async deleteUser(id: string): Promise<ApiResponse<void>> {
    const response: AxiosResponse<ApiResponse<void>> = await apiClient.delete(`/api/users/${id}`);
    return response.data;
  }

  // Admin endpoints
  static async getAdminDashboard(): Promise<ApiResponse<AdminDashboard>> {
    const response: AxiosResponse<ApiResponse<AdminDashboard>> =
      await apiClient.get('/api/admin/dashboard');
    return response.data;
  }

  // Health check
  static async healthCheck(): Promise<ApiResponse<{ status: string; timestamp: string }>> {
    const response: AxiosResponse<ApiResponse<{ status: string; timestamp: string }>> =
      await apiClient.get('/api/health');
    return response.data;
  }
}

export default ApiService;
