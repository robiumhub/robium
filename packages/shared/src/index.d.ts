export interface ApiResponse<T = any> {
    success: boolean;
    message?: string;
    data?: T;
    error?: string;
}
export interface User {
    id: string;
    email: string;
    username: string;
    role: 'admin' | 'user';
    createdAt: Date;
    updatedAt: Date;
}
export interface Project {
    id: string;
    name: string;
    description: string;
    ownerId: string;
    isActive: boolean;
    isTemplate: boolean;
    tags: string[];
    config: Record<string, any>;
    metadata: ProjectMetadata;
    templateVisibility?: 'public' | 'private';
    templateVersion?: string;
    templatePublishedAt?: Date;
    createdAt: Date;
    updatedAt: Date;
}
export interface ProjectMetadata {
    useCases?: string[];
    capabilities?: string[];
    robots?: string[];
    simulators?: string[];
    difficulty?: 'beginner' | 'intermediate' | 'advanced';
    estimatedRuntime?: string;
    hardwareRequirements?: string[];
    dependencies?: string[];
}
export interface ProjectFilters {
    useCases: string[];
    capabilities: string[];
    robots: string[];
    simulators: string[];
    difficulty: string[];
    tags: string[];
    searchQuery?: string;
}
export interface FilterCategory {
    id: string;
    name: string;
    displayName: string;
    description?: string;
    type: 'string' | 'boolean' | 'number';
    isActive: boolean;
    sortOrder: number;
    createdAt: Date;
    updatedAt: Date;
}
export interface FilterValue {
    id: string;
    categoryId: string;
    value: string;
    displayName: string;
    description?: string;
    isActive: boolean;
    sortOrder: number;
    createdAt: Date;
    updatedAt: Date;
}
export interface FilterStats {
    totalItems: number;
    useCaseCounts: Record<string, number>;
    capabilityCounts: Record<string, number>;
    robotCounts: Record<string, number>;
    simulatorCounts: Record<string, number>;
    difficultyCounts: Record<string, number>;
    tagCounts: Record<string, number>;
}
export interface FacetCounts {
    useCases: Record<string, number>;
    capabilities: Record<string, number>;
    robots: Record<string, number>;
    simulators: Record<string, number>;
    difficulty: Record<string, number>;
    tags: Record<string, number>;
}
export interface ProjectFilters {
    use_cases: string[];
    capabilities: string[];
    robots: string[];
    simulators: string[];
    difficulty: string[];
    tags: string[];
    searchQuery?: string;
}
export interface FacetCounts {
    use_cases: Record<string, number>;
    capabilities: Record<string, number>;
    robots: Record<string, number>;
    simulators: Record<string, number>;
    difficulty: Record<string, number>;
    tags: Record<string, number>;
}
export interface DockerfileResponse {
    content: string;
    filename: string;
    projectId: string;
    createdAt: Date;
}
export interface GitHubStatus {
    connected: boolean;
    username?: string;
    repositories?: string[];
    lastSync?: Date;
}
export interface AdminDashboard {
    totalUsers: number;
    totalProjects: number;
    totalTemplates: number;
    recentActivity: Array<{
        id: string;
        type: 'user_created' | 'project_created' | 'template_created';
        userId: string;
        username: string;
        timestamp: Date;
        details?: string;
    }>;
}
export interface PaginationParams {
    page: number;
    limit: number;
    sortBy?: string;
    sortOrder?: 'asc' | 'desc';
}
export interface PaginatedResponse<T> {
    items: T[];
    total: number;
    page: number;
    limit: number;
    totalPages: number;
    hasNext: boolean;
    hasPrev: boolean;
}
//# sourceMappingURL=index.d.ts.map