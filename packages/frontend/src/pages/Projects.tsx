import React, { useState, useEffect, useMemo } from 'react';
import {
  Box,
  Typography,
  Card,
  CardContent,
  CardActions,
  Button,
  Chip,
  Avatar,
  TextField,
  InputAdornment,
  FormControl,
  InputLabel,
  Select,
  IconButton,
  Tooltip,
  ToggleButton,
  ToggleButtonGroup,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  Alert,
  Skeleton,
  Fab,
  Menu,
  MenuItem,
  ListItemIcon,
  ListItemText,
  Divider,
  useTheme,
  useMediaQuery,
} from '@mui/material';

import {
  Add as AddIcon,
  Folder as FolderIcon,
  Search as SearchIcon,
  ViewList as ViewListIcon,
  ViewModule as ViewModuleIcon,
  Sort as SortIcon,
  Edit as EditIcon,
  Delete as DeleteIcon,
  ContentCopy as CloneIcon,
  MoreVert as MoreIcon,
  Refresh as RefreshIcon,
  Visibility as ViewIcon,
  Download as ExportIcon,
  Share as ShareIcon,
  Code as CodeIcon,
} from '@mui/icons-material';
import {
  Link as RouterLink,
  useNavigate,
  Outlet,
  useLocation,
} from 'react-router-dom';
import { ApiService } from '../services/api';
import { useAuth } from '../contexts/AuthContext';

// Types
interface Project {
  id: string;
  name: string;
  description: string;
  version: string;
  author: string;
  maintainer_email: string;
  license: string;
  type: 'custom' | 'template' | 'example';
  is_active: boolean;
  is_template: boolean;
  tags: string[];
  config: Record<string, any>;
  metadata: Record<string, any>;
  workspace_path: string;
  source_path: string;
  config_path: string;
  created_at: string;
  updated_at: string;
  created_by: string;
  updated_by: string;
  module_count: number;
  package_count: number;
  modules?: Module[];
  packages?: Package[];
  github_repo_url?: string;
  github_repo_owner?: string;
  github_repo_name?: string;
}

interface Module {
  id: string;
  name: string;
  description: string;
  category: string;
  type: string;
  dependency_type: string;
  order_index: number;
}

interface Package {
  id: string;
  name: string;
  description: string;
  category: string;
  type: string;
  is_required: boolean;
  order_index: number;
}

interface ProjectFilters {
  search: string;
  sortBy: string;
  sortOrder: 'asc' | 'desc';
  tags?: string[];
}

const Projects: React.FC = () => {
  const navigate = useNavigate();
  const location = useLocation();
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('md'));
  const [projects, setProjects] = useState<Project[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [viewMode, setViewMode] = useState<'grid' | 'list'>('grid');
  const [filters, setFilters] = useState<ProjectFilters>({
    search: '',
    sortBy: 'lastUpdated',
    sortOrder: 'desc',
  });

  const [deleteDialogOpen, setDeleteDialogOpen] = useState(false);
  const [projectToDelete, setProjectToDelete] = useState<Project | null>(null);
  const [dockerfileDialogOpen, setDockerfileDialogOpen] = useState(false);
  const [selectedProject, setSelectedProject] = useState<Project | null>(null);
  const [dockerfileContent, setDockerfileContent] = useState<string>('');
  const [dockerfileLoading, setDockerfileLoading] = useState(false);
  const [dockerfileError, setDockerfileError] = useState<string | null>(null);
  const [menuAnchorEl, setMenuAnchorEl] = useState<null | HTMLElement>(null);
  const [selectedProjectForMenu, setSelectedProjectForMenu] =
    useState<Project | null>(null);
  const { user } = useAuth();

  // Projects will be loaded from the API

  // Load projects
  useEffect(() => {
    const loadProjects = async () => {
      try {
        setLoading(true);

        // Fetch projects from the API using the proper service
        const result = await ApiService.get<any[]>('/projects');

        // Debug: Log the API response
        console.log('API Response:', result);

        // Transform backend data to match frontend interface
        // Handle case where data might be undefined or null
        const projectsData = result || [];
        console.log('Projects data:', projectsData);
        const transformedProjects: Project[] = projectsData.map(
          (project: any) => ({
            id: project.id,
            name: project.name,
            description: project.description || '',
            version: project.version || '1.0.0',
            author: project.author || 'Unknown',
            maintainer_email: project.maintainer_email || '',
            license: project.license || 'MIT',
            type: project.type || 'custom',
            is_active: project.is_active || true,
            is_template: project.is_template || false,
            tags: project.tags || [],
            config: project.config || {},
            metadata: project.metadata || {},
            workspace_path: project.workspace_path || '',
            source_path: project.source_path || '',
            config_path: project.config_path || '',
            created_at: project.created_at,
            updated_at: project.updated_at,
            created_by: project.created_by || 'Unknown',
            updated_by: project.updated_by || 'Unknown',
            module_count: project.module_count || 0,
            package_count: project.package_count || 0,
            modules: project.modules || [],
            packages: project.packages || [],
            github_repo_url: project.github_repo_url,
            github_repo_owner: project.github_repo_owner,
            github_repo_name: project.github_repo_name,
          })
        );

        setProjects(transformedProjects);
        setError(null);
      } catch (err) {
        console.error('Failed to load projects:', err);
        console.error('Error details:', {
          message: err instanceof Error ? err.message : 'Unknown error',
          stack: err instanceof Error ? err.stack : undefined,
        });
        setError(
          err instanceof Error ? err.message : 'Failed to load projects'
        );
        // Fallback to empty array instead of mock data
        setProjects([]);
      } finally {
        setLoading(false);
      }
    };

    loadProjects();
  }, []);

  // Filter and sort projects
  const filteredProjects = useMemo(() => {
    let filtered = projects.filter((project) => {
      const matchesSearch =
        project.name.toLowerCase().includes(filters.search.toLowerCase()) ||
        project.description
          .toLowerCase()
          .includes(filters.search.toLowerCase()) ||
        project.modules?.some((module) =>
          module.name.toLowerCase().includes(filters.search.toLowerCase())
        ) ||
        project.packages?.some((pkg) =>
          pkg.name.toLowerCase().includes(filters.search.toLowerCase())
        );
      const matchesTags =
        !filters.tags || filters.tags.length === 0
          ? true
          : (project.tags || []).some((t) => filters.tags!.includes(t));

      return matchesSearch && matchesTags;
    });

    // Sort projects
    filtered.sort((a, b) => {
      let aValue: any, bValue: any;

      switch (filters.sortBy) {
        case 'name':
          aValue = a.name.toLowerCase();
          bValue = b.name.toLowerCase();
          break;
        case 'moduleCount':
          aValue = a.module_count;
          bValue = b.module_count;
          break;
        case 'packageCount':
          aValue = a.package_count;
          bValue = b.package_count;
          break;
        case 'createdAt':
          aValue = new Date(a.created_at);
          bValue = new Date(b.created_at);
          break;
        default:
          aValue = new Date(a.updated_at);
          bValue = new Date(b.updated_at);
      }

      if (filters.sortOrder === 'asc') {
        return aValue > bValue ? 1 : -1;
      } else {
        return aValue < bValue ? 1 : -1;
      }
    });

    return filtered;
  }, [projects, filters]);

  // Removed type filtering; all projects are custom

  const availableTags = useMemo(() => {
    const set = new Set<string>();
    projects.forEach((p) => (p.tags || []).forEach((t) => set.add(t)));
    return Array.from(set).sort();
  }, [projects]);

  // Handle project actions
  const handleEditProject = (project: Project) => {
    navigate(`/projects/${project.id}/edit`);
  };

  const handleCloneProject = async (project: Project) => {
    try {
      setError(null);
      // Call backend clone endpoint
      const cloned = await ApiService.post<Project>(
        `/projects/${project.id}/clone`
      );
      // Optimistically add to list
      setProjects((prev) => [cloned as Project, ...prev]);
    } catch (err) {
      console.error('Failed to clone project:', err);
      setError(err instanceof Error ? err.message : 'Failed to clone project');
    }
  };

  const handleExportProject = (project: Project) => {
    // TODO: Implement export functionality
    console.log('Export project:', project.id);
  };

  const handleShareProject = (project: Project) => {
    // TODO: Implement share functionality
    console.log('Share project:', project.id);
  };

  const handleDeleteProject = (project: Project) => {
    setProjectToDelete(project);
    setDeleteDialogOpen(true);
  };

  const confirmDeleteProject = async () => {
    if (!projectToDelete) return;

    try {
      // Call the API to delete the project
      await ApiService.delete(`/projects/${projectToDelete.id}`);

      // Remove the project from the local state
      setProjects((prev) => prev.filter((p) => p.id !== projectToDelete.id));
      setDeleteDialogOpen(false);
      setProjectToDelete(null);
    } catch (err) {
      console.error('Failed to delete project:', err);
      setError(err instanceof Error ? err.message : 'Failed to delete project');
    }
  };

  const handleViewDockerfile = async (project: Project) => {
    try {
      setSelectedProject(project);
      setDockerfileDialogOpen(true);
      setDockerfileLoading(true);
      setDockerfileError(null);

      const result = await ApiService.get<{ content: string }>(
        `/dockerfiles/${project.id}`
      );

      setDockerfileContent(result.content);
    } catch (error) {
      console.error('Failed to load Dockerfile:', error);
      setDockerfileError(
        error instanceof Error ? error.message : 'Failed to load Dockerfile'
      );
    } finally {
      setDockerfileLoading(false);
    }
  };

  // Menu handlers
  const handleMenuOpen = (
    event: React.MouseEvent<HTMLElement>,
    project: Project
  ) => {
    setMenuAnchorEl(event.currentTarget);
    setSelectedProjectForMenu(project);
  };

  const handleMenuClose = () => {
    setMenuAnchorEl(null);
    setSelectedProjectForMenu(null);
  };

  const handleMenuAction = (action: string) => {
    if (!selectedProjectForMenu) return;

    switch (action) {
      case 'delete':
        handleDeleteProject(selectedProjectForMenu);
        break;
      case 'clone':
        handleCloneProject(selectedProjectForMenu);
        break;
      case 'export':
        handleExportProject(selectedProjectForMenu);
        break;
      case 'share':
        handleShareProject(selectedProjectForMenu);
        break;
      case 'dockerfile':
        handleViewDockerfile(selectedProjectForMenu);
        break;
      case 'convertTemplate':
        (async () => {
          try {
            await ApiService.post(`/projects/${selectedProjectForMenu.id}/convert-to-template`);
            setProjects((prev) =>
              prev.map((p) =>
                p.id === selectedProjectForMenu.id ? { ...p, is_template: true } : p
              )
            );
          } catch (err) {
            console.error('Failed to convert to template:', err);
            setError(
              err instanceof Error ? err.message : 'Failed to convert to template'
            );
          }
        })();
        break;
    }
    handleMenuClose();
  };

  // Loading skeleton
  const ProjectSkeleton = () => (
    <Card sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
      <CardContent sx={{ flexGrow: 1 }}>
        <Box sx={{ display: 'flex', alignItems: 'center', mb: 2 }}>
          <Skeleton variant="circular" width={40} height={40} sx={{ mr: 2 }} />
          <Box sx={{ flexGrow: 1 }}>
            <Skeleton variant="text" width="60%" height={24} />
            <Skeleton variant="text" width="40%" height={20} />
          </Box>
        </Box>
        <Skeleton variant="text" width="100%" height={16} sx={{ mb: 1 }} />
        <Skeleton variant="text" width="80%" height={16} sx={{ mb: 2 }} />
        <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
          <Skeleton variant="text" width="30%" height={16} />
          <Skeleton variant="text" width="40%" height={16} />
        </Box>
      </CardContent>
      <CardActions>
        <Skeleton variant="rectangular" width={80} height={32} />
        <Skeleton variant="rectangular" width={60} height={32} />
      </CardActions>
    </Card>
  );

  // If we're on a nested route, just render the outlet
  if (location.pathname !== '/projects') {
    return <Outlet />;
  }

  return (
    <Box>
      {/* Error Alert */}
      {error && (
        <Alert severity="error" sx={{ mb: 3 }}>
          {error}
        </Alert>
      )}

      {/* Projects Grid/List */}
      {isMobile ? (
        // original content for mobile
        <>
          {loading ? (
            <Box
              sx={{
                display: 'grid',
                gridTemplateColumns: {
                  xs: '1fr',
                  sm: 'repeat(2, 1fr)',
                  md: 'repeat(3, 1fr)',
                },
                gap: 3,
              }}
            >
              {[...Array(6)].map((_, index) => (
                <Box key={index}>
                  <ProjectSkeleton />
                </Box>
              ))}
            </Box>
          ) : filteredProjects.length === 0 ? (
            <Box sx={{ textAlign: 'center', py: 8, color: 'text.secondary' }}>
              <FolderIcon sx={{ fontSize: 64, mb: 2, opacity: 0.5 }} />
              <Typography variant="h6" gutterBottom>
                No projects found
              </Typography>
              <Typography variant="body2">
                {filters.search
                  ? 'Try adjusting your search or filters'
                  : 'Create your first project to get started'}
              </Typography>
            </Box>
          ) : (
            <Box
              sx={{
                display: 'grid',
                gridTemplateColumns: {
                  xs: '1fr',
                  sm: 'repeat(2, 1fr)',
                  md: 'repeat(3, 1fr)',
                },
                gap: 3,
              }}
            >
              {filteredProjects.map((project) => (
                <Card
                  key={project.id}
                  sx={{
                    height: '100%',
                    display: 'flex',
                    flexDirection: 'column',
                    position: 'relative',
                    '&:hover': {
                      boxShadow: 4,
                      transform: 'translateY(-2px)',
                      transition: 'all 0.2s ease-in-out',
                    },
                  }}
                >
                  <CardContent sx={{ flexGrow: 1 }}>
                    <Box
                      sx={{ display: 'flex', alignItems: 'flex-start', mb: 2 }}
                    >
                      <Avatar sx={{ mr: 2, bgcolor: 'primary.main' }}>
                        <FolderIcon />
                      </Avatar>
                      <Box sx={{ flexGrow: 1 }}>
                        <Typography variant="h6" component="h2" gutterBottom>
                          {project.name}
                        </Typography>
                      </Box>
                    </Box>
                    <Typography
                      variant="body2"
                      color="text.secondary"
                      sx={{ mb: 2, lineHeight: 1.5 }}
                    >
                      {project.description}
                    </Typography>
                    {project.tags && project.tags.length > 0 && (
                      <Box sx={{ mb: 2 }}>
                        {project.tags.slice(0, 5).map((tag, index) => (
                          <Chip
                            key={index}
                            label={tag}
                            size="small"
                            variant="outlined"
                            color="primary"
                            sx={{ mr: 0.5, mb: 0.5 }}
                          />
                        ))}
                        {project.tags.length > 5 && (
                          <Chip
                            label={`+${project.tags.length - 5}`}
                            size="small"
                            variant="outlined"
                          />
                        )}
                      </Box>
                    )}
                    <Box sx={{ mb: 2 }}>
                      {project.modules?.slice(0, 3).map((module) => (
                        <Chip
                          key={module.id}
                          label={module.name}
                          size="small"
                          sx={{ mr: 0.5, mb: 0.5 }}
                        />
                      ))}
                      {(project.modules?.length || 0) > 3 && (
                        <Chip
                          label={`+${(project.modules?.length || 0) - 3}`}
                          size="small"
                          variant="outlined"
                        />
                      )}
                    </Box>
                    <Box sx={{ mb: 2 }}>
                      {project.packages?.slice(0, 3).map((pkg) => (
                        <Chip
                          key={pkg.id}
                          label={pkg.name}
                          size="small"
                          sx={{ mr: 0.5, mb: 0.5 }}
                        />
                      ))}
                      {(project.packages?.length || 0) > 3 && (
                        <Chip
                          label={`+${(project.packages?.length || 0) - 3}`}
                          size="small"
                          variant="outlined"
                        />
                      )}
                    </Box>
                    <Box
                      sx={{
                        display: 'flex',
                        justifyContent: 'space-between',
                        alignItems: 'center',
                      }}
                    >
                      <Typography variant="caption" color="text.secondary">
                        {project.module_count} module
                        {project.module_count !== 1 ? 's' : ''}
                      </Typography>
                      <Typography variant="caption" color="text.secondary">
                        Updated:{' '}
                        {new Date(project.updated_at).toLocaleDateString()}
                      </Typography>
                    </Box>
                  </CardContent>
                  <CardActions sx={{ justifyContent: 'space-between' }}>
                    <Box>
                      <Button
                        size="small"
                        component={RouterLink}
                        to={`/workspace/${project.id}`}
                        startIcon={<CodeIcon />}
                        variant="contained"
                        color="primary"
                      >
                        Open Workspace
                      </Button>
                      <Button
                        size="small"
                        component={RouterLink}
                        to={`/projects/${project.id}`}
                        startIcon={<ViewIcon />}
                      >
                        View
                      </Button>
                      <Button
                        size="small"
                        onClick={() => handleEditProject(project)}
                        startIcon={<EditIcon />}
                      >
                        Edit
                      </Button>
                    </Box>
                    <Box>
                      <Tooltip title="Delete project">
                        <IconButton
                          size="small"
                          color="error"
                          onClick={() => handleDeleteProject(project)}
                        >
                          <DeleteIcon />
                        </IconButton>
                      </Tooltip>
                      <Tooltip title="More actions">
                        <IconButton
                          size="small"
                          onClick={(e) => handleMenuOpen(e, project)}
                        >
                          <MoreIcon />
                        </IconButton>
                      </Tooltip>
                    </Box>
                  </CardActions>
                </Card>
              ))}
            </Box>
          )}
        </>
      ) : (
        <Box sx={{ display: 'flex', alignItems: 'flex-start', gap: 3 }}>
          {/* Left Sidebar */}
          <Box
            sx={{
              width: 300,
              flexShrink: 0,
              position: 'sticky',
              top: 0,
              alignSelf: 'flex-start',
              border: 1,
              borderColor: 'divider',
              borderRadius: 2,
              bgcolor: 'background.paper',
              p: 2,
            }}
          >
            {/* New Project Button */}
            <Box sx={{ mb: 3 }}>
              <Button
                variant="contained"
                fullWidth
                startIcon={<AddIcon />}
                component={RouterLink}
                to="/projects/new"
                sx={{ mb: 2 }}
              >
                New Project
              </Button>
            </Box>

            {/* Tags */}
            <Box sx={{ mb: 2 }}>
              <Typography variant="subtitle2" sx={{ mb: 1, fontWeight: 600 }}>
                Tags
              </Typography>
              <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                {availableTags.map((tag) => {
                  const selected = (filters.tags || []).includes(tag);
                  return (
                    <Chip
                      key={tag}
                      label={tag}
                      clickable
                      color={selected ? 'primary' : 'default'}
                      variant={selected ? 'filled' : 'outlined'}
                      onClick={() => {
                        const exists = (filters.tags || []).includes(tag);
                        const next = exists
                          ? (filters.tags || []).filter((t) => t !== tag)
                          : [...(filters.tags || []), tag];
                        setFilters((prev) => ({ ...prev, tags: next }));
                      }}
                      size="small"
                    />
                  );
                })}
              </Box>
            </Box>
          </Box>

          {/* Right content */}
          <Box sx={{ flex: 1 }}>
            {/* Header Controls and Search/Sort - Cards Section */}
            <Box
              sx={{
                display: 'flex',
                justifyContent: 'space-between',
                alignItems: 'center',
                mb: 3,
                flexWrap: 'wrap',
                gap: 2,
              }}
            >
              {/* Left side: Search and Sort Controls */}
              <Box
                sx={{
                  display: 'flex',
                  gap: 2,
                  flexWrap: 'wrap',
                  alignItems: 'center',
                }}
              >
                <TextField
                  placeholder="Search projects..."
                  value={filters.search}
                  onChange={(e) =>
                    setFilters((prev) => ({ ...prev, search: e.target.value }))
                  }
                  InputProps={{
                    startAdornment: (
                      <InputAdornment position="start">
                        <SearchIcon />
                      </InputAdornment>
                    ),
                  }}
                  size="small"
                  sx={{ minWidth: 200 }}
                />

                <FormControl size="small" sx={{ minWidth: 140 }}>
                  <InputLabel>Sort By</InputLabel>
                  <Select
                    value={filters.sortBy}
                    label="Sort By"
                    onChange={(e) =>
                      setFilters((prev) => ({
                        ...prev,
                        sortBy: e.target.value,
                      }))
                    }
                  >
                    <MenuItem value="lastUpdated">Last Updated</MenuItem>
                    <MenuItem value="name">Name</MenuItem>
                    <MenuItem value="moduleCount">Modules</MenuItem>
                    <MenuItem value="packageCount">Packages</MenuItem>
                    <MenuItem value="createdAt">Created</MenuItem>
                  </Select>
                </FormControl>

                <ToggleButtonGroup
                  value={filters.sortOrder}
                  exclusive
                  onChange={(_, value) =>
                    value &&
                    setFilters((prev) => ({ ...prev, sortOrder: value }))
                  }
                  size="small"
                >
                  <ToggleButton value="asc">↑</ToggleButton>
                  <ToggleButton value="desc">↓</ToggleButton>
                </ToggleButtonGroup>
              </Box>

              {/* Right side: Project count, View mode */}
              <Box sx={{ display: 'flex', gap: 1, alignItems: 'center' }}>
                <Typography variant="body2" color="text.secondary">
                  {filteredProjects.length} of {projects.length} projects
                </Typography>

                <ToggleButtonGroup
                  value={viewMode}
                  exclusive
                  onChange={(_, newMode) => newMode && setViewMode(newMode)}
                  size="small"
                >
                  <ToggleButton value="grid">
                    <ViewModuleIcon />
                  </ToggleButton>
                  <ToggleButton value="list">
                    <ViewListIcon />
                  </ToggleButton>
                </ToggleButtonGroup>
              </Box>
            </Box>

            {loading ? (
              <Box
                sx={{
                  display: 'grid',
                  gridTemplateColumns: {
                    xs: '1fr',
                    sm: 'repeat(2, 1fr)',
                    md: 'repeat(3, 1fr)',
                  },
                  gap: 3,
                }}
              >
                {[...Array(6)].map((_, index) => (
                  <Box key={index}>
                    <ProjectSkeleton />
                  </Box>
                ))}
              </Box>
            ) : filteredProjects.length === 0 ? (
              <Box sx={{ textAlign: 'center', py: 8, color: 'text.secondary' }}>
                <FolderIcon sx={{ fontSize: 64, mb: 2, opacity: 0.5 }} />
                <Typography variant="h6" gutterBottom>
                  No projects found
                </Typography>
                <Typography variant="body2">
                  {filters.search
                    ? 'Try adjusting your search or filters'
                    : 'Create your first project to get started'}
                </Typography>
              </Box>
            ) : (
              <Box
                sx={{
                  display: 'grid',
                  gridTemplateColumns: {
                    xs: '1fr',
                    sm: 'repeat(2, 1fr)',
                    md: 'repeat(3, 1fr)',
                  },
                  gap: 3,
                }}
              >
                {filteredProjects.map((project) => (
                  <Card
                    key={project.id}
                    sx={{
                      height: '100%',
                      display: 'flex',
                      flexDirection: 'column',
                      position: 'relative',
                      '&:hover': {
                        boxShadow: 4,
                        transform: 'translateY(-2px)',
                        transition: 'all 0.2s ease-in-out',
                      },
                    }}
                  >
                    <CardContent sx={{ flexGrow: 1 }}>
                      <Box
                        sx={{
                          display: 'flex',
                          alignItems: 'flex-start',
                          mb: 2,
                        }}
                      >
                        <Avatar sx={{ mr: 2, bgcolor: 'primary.main' }}>
                          <FolderIcon />
                        </Avatar>
                        <Box sx={{ flexGrow: 1 }}>
                          <Typography variant="h6" component="h2" gutterBottom>
                            {project.name}
                          </Typography>
                        </Box>
                      </Box>
                      <Typography
                        variant="body2"
                        color="text.secondary"
                        sx={{ mb: 2, lineHeight: 1.5 }}
                      >
                        {project.description}
                      </Typography>
                      {project.tags && project.tags.length > 0 && (
                        <Box sx={{ mb: 2 }}>
                          {project.tags.slice(0, 5).map((tag, index) => (
                            <Chip
                              key={index}
                              label={tag}
                              size="small"
                              variant="outlined"
                              color="primary"
                              sx={{ mr: 0.5, mb: 0.5 }}
                            />
                          ))}
                          {project.tags.length > 5 && (
                            <Chip
                              label={`+${project.tags.length - 5}`}
                              size="small"
                              variant="outlined"
                            />
                          )}
                        </Box>
                      )}
                      <Box sx={{ mb: 2 }}>
                        {project.modules?.slice(0, 3).map((module) => (
                          <Chip
                            key={module.id}
                            label={module.name}
                            size="small"
                            sx={{ mr: 0.5, mb: 0.5 }}
                          />
                        ))}
                        {(project.modules?.length || 0) > 3 && (
                          <Chip
                            label={`+${(project.modules?.length || 0) - 3}`}
                            size="small"
                            variant="outlined"
                          />
                        )}
                      </Box>
                      {project.github_repo_url && (
                        <Box sx={{ mb: 2 }}>
                          <Button
                            size="small"
                            href={project.github_repo_url}
                            target="_blank"
                            rel="noopener noreferrer"
                          >
                            View GitHub Repo
                          </Button>
                        </Box>
                      )}
                      <Box sx={{ mb: 2 }}>
                        {project.packages?.slice(0, 3).map((pkg) => (
                          <Chip
                            key={pkg.id}
                            label={pkg.name}
                            size="small"
                            sx={{ mr: 0.5, mb: 0.5 }}
                          />
                        ))}
                        {(project.packages?.length || 0) > 3 && (
                          <Chip
                            label={`+${(project.packages?.length || 0) - 3}`}
                            size="small"
                            variant="outlined"
                          />
                        )}
                      </Box>
                      <Box
                        sx={{
                          display: 'flex',
                          justifyContent: 'space-between',
                          alignItems: 'center',
                        }}
                      >
                        <Typography variant="caption" color="text.secondary">
                          {project.module_count} module
                          {project.module_count !== 1 ? 's' : ''}
                        </Typography>
                        <Typography variant="caption" color="text.secondary">
                          Updated:{' '}
                          {new Date(project.updated_at).toLocaleDateString()}
                        </Typography>
                      </Box>
                    </CardContent>
                    <CardActions sx={{ justifyContent: 'space-between' }}>
                      <Box>
                        <Button
                          size="small"
                          component={RouterLink}
                          to={`/workspace/${project.id}`}
                          startIcon={<CodeIcon />}
                          variant="contained"
                          color="primary"
                        >
                          Open Workspace
                        </Button>
                        <Button
                          size="small"
                          component={RouterLink}
                          to={`/projects/${project.id}`}
                          startIcon={<ViewIcon />}
                        >
                          View
                        </Button>
                        <Button
                          size="small"
                          onClick={() => handleEditProject(project)}
                          startIcon={<EditIcon />}
                        >
                          Edit
                        </Button>
                      </Box>
                      <Box>
                        <Tooltip title="Delete project">
                          <IconButton
                            size="small"
                            color="error"
                            onClick={() => handleDeleteProject(project)}
                          >
                            <DeleteIcon />
                          </IconButton>
                        </Tooltip>
                        <Tooltip title="More actions">
                          <IconButton
                            size="small"
                            onClick={(e) => handleMenuOpen(e, project)}
                          >
                            <MoreIcon />
                          </IconButton>
                        </Tooltip>
                      </Box>
                    </CardActions>
                  </Card>
                ))}
              </Box>
            )}
          </Box>
        </Box>
      )}

      {/* Delete Confirmation Dialog */}
      <Dialog
        open={deleteDialogOpen}
        onClose={() => setDeleteDialogOpen(false)}
      >
        <DialogTitle>Delete Project</DialogTitle>
        <DialogContent>
          <Typography>
            Are you sure you want to delete "{projectToDelete?.name}"? This
            action cannot be undone.
          </Typography>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setDeleteDialogOpen(false)}>Cancel</Button>
          <Button
            onClick={confirmDeleteProject}
            color="error"
            variant="contained"
          >
            Delete
          </Button>
        </DialogActions>
      </Dialog>

      {/* Dockerfile View Dialog */}
      <Dialog
        open={dockerfileDialogOpen}
        onClose={() => setDockerfileDialogOpen(false)}
        maxWidth="lg"
        fullWidth
      >
        <DialogTitle>Dockerfile - {selectedProject?.name}</DialogTitle>
        <DialogContent>
          {dockerfileLoading ? (
            <Box sx={{ display: 'flex', justifyContent: 'center', p: 3 }}>
              <Typography>Loading Dockerfile...</Typography>
            </Box>
          ) : dockerfileError ? (
            <Alert severity="error" sx={{ mb: 2 }}>
              {dockerfileError}
            </Alert>
          ) : (
            <Box sx={{ mt: 1 }}>
              <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
                Generated Dockerfile for {selectedProject?.name}
              </Typography>
              <Box
                component="pre"
                sx={{
                  backgroundColor: 'grey.100',
                  p: 2,
                  borderRadius: 1,
                  overflow: 'auto',
                  maxHeight: '400px',
                  fontFamily: 'monospace',
                  fontSize: '0.875rem',
                  border: '1px solid',
                  borderColor: 'grey.300',
                }}
              >
                {dockerfileContent}
              </Box>
            </Box>
          )}
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setDockerfileDialogOpen(false)}>Close</Button>
        </DialogActions>
      </Dialog>

      {/* Floating Action Button - only show on projects list page */}
      {location.pathname === '/projects' && (
        <Fab
          color="primary"
          aria-label="add project"
          sx={{ position: 'fixed', bottom: 16, right: 16 }}
          component={RouterLink}
          to="/projects/new"
        >
          <AddIcon />
        </Fab>
      )}

      {/* Outlet for nested routes */}
      <Outlet />

      {/* More Actions Menu */}
      <Menu
        anchorEl={menuAnchorEl}
        open={Boolean(menuAnchorEl)}
        onClose={handleMenuClose}
        anchorOrigin={{
          vertical: 'bottom',
          horizontal: 'right',
        }}
        transformOrigin={{
          vertical: 'top',
          horizontal: 'right',
        }}
      >
        <MenuItem onClick={() => handleMenuAction('clone')}>
          <ListItemIcon>
            <CloneIcon fontSize="small" />
          </ListItemIcon>
          <ListItemText>Clone Project</ListItemText>
        </MenuItem>
        <MenuItem onClick={() => handleMenuAction('export')}>
          <ListItemIcon>
            <ExportIcon fontSize="small" />
          </ListItemIcon>
          <ListItemText>Export Project</ListItemText>
        </MenuItem>
        <MenuItem onClick={() => handleMenuAction('share')}>
          <ListItemIcon>
            <ShareIcon fontSize="small" />
          </ListItemIcon>
          <ListItemText>Share Project</ListItemText>
        </MenuItem>
        <MenuItem onClick={() => handleMenuAction('dockerfile')}>
          <ListItemIcon>
            <CodeIcon fontSize="small" />
          </ListItemIcon>
          <ListItemText>View Dockerfile</ListItemText>
        </MenuItem>
        {user?.role === 'admin' && selectedProjectForMenu && !selectedProjectForMenu.is_template && (
          <MenuItem onClick={() => handleMenuAction('convertTemplate')}>
            <ListItemIcon>
              <EditIcon fontSize="small" />
            </ListItemIcon>
            <ListItemText>Convert to Template</ListItemText>
          </MenuItem>
        )}
        <MenuItem
          onClick={() => handleMenuAction('delete')}
          sx={{ color: 'error.main' }}
        >
          <ListItemIcon>
            <DeleteIcon fontSize="small" color="error" />
          </ListItemIcon>
          <ListItemText>Delete Project</ListItemText>
        </MenuItem>
      </Menu>
    </Box>
  );
};

export default Projects;
