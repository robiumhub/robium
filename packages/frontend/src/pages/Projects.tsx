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
  MenuItem,
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
  Badge,
} from '@mui/material';
import Grid from '@mui/material/Grid';
import {
  Add as AddIcon,
  Folder as FolderIcon,
  Search as SearchIcon,
  ViewList as ViewListIcon,
  ViewModule as ViewModuleIcon,
  FilterList as FilterIcon,
  Sort as SortIcon,
  Edit as EditIcon,
  Delete as DeleteIcon,
  ContentCopy as CloneIcon,
  MoreVert as MoreIcon,
  Refresh as RefreshIcon,
  Visibility as ViewIcon,
  Settings as SettingsIcon,
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

// Types
interface Project {
  id: string;
  name: string;
  description: string;
  status: 'active' | 'completed' | 'planning' | 'archived';
  category: string;
  moduleCount: number;
  lastUpdated: string;
  createdAt: string;
  owner: string;
  tags: string[];
  isPublic: boolean;
}

interface ProjectFilters {
  search: string;
  status: string;
  category: string;
  sortBy: string;
  sortOrder: 'asc' | 'desc';
}

const Projects: React.FC = () => {
  const navigate = useNavigate();
  const location = useLocation();
  const [projects, setProjects] = useState<Project[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [viewMode, setViewMode] = useState<'grid' | 'list'>('grid');
  const [filters, setFilters] = useState<ProjectFilters>({
    search: '',
    status: '',
    category: '',
    sortBy: 'lastUpdated',
    sortOrder: 'desc',
  });
  const [showFilters, setShowFilters] = useState(false);
  const [deleteDialogOpen, setDeleteDialogOpen] = useState(false);
  const [projectToDelete, setProjectToDelete] = useState<Project | null>(null);
  const [dockerfileDialogOpen, setDockerfileDialogOpen] = useState(false);
  const [selectedProject, setSelectedProject] = useState<Project | null>(null);
  const [dockerfileContent, setDockerfileContent] = useState<string>('');
  const [dockerfileLoading, setDockerfileLoading] = useState(false);
  const [dockerfileError, setDockerfileError] = useState<string | null>(null);

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
            status: 'active' as const, // Default status since backend doesn't have this field yet
            category: 'General', // Default category since backend doesn't have this field yet
            moduleCount: 0, // Default since backend doesn't have this field yet
            lastUpdated: project.updated_at,
            createdAt: project.created_at,
            owner: project.owner_username || 'Unknown',
            tags: [], // Default since backend doesn't have this field yet
            isPublic: false, // Default since backend doesn't have this field yet
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
        project.tags.some((tag) =>
          tag.toLowerCase().includes(filters.search.toLowerCase())
        );

      const matchesStatus =
        !filters.status || project.status === filters.status;
      const matchesCategory =
        !filters.category || project.category === filters.category;

      return matchesSearch && matchesStatus && matchesCategory;
    });

    // Sort projects
    filtered.sort((a, b) => {
      let aValue: any, bValue: any;

      switch (filters.sortBy) {
        case 'name':
          aValue = a.name.toLowerCase();
          bValue = b.name.toLowerCase();
          break;
        case 'status':
          aValue = a.status;
          bValue = b.status;
          break;
        case 'category':
          aValue = a.category;
          bValue = b.category;
          break;
        case 'moduleCount':
          aValue = a.moduleCount;
          bValue = b.moduleCount;
          break;
        case 'createdAt':
          aValue = new Date(a.createdAt);
          bValue = new Date(b.createdAt);
          break;
        default:
          aValue = new Date(a.lastUpdated);
          bValue = new Date(b.lastUpdated);
      }

      if (filters.sortOrder === 'asc') {
        return aValue > bValue ? 1 : -1;
      } else {
        return aValue < bValue ? 1 : -1;
      }
    });

    return filtered;
  }, [projects, filters]);

  // Get unique categories and statuses for filters
  const categories = useMemo(() => {
    const uniqueCategories = Array.from(
      new Set(projects.map((p) => p.category))
    );
    return uniqueCategories.sort();
  }, [projects]);

  const statuses = useMemo(() => {
    const uniqueStatuses = Array.from(new Set(projects.map((p) => p.status)));
    return uniqueStatuses.sort();
  }, [projects]);

  // Status color mapping
  const getStatusColor = (status: string) => {
    switch (status) {
      case 'active':
        return 'success';
      case 'completed':
        return 'default';
      case 'planning':
        return 'warning';
      case 'archived':
        return 'error';
      default:
        return 'default';
    }
  };

  // Handle project actions
  const handleEditProject = (project: Project) => {
    navigate(`/projects/${project.id}/edit`);
  };

  const handleCloneProject = (project: Project) => {
    // TODO: Implement clone functionality
    console.log('Clone project:', project.id);
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
      // TODO: Replace with actual API call
      // await ApiService.delete(`/projects/${projectToDelete.id}`);

      setProjects((prev) => prev.filter((p) => p.id !== projectToDelete.id));
      setDeleteDialogOpen(false);
      setProjectToDelete(null);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to delete project');
    }
  };

  const handleViewDockerfile = async (project: Project) => {
    try {
      setSelectedProject(project);
      setDockerfileDialogOpen(true);
      setDockerfileLoading(true);
      setDockerfileError(null);

      const result = await ApiService.get<{
        success: boolean;
        data: { content: string };
        message?: string;
      }>(`/dockerfiles/${project.id}`);

      if (result.success) {
        setDockerfileContent(result.data.content);
      } else {
        throw new Error(result.message || 'Failed to load Dockerfile');
      }
    } catch (error) {
      console.error('Failed to load Dockerfile:', error);
      setDockerfileError(
        error instanceof Error ? error.message : 'Failed to load Dockerfile'
      );
    } finally {
      setDockerfileLoading(false);
    }
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
      {/* Header */}
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
        <Box>
          <Typography variant="h4" component="h1" gutterBottom>
            Projects
          </Typography>
          <Typography variant="body2" color="text.secondary">
            {filteredProjects.length} of {projects.length} projects
          </Typography>
        </Box>

        <Box sx={{ display: 'flex', gap: 1, alignItems: 'center' }}>
          <Tooltip title="Refresh">
            <IconButton onClick={() => window.location.reload()}>
              <RefreshIcon />
            </IconButton>
          </Tooltip>

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

          <Button
            variant="contained"
            startIcon={<AddIcon />}
            component={RouterLink}
            to="/projects/new"
          >
            New Project
          </Button>
        </Box>
      </Box>

      {/* Search and Filters */}
      <Box sx={{ mb: 3 }}>
        <Grid container spacing={2} alignItems="center">
          <Grid item xs={12} md={6}>
            <TextField
              fullWidth
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
            />
          </Grid>

          <Grid item xs={12} md={6}>
            <Box sx={{ display: 'flex', gap: 1, alignItems: 'center' }}>
              <FormControl size="small" sx={{ minWidth: 120 }}>
                <InputLabel>Status</InputLabel>
                <Select
                  value={filters.status}
                  label="Status"
                  onChange={(e) =>
                    setFilters((prev) => ({ ...prev, status: e.target.value }))
                  }
                >
                  <MenuItem value="">All</MenuItem>
                  {statuses.map((status) => (
                    <MenuItem key={status} value={status}>
                      {status.charAt(0).toUpperCase() + status.slice(1)}
                    </MenuItem>
                  ))}
                </Select>
              </FormControl>

              <FormControl size="small" sx={{ minWidth: 120 }}>
                <InputLabel>Category</InputLabel>
                <Select
                  value={filters.category}
                  label="Category"
                  onChange={(e) =>
                    setFilters((prev) => ({
                      ...prev,
                      category: e.target.value,
                    }))
                  }
                >
                  <MenuItem value="">All</MenuItem>
                  {categories.map((category) => (
                    <MenuItem key={category} value={category}>
                      {category}
                    </MenuItem>
                  ))}
                </Select>
              </FormControl>

              <FormControl size="small" sx={{ minWidth: 140 }}>
                <InputLabel>Sort By</InputLabel>
                <Select
                  value={filters.sortBy}
                  label="Sort By"
                  onChange={(e) =>
                    setFilters((prev) => ({ ...prev, sortBy: e.target.value }))
                  }
                >
                  <MenuItem value="lastUpdated">Last Updated</MenuItem>
                  <MenuItem value="name">Name</MenuItem>
                  <MenuItem value="status">Status</MenuItem>
                  <MenuItem value="category">Category</MenuItem>
                  <MenuItem value="moduleCount">Modules</MenuItem>
                  <MenuItem value="createdAt">Created</MenuItem>
                </Select>
              </FormControl>

              <IconButton
                onClick={() =>
                  setFilters((prev) => ({
                    ...prev,
                    sortOrder: prev.sortOrder === 'asc' ? 'desc' : 'asc',
                  }))
                }
              >
                <SortIcon
                  sx={{
                    transform:
                      filters.sortOrder === 'desc' ? 'rotate(180deg)' : 'none',
                  }}
                />
              </IconButton>
            </Box>
          </Grid>
        </Grid>
      </Box>

      {/* Error Alert */}
      {error && (
        <Alert severity="error" sx={{ mb: 3 }}>
          {error}
        </Alert>
      )}

      {/* Projects Grid/List */}
      {loading ? (
        <Grid container spacing={3}>
          {[...Array(6)].map((_, index) => (
            <Grid item xs={12} sm={6} md={4} key={index}>
              <ProjectSkeleton />
            </Grid>
          ))}
        </Grid>
      ) : filteredProjects.length === 0 ? (
        <Box
          sx={{
            textAlign: 'center',
            py: 8,
            color: 'text.secondary',
          }}
        >
          <FolderIcon sx={{ fontSize: 64, mb: 2, opacity: 0.5 }} />
          <Typography variant="h6" gutterBottom>
            No projects found
          </Typography>
          <Typography variant="body2">
            {filters.search || filters.status || filters.category
              ? 'Try adjusting your search or filters'
              : 'Create your first project to get started'}
          </Typography>
        </Box>
      ) : (
        <Grid container spacing={3}>
          {filteredProjects.map((project) => (
            <Grid item xs={12} sm={6} md={4} key={project.id}>
              <Card
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
                      <Box sx={{ display: 'flex', gap: 1, mb: 1 }}>
                        <Chip
                          label={project.status}
                          color={getStatusColor(project.status) as any}
                          size="small"
                        />
                        <Chip
                          label={project.category}
                          variant="outlined"
                          size="small"
                        />
                        {project.isPublic && (
                          <Chip label="Public" color="info" size="small" />
                        )}
                      </Box>
                    </Box>
                  </Box>

                  <Typography
                    variant="body2"
                    color="text.secondary"
                    sx={{ mb: 2, lineHeight: 1.5 }}
                  >
                    {project.description}
                  </Typography>

                  <Box sx={{ mb: 2 }}>
                    {project.tags.slice(0, 3).map((tag) => (
                      <Chip
                        key={tag}
                        label={tag}
                        size="small"
                        sx={{ mr: 0.5, mb: 0.5 }}
                      />
                    ))}
                    {project.tags.length > 3 && (
                      <Chip
                        label={`+${project.tags.length - 3}`}
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
                      {project.moduleCount} module
                      {project.moduleCount !== 1 ? 's' : ''}
                    </Typography>
                    <Typography variant="caption" color="text.secondary">
                      Updated:{' '}
                      {new Date(project.lastUpdated).toLocaleDateString()}
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
                    <Tooltip title="More actions">
                      <IconButton size="small">
                        <MoreIcon />
                      </IconButton>
                    </Tooltip>
                  </Box>
                </CardActions>
              </Card>
            </Grid>
          ))}
        </Grid>
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
    </Box>
  );
};

export default Projects;
