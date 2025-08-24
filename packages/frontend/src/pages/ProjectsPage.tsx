import React, { useState, useEffect, useMemo } from 'react';
import {
  Box,
  Typography,
  Grid,
  Card,
  CardContent,
  CardActions,
  Button,
  Chip,
  CircularProgress,
  Alert,
  TextField,
  InputAdornment,
  IconButton,
  useTheme,
  useMediaQuery,
  ToggleButtonGroup,
  ToggleButton,
  Paper,
  Drawer,
  Menu,
  MenuItem,
  ListItemIcon,
  ListItemText,
  Tooltip,
} from '@mui/material';
import {
  Search as SearchIcon,
  Clear as ClearIcon,
  GridView as GridIcon,
  ViewList as ListIcon,
  FilterList as FilterListIcon,
  Add as AddIcon,
  MoreVert as MoreVertIcon,
  Edit as EditIcon,
  Delete as DeleteIcon,
  ContentCopy as DuplicateIcon,
  Settings as SettingsIcon,
} from '@mui/icons-material';
import { useNavigate } from 'react-router-dom';
import { ApiService } from '../services/api';
import ProjectFilters from '../components/ProjectFilters';
import { Project, ProjectFilters as ProjectFiltersType } from '@robium/shared';
import { useFilterData } from '../hooks/useFilterData';

type ViewMode = 'grid' | 'list';

const ProjectsPage: React.FC = () => {
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('md'));
  const navigate = useNavigate();

  // State
  const [projects, setProjects] = useState<Project[]>([]);
  const [searchQuery, setSearchQuery] = useState('');
  const [filters, setFilters] = useState<ProjectFiltersType>({
    useCases: [],
    capabilities: [],
    robots: [],
    tags: [],
  });
  const [viewMode, setViewMode] = useState<ViewMode>('grid');
  const [filtersOpen, setFiltersOpen] = useState(false);

  // Filter data management
  const { categories, filterValues, stats, loading, error, retryCount, refresh, clearError } =
    useFilterData({ isTemplate: false, autoRefresh: true });

  // Project action menu state
  const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
  const [selectedProject, setSelectedProject] = useState<Project | null>(null);

  // Load projects
  useEffect(() => {
    const loadProjects = async () => {
      try {
        const projectsResponse = await ApiService.getProjects();
        if (projectsResponse.success && projectsResponse.data && projectsResponse.data.projects) {
          setProjects(projectsResponse.data.projects);
        }
      } catch (err) {
        console.error('Error loading projects:', err);
      }
    };

    loadProjects();
  }, []);

  // Filter and search projects
  const processedProjects = useMemo(() => {
    let filtered = projects;

    // Apply search filter
    if (searchQuery) {
      const query = searchQuery.toLowerCase();
      filtered = filtered.filter(
        (project) =>
          project.name.toLowerCase().includes(query) ||
          project.description.toLowerCase().includes(query) ||
          project.tags.some((tag) => tag.toLowerCase().includes(query))
      );
    }

    // Apply filters dynamically based on available categories
    categories.forEach((category) => {
      const filterKey = category.name as keyof ProjectFiltersType;
      const filterValues = (filters as any)[filterKey] as string[];

      if (filterValues && filterValues.length > 0) {
        filtered = filtered.filter((project) => {
          // Handle different category types
          if (category.name === 'useCases' && project.metadata?.useCases) {
            return project.metadata.useCases.some((useCase) => filterValues.includes(useCase));
          }
          if (category.name === 'capabilities' && project.metadata?.capabilities) {
            return project.metadata.capabilities.some((capability) =>
              filterValues.includes(capability)
            );
          }
          if (category.name === 'robots' && project.metadata?.robots) {
            return project.metadata.robots.some((robot) => filterValues.includes(robot));
          }
          if (category.name === 'tags') {
            return project.tags.some((tag) => filterValues.includes(tag));
          }

          // For any other categories, check if they exist in project metadata
          if (project.metadata && (project.metadata as any)[category.name]) {
            const projectValues = (project.metadata as any)[category.name];
            if (Array.isArray(projectValues)) {
              return projectValues.some((value: string) => filterValues.includes(value));
            }
          }

          return false;
        });
      }
    });

    return filtered;
  }, [projects, searchQuery, filters, categories]);

  const handleClearSearch = () => {
    setSearchQuery('');
  };

  const handleClearFilters = () => {
    const clearedFilters: ProjectFiltersType = {
      useCases: [],
      capabilities: [],
      robots: [],
      tags: [],
    };

    // Dynamically clear all category filters
    categories.forEach((category) => {
      const filterKey = category.name as keyof ProjectFiltersType;
      if (filterKey in clearedFilters) {
        (clearedFilters as any)[filterKey] = [];
      }
    });

    setFilters(clearedFilters);
  };

  const getActiveFiltersCount = () => {
    return Object.values(filters).reduce((count, value) => {
      if (Array.isArray(value)) {
        return count + value.length;
      }
      return count + (value ? 1 : 0);
    }, 0);
  };

  const getAvailableTags = () => {
    const allTags = new Set<string>();
    projects.forEach((project) => {
      project.tags.forEach((tag) => allTags.add(tag));
    });
    return Array.from(allTags).sort();
  };

  // Project action handlers
  const handleProjectMenuOpen = (event: React.MouseEvent<HTMLElement>, project: Project) => {
    setAnchorEl(event.currentTarget);
    setSelectedProject(project);
  };

  const handleProjectMenuClose = () => {
    setAnchorEl(null);
    setSelectedProject(null);
  };

  const handleEditProject = () => {
    if (selectedProject) {
      navigate(`/projects/${selectedProject.id}/edit`);
      handleProjectMenuClose();
    }
  };

  const handleDuplicateProject = () => {
    if (selectedProject) {
      // TODO: Implement project duplication
      console.log('Duplicate project:', selectedProject.id);
      handleProjectMenuClose();
    }
  };

  const handleDeleteProject = async () => {
    if (selectedProject) {
      if (window.confirm(`Are you sure you want to delete "${selectedProject.name}"?`)) {
        try {
          await ApiService.deleteProject(selectedProject.id);
          // Re-fetch projects from the server to ensure we have the latest data
          const projectsResponse = await ApiService.getProjects();
          if (projectsResponse.success && projectsResponse.data && projectsResponse.data.projects) {
            setProjects(projectsResponse.data.projects);
          }
          // Show success message
          console.log('Project deleted successfully');
        } catch (error) {
          console.error('Failed to delete project:', error);
          // You might want to show an error message to the user here
        }
      }
      handleProjectMenuClose();
    }
  };

  const handleProjectSettings = () => {
    if (selectedProject) {
      navigate(`/projects/${selectedProject.id}/settings`);
      handleProjectMenuClose();
    }
  };

  // Loading state
  if (loading) {
    return (
      <Box display="flex" justifyContent="center" alignItems="center" minHeight="400px">
        <CircularProgress />
      </Box>
    );
  }

  // Error state
  if (error) {
    return (
      <Box display="flex" justifyContent="center" alignItems="center" minHeight="400px">
        <Alert severity="error">{error}</Alert>
      </Box>
    );
  }

  // Mobile layout with drawer
  if (isMobile) {
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
            <Typography variant="h4" gutterBottom>
              My Projects
            </Typography>
            <Typography variant="body2" color="text.secondary">
              {processedProjects.length} of {projects.length} projects
            </Typography>
          </Box>

          <Box sx={{ display: 'flex', gap: 1, alignItems: 'center' }}>
            <Button
              variant="contained"
              startIcon={<AddIcon />}
              onClick={() => navigate('/projects/new')}
              size="small"
            >
              New Project
            </Button>
            <ToggleButtonGroup
              value={viewMode}
              exclusive
              onChange={(_, newMode) => newMode && setViewMode(newMode)}
              size="small"
            >
              <ToggleButton value="grid">
                <GridIcon />
              </ToggleButton>
              <ToggleButton value="list">
                <ListIcon />
              </ToggleButton>
            </ToggleButtonGroup>
          </Box>
        </Box>

        {/* Search and Controls */}
        <Paper sx={{ p: 2, mb: 3 }}>
          <Box
            sx={{
              display: 'flex',
              gap: 2,
              alignItems: 'center',
              flexWrap: 'wrap',
            }}
          >
            {/* Search */}
            <TextField
              placeholder="Search projects..."
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              InputProps={{
                startAdornment: (
                  <InputAdornment position="start">
                    <SearchIcon />
                  </InputAdornment>
                ),
                endAdornment: searchQuery && (
                  <InputAdornment position="end">
                    <IconButton size="small" onClick={handleClearSearch}>
                      <ClearIcon />
                    </IconButton>
                  </InputAdornment>
                ),
              }}
              sx={{ flexGrow: 1, minWidth: 200 }}
              size="small"
            />

            {/* Filters Button */}
            <Button
              variant="outlined"
              startIcon={<FilterListIcon />}
              onClick={() => setFiltersOpen(true)}
              size="small"
            >
              Filters
              {getActiveFiltersCount() > 0 && (
                <Chip
                  label={getActiveFiltersCount()}
                  size="small"
                  color="primary"
                  sx={{ ml: 1, minWidth: 20, height: 20 }}
                />
              )}
            </Button>
          </Box>

          {/* Active Filters */}
          {getActiveFiltersCount() > 0 && (
            <Box
              sx={{
                mt: 2,
                display: 'flex',
                gap: 1,
                flexWrap: 'wrap',
                alignItems: 'center',
              }}
            >
              <Typography variant="body2" color="text.secondary">
                Active filters:
              </Typography>
              {Object.entries(filters).map(([key, value]) => {
                if (Array.isArray(value) && value.length > 0) {
                  return value.map((v) => (
                    <Chip
                      key={`${key}-${v}`}
                      label={v}
                      size="small"
                      onDelete={() => {
                        const newFilters = { ...filters };
                        (newFilters[key as keyof ProjectFiltersType] as string[]) = (
                          newFilters[key as keyof ProjectFiltersType] as string[]
                        ).filter((item) => item !== v);
                        setFilters(newFilters);
                      }}
                    />
                  ));
                }
                return null;
              })}
              <Button size="small" onClick={handleClearFilters}>
                Clear all
              </Button>
            </Box>
          )}
        </Paper>

        {/* Results */}
        <Box sx={{ mb: 2 }}>
          <Typography variant="body2" color="text.secondary">
            {processedProjects.length} of {projects.length} projects
          </Typography>
        </Box>

        {/* Projects Grid */}
        <Grid container spacing={3}>
          {processedProjects.map((project) => (
            <Grid
              item
              xs={12}
              sm={viewMode === 'list' ? 12 : 6}
              md={viewMode === 'list' ? 12 : 4}
              lg={viewMode === 'list' ? 12 : 3}
              key={project.id}
            >
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    {project.name}
                  </Typography>
                  <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
                    {project.description}
                  </Typography>
                  <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap', mb: 2 }}>
                    {project.tags.slice(0, 3).map((tag) => (
                      <Chip key={tag} label={tag} size="small" variant="outlined" />
                    ))}
                    {project.tags.length > 3 && (
                      <Chip label={`+${project.tags.length - 3}`} size="small" variant="outlined" />
                    )}
                  </Box>
                  <Typography variant="caption" color="text.secondary">
                    Created: {new Date(project.createdAt).toLocaleDateString()}
                  </Typography>
                </CardContent>
                <CardActions sx={{ justifyContent: 'space-between', px: 2, pb: 2 }}>
                  <Button
                    size="small"
                    variant="contained"
                    onClick={() => navigate(`/projects/${project.id}`)}
                  >
                    View
                  </Button>
                  <Box sx={{ display: 'flex', gap: 1 }}>
                    <Tooltip title="Edit">
                      <IconButton
                        size="small"
                        onClick={() => navigate(`/projects/${project.id}/edit`)}
                      >
                        <EditIcon fontSize="small" />
                      </IconButton>
                    </Tooltip>
                    <Tooltip title="Duplicate">
                      <IconButton size="small" onClick={() => handleDuplicateProject()}>
                        <DuplicateIcon fontSize="small" />
                      </IconButton>
                    </Tooltip>
                    <Tooltip title="More actions">
                      <IconButton size="small" onClick={(e) => handleProjectMenuOpen(e, project)}>
                        <MoreVertIcon fontSize="small" />
                      </IconButton>
                    </Tooltip>
                  </Box>
                </CardActions>
              </Card>
            </Grid>
          ))}
        </Grid>

        {/* Empty State */}
        {processedProjects.length === 0 && !loading && (
          <Box
            sx={{
              textAlign: 'center',
              py: 8,
              color: 'text.secondary',
            }}
          >
            <Typography variant="h6" gutterBottom>
              No projects found
            </Typography>
            <Typography variant="body2">
              Try adjusting your search or filters to find what you're looking for.
            </Typography>
          </Box>
        )}

        {/* Filters Drawer for Mobile */}
        <Drawer
          anchor="bottom"
          open={filtersOpen}
          onClose={() => setFiltersOpen(false)}
          PaperProps={{
            sx: {
              height: '80vh',
              borderTopLeftRadius: 16,
              borderTopRightRadius: 16,
            },
          }}
        >
          <ProjectFilters
            open={filtersOpen}
            onClose={() => setFiltersOpen(false)}
            filters={filters}
            onFiltersChange={setFilters}
            categories={categories}
            values={filterValues}
            stats={stats}
            availableTags={getAvailableTags()}
            loading={loading}
            error={error}
            onRetry={refresh}
          />
        </Drawer>
      </Box>
    );
  }

  // Desktop layout
  return (
    <Box>
      {/* Main Content: Left Sidebar + Cards Area */}
      <Box sx={{ display: 'flex', alignItems: 'flex-start', gap: 3 }}>
        {/* Left Sidebar */}
        <Box
          sx={{
            width: 300,
            flexShrink: 0,
            position: 'sticky',
            top: 0,
            alignSelf: 'flex-start',
          }}
        >
          {/* Active Filters Summary */}
          {getActiveFiltersCount() > 0 && (
            <Box
              sx={{
                mb: 2,
                display: 'flex',
                gap: 1,
                flexWrap: 'wrap',
                alignItems: 'center',
              }}
            >
              <Typography variant="body2" color="text.secondary">
                Active filters:
              </Typography>
              {Object.entries(filters).map(([key, value]) => {
                if (Array.isArray(value) && value.length > 0) {
                  return value.map((v) => (
                    <Chip
                      key={`${key}-${v}`}
                      label={v}
                      size="small"
                      onDelete={() => {
                        const newFilters = { ...filters };
                        (newFilters[key as keyof ProjectFiltersType] as string[]) = (
                          newFilters[key as keyof ProjectFiltersType] as string[]
                        ).filter((item) => item !== v);
                        setFilters(newFilters);
                      }}
                    />
                  ));
                }
                return null;
              })}
              <Button size="small" onClick={handleClearFilters}>
                Clear all
              </Button>
            </Box>
          )}

          {/* Filters */}
          <Box
            sx={{
              border: 1,
              borderColor: 'divider',
              borderRadius: 2,
              bgcolor: 'background.paper',
              p: 2,
            }}
          >
            <ProjectFilters
              open={true}
              onClose={() => {}}
              filters={filters}
              onFiltersChange={setFilters}
              categories={categories}
              values={filterValues}
              stats={stats}
              availableTags={getAvailableTags()}
              permanent={true}
              loading={loading}
              error={error}
              onRetry={refresh}
            />
          </Box>
        </Box>

        {/* Cards Area */}
        <Box sx={{ flex: 1 }}>
          {/* Header Controls and Search/Sort */}
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
            {/* Left side: Search Controls */}
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
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
                InputProps={{
                  startAdornment: (
                    <InputAdornment position="start">
                      <SearchIcon />
                    </InputAdornment>
                  ),
                  endAdornment: searchQuery && (
                    <InputAdornment position="end">
                      <IconButton size="small" onClick={handleClearSearch}>
                        <ClearIcon />
                      </IconButton>
                    </InputAdornment>
                  ),
                }}
                size="small"
                sx={{ minWidth: 200 }}
              />
            </Box>

            {/* Right side: Project count, View mode, Add button */}
            <Box sx={{ display: 'flex', gap: 1, alignItems: 'center' }}>
              <Typography variant="body2" color="text.secondary">
                {processedProjects.length} of {projects.length} projects
              </Typography>

              <Button
                variant="contained"
                startIcon={<AddIcon />}
                onClick={() => navigate('/projects/new')}
                size="small"
              >
                New Project
              </Button>

              <ToggleButtonGroup
                value={viewMode}
                exclusive
                onChange={(_, newMode) => newMode && setViewMode(newMode)}
                size="small"
              >
                <ToggleButton value="grid">
                  <GridIcon />
                </ToggleButton>
                <ToggleButton value="list">
                  <ListIcon />
                </ToggleButton>
              </ToggleButtonGroup>
            </Box>
          </Box>

          {/* Projects Grid */}
          {processedProjects.length > 0 ? (
            <Grid container spacing={3}>
              {processedProjects.map((project) => (
                <Grid item xs={12} sm={6} md={4} lg={3} key={project.id}>
                  <Card>
                    <CardContent>
                      <Typography variant="h6" gutterBottom>
                        {project.name}
                      </Typography>
                      <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
                        {project.description}
                      </Typography>
                      <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap', mb: 2 }}>
                        {project.tags.slice(0, 3).map((tag) => (
                          <Chip key={tag} label={tag} size="small" variant="outlined" />
                        ))}
                        {project.tags.length > 3 && (
                          <Chip
                            label={`+${project.tags.length - 3}`}
                            size="small"
                            variant="outlined"
                          />
                        )}
                      </Box>
                      <Typography variant="caption" color="text.secondary">
                        Created: {new Date(project.createdAt).toLocaleDateString()}
                      </Typography>
                    </CardContent>
                    <CardActions sx={{ justifyContent: 'space-between', px: 2, pb: 2 }}>
                      <Button
                        size="small"
                        variant="contained"
                        onClick={() => navigate(`/projects/${project.id}`)}
                      >
                        View
                      </Button>
                      <Box sx={{ display: 'flex', gap: 1 }}>
                        <Tooltip title="Edit">
                          <IconButton
                            size="small"
                            onClick={() => navigate(`/projects/${project.id}/edit`)}
                          >
                            <EditIcon fontSize="small" />
                          </IconButton>
                        </Tooltip>
                        <Tooltip title="Duplicate">
                          <IconButton size="small" onClick={() => handleDuplicateProject()}>
                            <DuplicateIcon fontSize="small" />
                          </IconButton>
                        </Tooltip>
                        <Tooltip title="More actions">
                          <IconButton
                            size="small"
                            onClick={(e) => handleProjectMenuOpen(e, project)}
                          >
                            <MoreVertIcon fontSize="small" />
                          </IconButton>
                        </Tooltip>
                      </Box>
                    </CardActions>
                  </Card>
                </Grid>
              ))}
            </Grid>
          ) : (
            <Box sx={{ textAlign: 'center', py: 8, color: 'text.secondary' }}>
              <Typography variant="h6" gutterBottom>
                No projects found
              </Typography>
              <Typography variant="body2">
                Try adjusting your search or filters to find what you're looking for.
              </Typography>
            </Box>
          )}
        </Box>
      </Box>

      {/* Mobile Filters Drawer remains */}
      <Drawer
        anchor="bottom"
        open={filtersOpen}
        onClose={() => setFiltersOpen(false)}
        PaperProps={{
          sx: {
            height: '80vh',
            borderTopLeftRadius: 16,
            borderTopRightRadius: 16,
          },
        }}
      >
        <ProjectFilters
          open={filtersOpen}
          onClose={() => setFiltersOpen(false)}
          filters={filters}
          onFiltersChange={setFilters}
          categories={categories}
          values={filterValues}
          stats={stats}
          availableTags={getAvailableTags()}
        />
      </Drawer>

      {/* Project Action Menu */}
      <Menu
        anchorEl={anchorEl}
        open={Boolean(anchorEl)}
        onClose={handleProjectMenuClose}
        anchorOrigin={{
          vertical: 'bottom',
          horizontal: 'right',
        }}
        transformOrigin={{
          vertical: 'top',
          horizontal: 'right',
        }}
      >
        <MenuItem onClick={handleProjectSettings}>
          <ListItemIcon>
            <SettingsIcon fontSize="small" />
          </ListItemIcon>
          <ListItemText>Settings</ListItemText>
        </MenuItem>
        <MenuItem onClick={handleEditProject}>
          <ListItemIcon>
            <EditIcon fontSize="small" />
          </ListItemIcon>
          <ListItemText>Edit</ListItemText>
        </MenuItem>
        <MenuItem onClick={handleDuplicateProject}>
          <ListItemIcon>
            <DuplicateIcon fontSize="small" />
          </ListItemIcon>
          <ListItemText>Duplicate</ListItemText>
        </MenuItem>
        <MenuItem onClick={handleDeleteProject} sx={{ color: 'error.main' }}>
          <ListItemIcon>
            <DeleteIcon fontSize="small" sx={{ color: 'error.main' }} />
          </ListItemIcon>
          <ListItemText>Delete</ListItemText>
        </MenuItem>
      </Menu>
    </Box>
  );
};

export default ProjectsPage;
