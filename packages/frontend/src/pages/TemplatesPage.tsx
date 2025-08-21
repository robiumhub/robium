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
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  FormControl,
  FormLabel,
  FormHelperText,
} from '@mui/material';
import {
  Search as SearchIcon,
  Clear as ClearIcon,
  GridView as GridIcon,
  ViewList as ListIcon,
  FilterList as FilterListIcon,
  Add as AddIcon,
} from '@mui/icons-material';
import { useNavigate } from 'react-router-dom';
import { ApiService } from '../services/api';
import ProjectFilters from '../components/ProjectFilters';
import {
  Project,
  ProjectFilters as ProjectFiltersType,
  FilterCategory,
  FilterValue,
} from '@robium/shared';

type ViewMode = 'grid' | 'list';

const TemplatesPage: React.FC = () => {
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('md'));
  const navigate = useNavigate();

  // State
  const [templates, setTemplates] = useState<Project[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [searchQuery, setSearchQuery] = useState('');
  const [filters, setFilters] = useState<ProjectFiltersType>({
    useCases: [],
    capabilities: [],
    robots: [],
    simulators: [],
    difficulty: [],
    tags: [],
  });
  const [viewMode, setViewMode] = useState<ViewMode>('grid');
  const [filtersOpen, setFiltersOpen] = useState(false);
  const [categories, setCategories] = useState<FilterCategory[]>([]);
  const [filterValues, setFilterValues] = useState<FilterValue[]>([]);
  const [stats, setStats] = useState<Record<string, Record<string, number>>>({});
  
  // Template dialog state
  const [dialogOpen, setDialogOpen] = useState(false);
  const [selectedTemplate, setSelectedTemplate] = useState<Project | null>(null);
  const [newProjectName, setNewProjectName] = useState('');
  const [newProjectDescription, setNewProjectDescription] = useState('');
  const [isCreating, setIsCreating] = useState(false);

  // Load templates and filter data
  useEffect(() => {
    const loadData = async () => {
      try {
        setLoading(true);
        setError(null);

        // Load templates
        const templatesResponse = await ApiService.getTemplates();
        if (
          templatesResponse.success &&
          templatesResponse.data &&
          templatesResponse.data.projects
        ) {
          setTemplates(templatesResponse.data.projects);
        } else {
          setError('Failed to fetch templates');
        }

        // Load filter categories
        const categoriesResponse = await ApiService.getFilterCategories();
        if (categoriesResponse.success && categoriesResponse.data) {
          setCategories(categoriesResponse.data.categories);
        }

        // Load filter values
        const valuesResponse = await ApiService.getFilterValues();
        if (valuesResponse.success && valuesResponse.data) {
          setFilterValues(valuesResponse.data.values);
        }

        // Load filter stats for templates
        const statsResponse = await ApiService.getFilterStats(true);
        if (statsResponse.success && statsResponse.data) {
          setStats(statsResponse.data.stats);
        }
      } catch (err) {
        console.error('Error loading data:', err);
        setError('Failed to load data');
      } finally {
        setLoading(false);
      }
    };

    loadData();
  }, []);

  // Filter and search templates
  const processedTemplates = useMemo(() => {
    let filtered = templates;

    // Apply search filter
    if (searchQuery) {
      const query = searchQuery.toLowerCase();
      filtered = filtered.filter(
        (template) =>
          template.name.toLowerCase().includes(query) ||
          template.description.toLowerCase().includes(query) ||
          template.tags.some((tag) => tag.toLowerCase().includes(query))
      );
    }

    // Apply filters
    if (filters.useCases.length > 0) {
      filtered = filtered.filter((template) =>
        template.metadata?.useCases?.some((useCase) => filters.useCases.includes(useCase))
      );
    }

    if (filters.capabilities.length > 0) {
      filtered = filtered.filter((template) =>
        template.metadata?.capabilities?.some((capability) =>
          filters.capabilities.includes(capability)
        )
      );
    }

    if (filters.robots.length > 0) {
      filtered = filtered.filter((template) =>
        template.metadata?.robots?.some((robot) => filters.robots.includes(robot))
      );
    }

    if (filters.simulators.length > 0) {
      filtered = filtered.filter((template) =>
        template.metadata?.simulators?.some((simulator) => filters.simulators.includes(simulator))
      );
    }

    if (filters.difficulty.length > 0) {
      filtered = filtered.filter(
        (template) =>
          template.metadata?.difficulty && filters.difficulty.includes(template.metadata.difficulty)
      );
    }

    if (filters.tags.length > 0) {
      filtered = filtered.filter((template) =>
        template.tags.some((tag) => filters.tags.includes(tag))
      );
    }

    return filtered;
  }, [templates, searchQuery, filters]);

  const handleClearSearch = () => {
    setSearchQuery('');
  };

  const handleClearFilters = () => {
    setFilters({
      useCases: [],
      capabilities: [],
      robots: [],
      simulators: [],
      difficulty: [],
      tags: [],
    });
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
    templates.forEach((template) => {
      template.tags.forEach((tag) => allTags.add(tag));
    });
    return Array.from(allTags).sort();
  };

  const handleUseTemplate = (template: Project) => {
    setSelectedTemplate(template);
    setNewProjectName(`${template.name}-copy`);
    setNewProjectDescription(template.description);
    setDialogOpen(true);
  };

  const handleCreateProject = async () => {
    if (!selectedTemplate || !newProjectName.trim()) return;

    setIsCreating(true);
    try {
      const response = await ApiService.cloneProject(selectedTemplate.id, newProjectName.trim());
      if (response.success && response.data) {
        setDialogOpen(false);
        setSelectedTemplate(null);
        setNewProjectName('');
        setNewProjectDescription('');
        navigate(`/projects/${response.data.project.id}`);
      }
    } catch (err) {
      console.error('Failed to create project from template:', err);
      alert('Failed to create project from template');
    } finally {
      setIsCreating(false);
    }
  };

  const handleDialogClose = () => {
    setDialogOpen(false);
    setSelectedTemplate(null);
    setNewProjectName('');
    setNewProjectDescription('');
    setIsCreating(false);
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
              Templates
            </Typography>
            <Typography variant="body2" color="text.secondary">
              {processedTemplates.length} of {templates.length} templates
            </Typography>
          </Box>

          <Box sx={{ display: 'flex', gap: 1, alignItems: 'center' }}>
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
              placeholder="Search templates..."
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
            {processedTemplates.length} of {templates.length} templates
          </Typography>
        </Box>

        {/* Templates Grid */}
        <Grid container spacing={3}>
          {processedTemplates.map((template) => (
            <Grid
              item
              xs={12}
              sm={viewMode === 'list' ? 12 : 6}
              md={viewMode === 'list' ? 12 : 4}
              lg={viewMode === 'list' ? 12 : 3}
              key={template.id}
            >
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    {template.name}
                  </Typography>
                  <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
                    {template.description}
                  </Typography>
                  <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap', mb: 2 }}>
                    {template.tags.slice(0, 3).map((tag) => (
                      <Chip key={tag} label={tag} size="small" variant="outlined" />
                    ))}
                    {template.tags.length > 3 && (
                      <Chip
                        label={`+${template.tags.length - 3}`}
                        size="small"
                        variant="outlined"
                      />
                    )}
                  </Box>
                  <Typography variant="caption" color="text.secondary">
                    Version: {template.templateVersion || '1.0.0'} • Created:{' '}
                    {new Date(template.createdAt).toLocaleDateString()}
                  </Typography>
                </CardContent>
                <CardActions>
                  <Button size="small" onClick={() => handleUseTemplate(template)}>
                    Use Template
                  </Button>
                  <Button size="small" variant="outlined">
                    Preview
                  </Button>
                </CardActions>
              </Card>
            </Grid>
          ))}
        </Grid>

        {/* Empty State */}
        {processedTemplates.length === 0 && !loading && (
          <Box
            sx={{
              textAlign: 'center',
              py: 8,
              color: 'text.secondary',
            }}
          >
            <Typography variant="h6" gutterBottom>
              No templates found
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
                placeholder="Search templates..."
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

            {/* Right side: Template count, View mode */}
            <Box sx={{ display: 'flex', gap: 1, alignItems: 'center' }}>
              <Typography variant="body2" color="text.secondary">
                {processedTemplates.length} of {templates.length} templates
              </Typography>

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

          {/* Templates Grid */}
          {processedTemplates.length > 0 ? (
            <Grid container spacing={3}>
              {processedTemplates.map((template) => (
                <Grid item xs={12} sm={6} md={4} lg={3} key={template.id}>
                  <Card>
                    <CardContent>
                      <Typography variant="h6" gutterBottom>
                        {template.name}
                      </Typography>
                      <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
                        {template.description}
                      </Typography>
                      <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap', mb: 2 }}>
                        {template.tags.slice(0, 3).map((tag) => (
                          <Chip key={tag} label={tag} size="small" variant="outlined" />
                        ))}
                        {template.tags.length > 3 && (
                          <Chip
                            label={`+${template.tags.length - 3}`}
                            size="small"
                            variant="outlined"
                          />
                        )}
                      </Box>
                      <Typography variant="caption" color="text.secondary">
                        Version: {template.templateVersion || '1.0.0'} • Created:{' '}
                        {new Date(template.createdAt).toLocaleDateString()}
                      </Typography>
                    </CardContent>
                    <CardActions>
                      <Button size="small" onClick={() => handleUseTemplate(template)}>
                        Use Template
                      </Button>
                      <Button size="small" variant="outlined">
                        Preview
                      </Button>
                    </CardActions>
                  </Card>
                </Grid>
              ))}
            </Grid>
          ) : (
            <Box sx={{ textAlign: 'center', py: 8, color: 'text.secondary' }}>
              <Typography variant="h6" gutterBottom>
                No templates found
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

      {/* Create Project from Template Dialog */}
      <Dialog 
        open={dialogOpen} 
        onClose={handleDialogClose}
        maxWidth="sm"
        fullWidth
      >
        <DialogTitle>
          Create New Project from Template
        </DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 1 }}>
            {selectedTemplate && (
              <Box sx={{ mb: 3, p: 2, bgcolor: 'grey.50', borderRadius: 1 }}>
                <Typography variant="subtitle2" gutterBottom>
                  Template: {selectedTemplate.name}
                </Typography>
                <Typography variant="body2" color="text.secondary">
                  {selectedTemplate.description}
                </Typography>
              </Box>
            )}
            
            <FormControl fullWidth sx={{ mb: 2 }}>
              <FormLabel>Project Name *</FormLabel>
              <TextField
                value={newProjectName}
                onChange={(e) => setNewProjectName(e.target.value)}
                placeholder="Enter project name"
                size="small"
                error={!newProjectName.trim()}
                helperText={!newProjectName.trim() ? 'Project name is required' : ''}
              />
            </FormControl>

            <FormControl fullWidth>
              <FormLabel>Description</FormLabel>
              <TextField
                value={newProjectDescription}
                onChange={(e) => setNewProjectDescription(e.target.value)}
                placeholder="Enter project description"
                multiline
                rows={3}
                size="small"
              />
              <FormHelperText>
                Optional: Customize the project description
              </FormHelperText>
            </FormControl>
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleDialogClose} disabled={isCreating}>
            Cancel
          </Button>
          <Button 
            onClick={handleCreateProject} 
            variant="contained"
            disabled={!newProjectName.trim() || isCreating}
          >
            {isCreating ? 'Creating...' : 'Create Project'}
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default TemplatesPage;
