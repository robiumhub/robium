import React, { useState, useEffect } from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
  Grid,
  Chip,
  IconButton,
  Tooltip,
  TextField,
  InputAdornment,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Button,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  List,
  ListItem,
  ListItemText,
  Pagination,
  Stack,
  Checkbox,
  ToggleButton,
  ToggleButtonGroup,
  Alert,
} from '@mui/material';
import {
  Search as SearchIcon,
  FilterList as FilterIcon,
  Info as InfoIcon,
  Category as CategoryIcon,
  Tag as TagIcon,
  Inventory as PackageIcon,
  AccountTree as DependencyIcon,
  Compare as CompareIcon,
  ViewList as ViewListIcon,
  ViewModule as ViewModuleIcon,
} from '@mui/icons-material';
import { RobotsService, Robot } from '../services/robotsService';

interface ModuleMetadata {
  name: string;
  description: string;
  version: string;
  category: string;
  packages: string[];
  dependencies?: string[];
  tags?: string[];
  supported_robots?: string[];
  parameters?: Record<string, unknown>;
}

interface ModuleFilters {
  search: string;
  category: string;
  robots: string[];
  sortBy: string;
  sortOrder: 'asc' | 'desc';
}

const Modules: React.FC = () => {
  const [modules, setModules] = useState<ModuleMetadata[]>([]);
  const [filteredModules, setFilteredModules] = useState<ModuleMetadata[]>([]);
  const [displayedModules, setDisplayedModules] = useState<ModuleMetadata[]>(
    []
  );
  const [selectedModules, setSelectedModules] = useState<Set<string>>(
    new Set()
  );
  const [compareModules, setCompareModules] = useState<ModuleMetadata[]>([]);
  const [viewMode, setViewMode] = useState<'grid' | 'list'>('grid');
  const [showFilters, setShowFilters] = useState(false);
  const [selectedModule, setSelectedModule] = useState<ModuleMetadata | null>(
    null
  );
  const [dialogOpen, setDialogOpen] = useState(false);
  const [compareDialogOpen, setCompareDialogOpen] = useState(false);
  const [loading, setLoading] = useState(true);
  const [currentPage, setCurrentPage] = useState(1);
  const [robots, setRobots] = useState<Robot[]>([]);
  const [filters, setFilters] = useState<ModuleFilters>({
    search: '',
    category: 'all',
    robots: [],
    sortBy: 'name',
    sortOrder: 'asc',
  });
  const modulesPerPage = 12;

  useEffect(() => {
    // Fetch modules and robots from API
    const fetchData = async () => {
      try {
        setLoading(true);

        // Fetch robots data
        const robotsData = await RobotsService.getRobots();
        setRobots(robotsData);

        // Fetch modules data
        const response = await fetch('http://localhost:8000/modules', {
          headers: {
            'Content-Type': 'application/json',
          },
        });

        if (response.ok) {
          const result = await response.json();
          if (result.success) {
            // Handle both old and new API response formats
            const modulesData = Array.isArray(result.data)
              ? result.data
              : result.data.modules;
            setModules(modulesData);
            setFilteredModules(modulesData);
          } else {
            console.error('Failed to fetch modules:', result.error);
          }
        } else {
          console.error('Failed to fetch modules:', response.statusText);
        }
      } catch (error) {
        console.error('Error fetching data:', error);
      } finally {
        setLoading(false);
      }
    };

    fetchData();
  }, []);

  useEffect(() => {
    let filtered = modules;

    // Apply search filter
    if (filters.search) {
      filtered = filtered.filter(
        (module) =>
          module.name.toLowerCase().includes(filters.search.toLowerCase()) ||
          module.description
            .toLowerCase()
            .includes(filters.search.toLowerCase()) ||
          module.tags?.some((tag) =>
            tag.toLowerCase().includes(filters.search.toLowerCase())
          )
      );
    }

    // Apply category filter
    if (filters.category !== 'all') {
      filtered = filtered.filter(
        (module) => module.category === filters.category
      );
    }

    // Apply robots filter (intersection)
    if (filters.robots.length > 0) {
      filtered = filtered.filter((module) => {
        const robots = module.supported_robots || [];
        return filters.robots.every((r) => robots.includes(r));
      });
    }

    // Apply sorting
    filtered.sort((a, b) => {
      let aValue: any, bValue: any;

      switch (filters.sortBy) {
        case 'name':
          aValue = a.name.toLowerCase();
          bValue = b.name.toLowerCase();
          break;
        case 'category':
          aValue = a.category.toLowerCase();
          bValue = b.category.toLowerCase();
          break;
        default:
          aValue = a.name.toLowerCase();
          bValue = b.name.toLowerCase();
      }

      if (filters.sortOrder === 'asc') {
        return aValue.localeCompare(bValue);
      } else {
        return bValue.localeCompare(aValue);
      }
    });

    setFilteredModules(filtered);
    setCurrentPage(1); // Reset to first page when filters change
  }, [modules, filters]);

  // Calculate pagination
  useEffect(() => {
    const startIndex = (currentPage - 1) * modulesPerPage;
    const endIndex = startIndex + modulesPerPage;
    setDisplayedModules(filteredModules.slice(startIndex, endIndex));
  }, [filteredModules, currentPage]);

  const totalPages = Math.ceil(filteredModules.length / modulesPerPage);

  const handlePageChange = (
    event: React.ChangeEvent<unknown>,
    value: number
  ) => {
    setCurrentPage(value);
  };

  const handleModuleClick = (module: ModuleMetadata) => {
    setSelectedModule(module);
    setDialogOpen(true);
  };

  const handleCloseDialog = () => {
    setDialogOpen(false);
    setSelectedModule(null);
  };

  const getCategoryColor = (category: string) => {
    const colors: { [key: string]: string } = {
      navigation: '#2196f3',
      perception: '#4caf50',
      manipulation: '#ff9800',
      control: '#9c27b0',
      communication: '#f44336',
      simulation: '#00bcd4',
      safety: '#e91e63',
      ai: '#673ab7',
    };
    return colors[category] || '#757575';
  };

  const categories = Array.from(new Set(modules.map((m) => m.category)));

  if (loading) {
    return (
      <Box
        sx={{
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          minHeight: '400px',
        }}
      >
        <Typography>Loading modules...</Typography>
      </Box>
    );
  }

  return (
    <Box>
      <Typography variant="h4" component="h1" gutterBottom>
        Modules
      </Typography>
      <Typography variant="body1" color="text.secondary" gutterBottom>
        Browse and manage available robotics modules for your projects
      </Typography>

      {/* Enhanced Search and Filter Controls */}
      <Box sx={{ mb: 3 }}>
        <Box
          sx={{
            display: 'flex',
            gap: 2,
            flexWrap: 'wrap',
            alignItems: 'center',
            mb: 2,
          }}
        >
          <TextField
            placeholder="Search modules..."
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
            sx={{ minWidth: 300 }}
          />
          <FormControl sx={{ minWidth: 150 }}>
            <InputLabel>Category</InputLabel>
            <Select
              value={filters.category}
              label="Category"
              onChange={(e) =>
                setFilters((prev) => ({ ...prev, category: e.target.value }))
              }
            >
              <MenuItem value="all">All Categories</MenuItem>
              {categories.map((category) => (
                <MenuItem key={category} value={category}>
                  {category.charAt(0).toUpperCase() + category.slice(1)}
                </MenuItem>
              ))}
            </Select>
          </FormControl>
          <FormControl sx={{ minWidth: 220 }}>
            <InputLabel>Supported Robots</InputLabel>
            <Select
              multiple
              value={filters.robots}
              label="Supported Robots"
              onChange={(e) =>
                setFilters((prev) => ({
                  ...prev,
                  robots: e.target.value as string[],
                }))
              }
              renderValue={(selected) =>
                (selected as string[])
                  .map((r) => RobotsService.getRobotDisplayName(r))
                  .join(', ')
              }
            >
              {robots.map((robot) => (
                <MenuItem key={robot.code} value={robot.code}>
                  {robot.name} ({robot.module_count} modules)
                </MenuItem>
              ))}
            </Select>
          </FormControl>
          <FormControl sx={{ minWidth: 150 }}>
            <InputLabel>Sort By</InputLabel>
            <Select
              value={filters.sortBy}
              label="Sort By"
              onChange={(e) =>
                setFilters((prev) => ({ ...prev, sortBy: e.target.value }))
              }
            >
              <MenuItem value="name">Name</MenuItem>
              <MenuItem value="category">Category</MenuItem>
              <MenuItem value="complexity">Complexity</MenuItem>
              <MenuItem value="status">Status</MenuItem>
            </Select>
          </FormControl>
          <ToggleButtonGroup
            value={filters.sortOrder}
            exclusive
            onChange={(_, value) =>
              value && setFilters((prev) => ({ ...prev, sortOrder: value }))
            }
            size="small"
          >
            <ToggleButton value="asc">↑</ToggleButton>
            <ToggleButton value="desc">↓</ToggleButton>
          </ToggleButtonGroup>
          <ToggleButtonGroup
            value={viewMode}
            exclusive
            onChange={(_, value) => value && setViewMode(value)}
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
            variant="outlined"
            startIcon={<FilterIcon />}
            onClick={() => setShowFilters(!showFilters)}
          >
            {showFilters ? 'Hide' : 'Show'} Filters
          </Button>
        </Box>

        {/* Selection Controls */}
        {selectedModules.size > 0 && (
          <Alert severity="info" sx={{ mb: 2 }}>
            <Box
              sx={{
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'space-between',
              }}
            >
              <Typography>
                {selectedModules.size} module
                {selectedModules.size !== 1 ? 's' : ''} selected
              </Typography>
              <Box sx={{ display: 'flex', gap: 1 }}>
                <Button
                  size="small"
                  startIcon={<CompareIcon />}
                  onClick={() => {
                    const selectedModuleList = modules.filter((m) =>
                      selectedModules.has(m.name)
                    );
                    setCompareModules(selectedModuleList);
                    setCompareDialogOpen(true);
                  }}
                  disabled={selectedModules.size < 2}
                >
                  Compare ({selectedModules.size})
                </Button>
                <Button
                  size="small"
                  variant="outlined"
                  onClick={() => setSelectedModules(new Set())}
                >
                  Clear Selection
                </Button>
              </Box>
            </Box>
          </Alert>
        )}

        <Typography variant="body2" color="text.secondary">
          Showing {displayedModules.length} of {filteredModules.length} modules
        </Typography>
      </Box>

      {/* Modules Grid/List */}
      <Box
        sx={{
          display: viewMode === 'grid' ? 'grid' : 'block',
          gridTemplateColumns:
            viewMode === 'grid'
              ? {
                  xs: '1fr',
                  sm: 'repeat(2, 1fr)',
                  md: 'repeat(3, 1fr)',
                  lg: 'repeat(4, 1fr)',
                }
              : '1fr',
          gap: 3,
        }}
      >
        {displayedModules.map((module) => (
          <Box key={module.name}>
            <Card
              sx={{
                height: viewMode === 'grid' ? '100%' : 'auto',
                display: 'flex',
                flexDirection: 'column',
                cursor: 'pointer',
                border: selectedModules.has(module.name) ? 2 : 1,
                borderColor: selectedModules.has(module.name)
                  ? 'primary.main'
                  : 'divider',
                '&:hover': {
                  boxShadow: 4,
                  transform: selectedModules.has(module.name)
                    ? 'none'
                    : 'translateY(-2px)',
                  transition: 'all 0.2s ease-in-out',
                },
              }}
              onClick={() => handleModuleClick(module)}
            >
              <CardContent sx={{ flexGrow: 1 }}>
                <Box
                  sx={{
                    display: 'flex',
                    justifyContent: 'space-between',
                    alignItems: 'flex-start',
                    mb: 2,
                  }}
                >
                  <Box
                    sx={{
                      display: 'flex',
                      alignItems: 'center',
                      gap: 1,
                      flex: 1,
                    }}
                  >
                    <Checkbox
                      checked={selectedModules.has(module.name)}
                      onChange={(e) => {
                        e.stopPropagation();
                        const newSelected = new Set(selectedModules);
                        if (newSelected.has(module.name)) {
                          newSelected.delete(module.name);
                        } else {
                          newSelected.add(module.name);
                        }
                        setSelectedModules(newSelected);
                      }}
                      size="small"
                    />
                    <Typography variant="h6" component="h2" gutterBottom>
                      {module.name
                        .replace(/_/g, ' ')
                        .replace(/\b\w/g, (l) => l.toUpperCase())}
                    </Typography>
                  </Box>
                  <Tooltip title="Module Details">
                    <IconButton size="small">
                      <InfoIcon />
                    </IconButton>
                  </Tooltip>
                </Box>

                <Typography
                  variant="body2"
                  color="text.secondary"
                  sx={{ mb: 2 }}
                >
                  {module.description}
                </Typography>

                <Box
                  sx={{ display: 'flex', alignItems: 'center', mb: 1, gap: 1 }}
                >
                  <CategoryIcon
                    sx={{ fontSize: 16, color: 'text.secondary' }}
                  />
                  <Chip
                    label={module.category}
                    size="small"
                    sx={{
                      backgroundColor: getCategoryColor(module.category),
                      color: 'white',
                      fontSize: '0.75rem',
                    }}
                  />
                </Box>

                <Box
                  sx={{
                    display: 'flex',
                    alignItems: 'center',
                    flexWrap: 'wrap',
                    gap: 0.5,
                    mb: 1,
                  }}
                >
                  {(module.supported_robots || []).map((r) => (
                    <Chip
                      key={r}
                      label={r.replace(/_/g, ' ')}
                      size="small"
                      variant="outlined"
                    />
                  ))}
                </Box>

                <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
                  <PackageIcon
                    sx={{ fontSize: 16, mr: 0.5, color: 'text.secondary' }}
                  />
                  <Typography variant="caption" color="text.secondary">
                    {module.packages.length} packages
                  </Typography>
                </Box>

                {module.dependencies && module.dependencies.length > 0 && (
                  <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
                    <DependencyIcon
                      sx={{ fontSize: 16, mr: 0.5, color: 'text.secondary' }}
                    />
                    <Typography variant="caption" color="text.secondary">
                      {module.dependencies.length} dependencies
                    </Typography>
                  </Box>
                )}

                <Box sx={{ mt: 2 }}>
                  {module.tags?.slice(0, 3).map((tag) => (
                    <Chip
                      key={tag}
                      label={tag}
                      size="small"
                      variant="outlined"
                      sx={{ mr: 0.5, mb: 0.5 }}
                    />
                  ))}
                  {module.tags && module.tags.length > 3 && (
                    <Chip
                      label={`+${module.tags.length - 3} more`}
                      size="small"
                      variant="outlined"
                      sx={{ mr: 0.5, mb: 0.5 }}
                    />
                  )}
                </Box>

                <Typography
                  variant="caption"
                  color="text.secondary"
                  sx={{ mt: 2, display: 'block' }}
                >
                  Version: {module.version}
                </Typography>
              </CardContent>
            </Card>
          </Box>
        ))}
      </Box>

      {/* Pagination */}
      {totalPages > 1 && (
        <Box sx={{ mt: 4, display: 'flex', justifyContent: 'center' }}>
          <Stack spacing={2}>
            <Pagination
              count={totalPages}
              page={currentPage}
              onChange={handlePageChange}
              color="primary"
              size="large"
              showFirstButton
              showLastButton
            />
            <Typography
              variant="body2"
              color="text.secondary"
              textAlign="center"
            >
              Page {currentPage} of {totalPages}
            </Typography>
          </Stack>
        </Box>
      )}

      {/* Module Details Dialog */}
      <Dialog
        open={dialogOpen}
        onClose={handleCloseDialog}
        maxWidth="md"
        fullWidth
      >
        {selectedModule && (
          <>
            <DialogTitle>
              <Typography variant="h5">
                {selectedModule.name
                  .replace(/_/g, ' ')
                  .replace(/\b\w/g, (l) => l.toUpperCase())}
              </Typography>
              <Typography variant="body2" color="text.secondary">
                Version {selectedModule.version}
              </Typography>
            </DialogTitle>
            <DialogContent>
              <Typography variant="body1" sx={{ mb: 2 }}>
                {selectedModule.description}
              </Typography>

              <Box sx={{ mb: 2 }}>
                <Typography variant="h6" gutterBottom>
                  <CategoryIcon
                    sx={{ fontSize: 20, mr: 1, verticalAlign: 'middle' }}
                  />
                  Category
                </Typography>
                <Chip
                  label={selectedModule.category}
                  sx={{
                    backgroundColor: getCategoryColor(selectedModule.category),
                    color: 'white',
                  }}
                />
              </Box>

              <Box sx={{ mb: 2 }}>
                <Typography variant="h6" gutterBottom>
                  <PackageIcon
                    sx={{ fontSize: 20, mr: 1, verticalAlign: 'middle' }}
                  />
                  ROS Packages ({selectedModule.packages.length})
                </Typography>
                <List dense>
                  {selectedModule.packages.map((pkg) => (
                    <ListItem key={pkg}>
                      <ListItemText primary={pkg} />
                    </ListItem>
                  ))}
                </List>
              </Box>

              {selectedModule.dependencies &&
                selectedModule.dependencies.length > 0 && (
                  <Box sx={{ mb: 2 }}>
                    <Typography variant="h6" gutterBottom>
                      <DependencyIcon
                        sx={{ fontSize: 20, mr: 1, verticalAlign: 'middle' }}
                      />
                      Dependencies ({selectedModule.dependencies.length})
                    </Typography>
                    <List dense>
                      {selectedModule.dependencies.map((dep) => (
                        <ListItem key={dep}>
                          <ListItemText primary={dep} />
                        </ListItem>
                      ))}
                    </List>
                  </Box>
                )}

              {selectedModule.tags && selectedModule.tags.length > 0 && (
                <Box>
                  <Typography variant="h6" gutterBottom>
                    <TagIcon
                      sx={{ fontSize: 20, mr: 1, verticalAlign: 'middle' }}
                    />
                    Tags
                  </Typography>
                  <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
                    {selectedModule.tags.map((tag) => (
                      <Chip key={tag} label={tag} size="small" />
                    ))}
                  </Box>
                </Box>
              )}
            </DialogContent>
            <DialogActions>
              <Button onClick={handleCloseDialog}>Close</Button>
              <Button variant="contained" color="primary">
                Add to Project
              </Button>
            </DialogActions>
          </>
        )}
      </Dialog>

      {/* Module Comparison Dialog */}
      <Dialog
        open={compareDialogOpen}
        onClose={() => setCompareDialogOpen(false)}
        maxWidth="lg"
        fullWidth
      >
        <DialogTitle>
          <Typography variant="h5">Module Comparison</Typography>
          <Typography variant="body2" color="text.secondary">
            Compare {compareModules.length} modules
          </Typography>
        </DialogTitle>
        <DialogContent>
          <Box sx={{ overflow: 'auto' }}>
            <Grid container spacing={2}>
              {compareModules.map((module) => (
                <Grid item xs={12} md={6} lg={4} key={module.name}>
                  <Card>
                    <CardContent>
                      <Typography variant="h6" gutterBottom>
                        {module.name
                          .replace(/_/g, ' ')
                          .replace(/\b\w/g, (l) => l.toUpperCase())}
                      </Typography>
                      <Typography
                        variant="body2"
                        color="text.secondary"
                        sx={{ mb: 2 }}
                      >
                        {module.description}
                      </Typography>

                      <Box sx={{ mb: 2 }}>
                        <Typography variant="subtitle2" gutterBottom>
                          Category
                        </Typography>
                        <Chip
                          label={module.category}
                          size="small"
                          sx={{
                            backgroundColor: getCategoryColor(module.category),
                            color: 'white',
                          }}
                        />
                      </Box>

                      {module.complexity && (
                        <Box sx={{ mb: 2 }}>
                          <Typography variant="subtitle2" gutterBottom>
                            Complexity
                          </Typography>
                          <Chip
                            label={module.complexity}
                            size="small"
                            variant="outlined"
                          />
                        </Box>
                      )}

                      {module.status && (
                        <Box sx={{ mb: 2 }}>
                          <Typography variant="subtitle2" gutterBottom>
                            Status
                          </Typography>
                          <Chip
                            label={module.status}
                            size="small"
                            variant="outlined"
                            color={
                              module.status === 'stable'
                                ? 'success'
                                : module.status === 'deprecated'
                                ? 'error'
                                : 'warning'
                            }
                          />
                        </Box>
                      )}

                      <Box sx={{ mb: 2 }}>
                        <Typography variant="subtitle2" gutterBottom>
                          Packages ({module.packages.length})
                        </Typography>
                        <Box
                          sx={{ display: 'flex', flexWrap: 'wrap', gap: 0.5 }}
                        >
                          {module.packages.slice(0, 3).map((pkg) => (
                            <Chip
                              key={pkg}
                              label={pkg}
                              size="small"
                              variant="outlined"
                            />
                          ))}
                          {module.packages.length > 3 && (
                            <Chip
                              label={`+${module.packages.length - 3} more`}
                              size="small"
                              variant="outlined"
                            />
                          )}
                        </Box>
                      </Box>

                      {module.dependencies &&
                        module.dependencies.length > 0 && (
                          <Box sx={{ mb: 2 }}>
                            <Typography variant="subtitle2" gutterBottom>
                              Dependencies ({module.dependencies.length})
                            </Typography>
                            <Box
                              sx={{
                                display: 'flex',
                                flexWrap: 'wrap',
                                gap: 0.5,
                              }}
                            >
                              {module.dependencies.slice(0, 3).map((dep) => (
                                <Chip
                                  key={dep}
                                  label={dep}
                                  size="small"
                                  variant="outlined"
                                />
                              ))}
                              {module.dependencies.length > 3 && (
                                <Chip
                                  label={`+${
                                    module.dependencies.length - 3
                                  } more`}
                                  size="small"
                                  variant="outlined"
                                />
                              )}
                            </Box>
                          </Box>
                        )}

                      <Typography variant="caption" color="text.secondary">
                        Version: {module.version}
                      </Typography>
                    </CardContent>
                  </Card>
                </Grid>
              ))}
            </Grid>
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setCompareDialogOpen(false)}>Close</Button>
          <Button variant="contained" color="primary">
            Add Selected to Project
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default Modules;
