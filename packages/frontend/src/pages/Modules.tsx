import React, { useState, useEffect } from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
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
  Divider,
  useTheme,
  useMediaQuery,
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
  Add as AddIcon,
} from '@mui/icons-material';
import { RobotsService, Robot } from '../services/robotsService';
import { Link as RouterLink } from 'react-router-dom';

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
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('md'));
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
    const fetchModules = async () => {
      try {
        // Mock data for now
        const mockModules: ModuleMetadata[] = [
          {
            name: 'navigation_core',
            description: 'Core navigation functionality for autonomous robots',
            version: '1.0.0',
            category: 'navigation',
            packages: ['nav2_core', 'nav2_controller'],
            tags: ['autonomous', 'pathfinding'],
            supported_robots: ['turtlebot4', 'kobuki'],
          },
          {
            name: 'perception_vision',
            description: 'Computer vision and image processing modules',
            version: '2.1.0',
            category: 'perception',
            packages: ['opencv_ros', 'vision_msgs'],
            tags: ['vision', 'detection'],
            supported_robots: ['turtlebot4', 'depthai_oakd'],
          },
          {
            name: 'manipulation_arm',
            description: 'Robotic arm manipulation and control',
            version: '1.5.0',
            category: 'manipulation',
            packages: ['moveit', 'ur5_control'],
            tags: ['arm', 'grasping'],
            supported_robots: ['ur5', 'franka'],
          },
        ];
        setModules(mockModules);
        setFilteredModules(mockModules);
        setLoading(false);
      } catch (error) {
        console.error('Error fetching modules:', error);
        setLoading(false);
      }
    };

    const fetchRobots = async () => {
      try {
        const robotsData = await RobotsService.getRobots();
        setRobots(robotsData);
      } catch (error) {
        console.error('Error fetching robots:', error);
      }
    };

    fetchModules();
    fetchRobots();
  }, []);

  // Filter modules based on search, category, and robots
  useEffect(() => {
    let filtered = modules.filter((module) => {
      const matchesSearch =
        module.name.toLowerCase().includes(filters.search.toLowerCase()) ||
        module.description.toLowerCase().includes(filters.search.toLowerCase());

      const matchesCategory =
        filters.category === 'all' || module.category === filters.category;

      const matchesRobots =
        filters.robots.length === 0 ||
        (module.supported_robots &&
          module.supported_robots.some((robot) =>
            filters.robots.includes(robot)
          ));

      return matchesSearch && matchesCategory && matchesRobots;
    });

    // Sort modules
    filtered.sort((a, b) => {
      let comparison = 0;
      switch (filters.sortBy) {
        case 'name':
          comparison = a.name.localeCompare(b.name);
          break;
        case 'category':
          comparison = a.category.localeCompare(b.category);
          break;
        case 'complexity':
          comparison = (a.packages.length || 0) - (b.packages.length || 0);
          break;
        case 'status':
          comparison = (a.version || '').localeCompare(b.version || '');
          break;
        default:
          comparison = 0;
      }
      return filters.sortOrder === 'asc' ? comparison : -comparison;
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
      {/* Mobile controls keep current toolbar; Desktop uses left sidebar */}
      {isMobile ? (
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
            Showing {displayedModules.length} of {filteredModules.length}{' '}
            modules
          </Typography>

          {/* Modules Grid/List for Mobile */}
          <Box
            sx={{
              display: viewMode === 'grid' ? 'grid' : 'block',
              gridTemplateColumns:
                viewMode === 'grid'
                  ? {
                      xs: '1fr',
                      sm: 'repeat(2, 1fr)',
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
                      sx={{
                        display: 'flex',
                        gap: 1,
                        flexWrap: 'wrap',
                        mb: 2,
                      }}
                    >
                      <Chip
                        label={module.category}
                        size="small"
                        sx={{
                          backgroundColor: getCategoryColor(module.category),
                          color: 'white',
                        }}
                      />
                      {module.tags?.map((tag) => (
                        <Chip
                          key={tag}
                          label={tag}
                          size="small"
                          variant="outlined"
                        />
                      ))}
                    </Box>

                    <Box
                      sx={{
                        display: 'flex',
                        justifyContent: 'space-between',
                        alignItems: 'center',
                      }}
                    >
                      <Typography variant="caption" color="text.secondary">
                        Version: {module.version}
                      </Typography>
                    </Box>
                  </CardContent>
                </Card>
              </Box>
            ))}
          </Box>
        </Box>
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


            {/* Categories */}
            <Box sx={{ mb: 2 }}>
              <Typography variant="subtitle2" sx={{ mb: 1, fontWeight: 600 }}>
                Categories
              </Typography>
              <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                <Chip
                  label="All"
                  clickable
                  color={filters.category === 'all' ? 'primary' : 'default'}
                  variant={filters.category === 'all' ? 'filled' : 'outlined'}
                  onClick={() =>
                    setFilters((prev) => ({ ...prev, category: 'all' }))
                  }
                />
                {categories.map((category) => (
                  <Chip
                    key={category}
                    label={category}
                    clickable
                    color={
                      filters.category === category ? 'primary' : 'default'
                    }
                    variant={
                      filters.category === category ? 'filled' : 'outlined'
                    }
                    onClick={() =>
                      setFilters((prev) => ({ ...prev, category }))
                    }
                  />
                ))}
              </Box>
            </Box>

            <Divider sx={{ my: 2 }} />

            {/* Robots */}
            <Box sx={{ mb: 2 }}>
              <Typography variant="subtitle2" sx={{ mb: 1, fontWeight: 600 }}>
                Supported Robots
              </Typography>
              <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                {robots.map((robot) => {
                  const selected = filters.robots.includes(robot.code);
                  return (
                    <Chip
                      key={robot.code}
                      label={robot.name}
                      clickable
                      color={selected ? 'primary' : 'default'}
                      variant={selected ? 'filled' : 'outlined'}
                      onClick={() => {
                        const exists = filters.robots.includes(robot.code);
                        const next = exists
                          ? filters.robots.filter((r) => r !== robot.code)
                          : [...filters.robots, robot.code];
                        setFilters((prev) => ({ ...prev, robots: next }));
                      }}
                      size="small"
                    />
                  );
                })}
              </Box>
            </Box>

            <Divider sx={{ my: 2 }} />

            {/* Sorting */}
            <Box sx={{ mb: 2 }}>
              <Typography variant="subtitle2" sx={{ mb: 1, fontWeight: 600 }}>
                Sort
              </Typography>
              <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                {['name', 'category', 'complexity', 'status'].map((opt) => (
                  <Chip
                    key={opt}
                    label={opt.charAt(0).toUpperCase() + opt.slice(1)}
                    clickable
                    color={filters.sortBy === opt ? 'primary' : 'default'}
                    variant={filters.sortBy === opt ? 'filled' : 'outlined'}
                    onClick={() =>
                      setFilters((prev) => ({ ...prev, sortBy: opt }))
                    }
                    size="small"
                  />
                ))}
                <Chip
                  label={filters.sortOrder === 'asc' ? 'Asc' : 'Desc'}
                  clickable
                  onClick={() =>
                    setFilters((prev) => ({
                      ...prev,
                      sortOrder: prev.sortOrder === 'asc' ? 'desc' : 'asc',
                    }))
                  }
                  size="small"
                />
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
                  size="small"
                  sx={{ minWidth: 200 }}
                />

                <FormControl size="small" sx={{ minWidth: 150 }}>
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
                    <MenuItem value="all">All Categories</MenuItem>
                    {categories.map((category) => (
                      <MenuItem key={category} value={category}>
                        {category.charAt(0).toUpperCase() + category.slice(1)}
                      </MenuItem>
                    ))}
                  </Select>
                </FormControl>

                <FormControl size="small" sx={{ minWidth: 150 }}>
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
                    value &&
                    setFilters((prev) => ({ ...prev, sortOrder: value }))
                  }
                  size="small"
                >
                  <ToggleButton value="asc">↑</ToggleButton>
                  <ToggleButton value="desc">↓</ToggleButton>
                </ToggleButtonGroup>
              </Box>

              {/* Right side: Module count, View mode */}
              <Box sx={{ display: 'flex', gap: 1, alignItems: 'center' }}>
                <Typography variant="body2" color="text.secondary">
                  {displayedModules.length} of {filteredModules.length} modules
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
                        sx={{
                          display: 'flex',
                          alignItems: 'center',
                          mb: 1,
                          gap: 1,
                        }}
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

                      <Box
                        sx={{ display: 'flex', alignItems: 'center', mb: 1 }}
                      >
                        <PackageIcon
                          sx={{
                            fontSize: 16,
                            mr: 0.5,
                            color: 'text.secondary',
                          }}
                        />
                        <Typography variant="caption" color="text.secondary">
                          {module.packages.length} packages
                        </Typography>
                      </Box>

                      {module.dependencies &&
                        module.dependencies.length > 0 && (
                          <Box
                            sx={{
                              display: 'flex',
                              alignItems: 'center',
                              mb: 1,
                            }}
                          >
                            <DependencyIcon
                              sx={{
                                fontSize: 16,
                                mr: 0.5,
                                color: 'text.secondary',
                              }}
                            />
                            <Typography
                              variant="caption"
                              color="text.secondary"
                            >
                              {module.dependencies.length} dependencies
                            </Typography>
                          </Box>
                        )}

                      <Box
                        sx={{
                          display: 'flex',
                          alignItems: 'center',
                          flexWrap: 'wrap',
                          gap: 0.5,
                          mb: 1,
                        }}
                      >
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
          </Box>
        </Box>
      )}

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
                <Box sx={{ mb: 2 }}>
                  <Typography variant="h6" gutterBottom>
                    <TagIcon
                      sx={{ fontSize: 20, mr: 1, verticalAlign: 'middle' }}
                    />
                    Tags ({selectedModule.tags.length})
                  </Typography>
                  <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
                    {selectedModule.tags.map((tag) => (
                      <Chip
                        key={tag}
                        label={tag}
                        size="small"
                        variant="outlined"
                      />
                    ))}
                  </Box>
                </Box>
              )}

              {selectedModule.supported_robots &&
                selectedModule.supported_robots.length > 0 && (
                  <Box sx={{ mb: 2 }}>
                    <Typography variant="h6" gutterBottom>
                      Supported Robots ({selectedModule.supported_robots.length}
                      )
                    </Typography>
                    <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
                      {selectedModule.supported_robots.map((robot) => (
                        <Chip
                          key={robot}
                          label={robot.replace(/_/g, ' ')}
                          size="small"
                          variant="outlined"
                        />
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

      {/* Compare Modules Dialog */}
      <Dialog
        open={compareDialogOpen}
        onClose={() => setCompareDialogOpen(false)}
        maxWidth="lg"
        fullWidth
      >
        <DialogTitle>Compare Modules</DialogTitle>
        <DialogContent>
          <Box
            sx={{
              display: 'grid',
              gridTemplateColumns: {
                xs: '1fr',
                md: 'repeat(2, 1fr)',
              },
              gap: 2,
            }}
          >
            {compareModules.map((module) => (
              <Box key={module.name}>
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

                    <Box sx={{ mb: 2 }}>
                      <Typography variant="subtitle2" gutterBottom>
                        Packages ({module.packages.length})
                      </Typography>
                      <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 0.5 }}>
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

                    {module.dependencies && module.dependencies.length > 0 && (
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
                              label={`+${module.dependencies.length - 3} more`}
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
              </Box>
            ))}
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
