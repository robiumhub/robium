import React, { useState, useEffect, useMemo } from 'react';
import {
  Box,
  Typography,
  Grid,
  TextField,
  InputAdornment,
  IconButton,
  Button,
  Chip,
  Tooltip,
  useTheme,
  useMediaQuery,
  Tabs,
  Tab,
  CircularProgress,
  Alert,
  Paper,
  Drawer,
  ToggleButtonGroup,
  ToggleButton,
  MenuItem,
} from '@mui/material';
import {
  Search as SearchIcon,
  Clear as ClearIcon,
  GridView as GridIcon,
  ViewList as ListIcon,
  TrendingUp as TrendingIcon,
  Star as StarIcon,
  FilterList as FilterListIcon,
  NewReleases as NewIcon,
  Add as AddIcon,
} from '@mui/icons-material';
import { useNavigate, Link as RouterLink } from 'react-router-dom';
import TemplateCard from '../components/TemplateCard';
import TemplateFilters from '../components/TemplateFilters';
import TemplatePreview from '../components/TemplatePreview';
import {
  Template,
  TemplateFilters as TemplateFiltersType,
  DEFAULT_TEMPLATE_FILTERS,
  TemplateStats,
} from '../types/template';
import { ApiService } from '../services/api';

type SortOption =
  | 'name'
  | 'created_at'
  | 'updated_at'
  | 'rating'
  | 'installs_7d';
type ViewMode = 'grid' | 'list';
type FilterTab =
  | 'main'
  | 'use_cases'
  | 'capabilities'
  | 'targets'
  | 'simulators'
  | 'other';

const Templates: React.FC = () => {
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('md'));
  const navigate = useNavigate();

  // State
  const [templates, setTemplates] = useState<Template[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [searchQuery, setSearchQuery] = useState('');
  const [filters, setFilters] = useState<TemplateFiltersType>(
    DEFAULT_TEMPLATE_FILTERS
  );
  const [sortBy, setSortBy] = useState<SortOption>('updated_at');
  const [sortOrder, setSortOrder] = useState<'asc' | 'desc'>('desc');
  const [viewMode, setViewMode] = useState<ViewMode>('grid');
  const [filtersOpen, setFiltersOpen] = useState(false);
  const [previewTemplate, setPreviewTemplate] = useState<Template | null>(null);
  const [previewOpen, setPreviewOpen] = useState(false);
  const [bookmarkedTemplates, setBookmarkedTemplates] = useState<Set<string>>(
    new Set()
  );
  const [stats, setStats] = useState<TemplateStats | undefined>(undefined);
  const [activeFilterTab, setActiveFilterTab] = useState<FilterTab>('main');

  // Load templates from backend
  useEffect(() => {
    const loadTemplates = async () => {
      try {
        setLoading(true);
        const data = await ApiService.get<any[]>('/projects/templates');

        // Transform the data to match our Template interface
        const transformedTemplates: Template[] = (
          Array.isArray(data) ? data : []
        ).map((item) => ({
          id: item.id,
          name: item.name,
          description: item.description,
          summary:
            item.description?.substring(0, 100) +
            (item.description?.length > 100 ? '...' : ''),
          tags: item.tags || [],
          category: item.category || '',
          type: item.type || 'custom',
          version: item.version || '1.0.0',
          author: item.author || 'Unknown',
          maintainer_email: item.maintainer_email || '',
          license: item.license || 'Apache-2.0',
          is_active: item.is_active ?? true,
          is_public: item.is_public ?? true,
          is_template: item.is_template ?? true,
          config: item.config || {},
          metadata: item.metadata || {},
          module_count: item.module_count || 0,
          package_count: item.package_count || 0,
          created_at: item.created_at,
          updated_at: item.updated_at,
          created_by: item.created_by || '',
          updated_by: item.updated_by || '',
          // Mock data for now - these would come from analytics
          installs_7d: Math.floor(Math.random() * 1000),
          rating: 4.0 + Math.random() * 1.0,
          rating_count: Math.floor(Math.random() * 100),
          is_official: item.author === 'Robium Team',
          is_verified: Math.random() > 0.5,
          requires_gpu:
            item.config?.simulation === 'isaac' ||
            item.config?.deployment === 'cloud_gpu',
          is_new:
            new Date(item.created_at) >
            new Date(Date.now() - 7 * 24 * 60 * 60 * 1000),
        }));

        setTemplates(transformedTemplates);

        // Generate mock stats
        const mockStats: TemplateStats = {
          total_templates: transformedTemplates.length,
          use_case_counts: {},
          capability_counts: {},
          robot_target_counts: {},
          simulator_counts: {},
          ros_distro_counts: {},
          rmw_counts: {},
          tag_counts: {},
          license_counts: {},
        };

        // Populate stats from templates
        transformedTemplates.forEach((template) => {
          // Count tags
          template.tags.forEach((tag) => {
            mockStats.tag_counts[tag] = (mockStats.tag_counts[tag] || 0) + 1;
          });

          // Count licenses
          mockStats.license_counts[template.license] =
            (mockStats.license_counts[template.license] || 0) + 1;

          // Count metadata fields
          if (template.metadata?.use_cases) {
            template.metadata.use_cases.forEach((useCase) => {
              mockStats.use_case_counts[useCase] =
                (mockStats.use_case_counts[useCase] || 0) + 1;
            });
          }

          if (template.metadata?.capabilities) {
            template.metadata.capabilities.forEach((capability) => {
              mockStats.capability_counts[capability] =
                (mockStats.capability_counts[capability] || 0) + 1;
            });
          }

          if (template.metadata?.robot_targets) {
            template.metadata.robot_targets.forEach((target) => {
              mockStats.robot_target_counts[target] =
                (mockStats.robot_target_counts[target] || 0) + 1;
            });
          }

          if (template.metadata?.simulators) {
            template.metadata.simulators.forEach((simulator) => {
              mockStats.simulator_counts[simulator] =
                (mockStats.simulator_counts[simulator] || 0) + 1;
            });
          }

          if (template.metadata?.ros_distros) {
            template.metadata.ros_distros.forEach((distro) => {
              mockStats.ros_distro_counts[distro] =
                (mockStats.ros_distro_counts[distro] || 0) + 1;
            });
          }

          if (template.metadata?.rmw_implementations) {
            template.metadata.rmw_implementations.forEach((rmw) => {
              mockStats.rmw_counts[rmw] = (mockStats.rmw_counts[rmw] || 0) + 1;
            });
          }
        });

        setStats(mockStats);
      } catch (err) {
        setError('Failed to load templates');
        console.error('Error loading templates:', err);
      } finally {
        setLoading(false);
      }
    };

    loadTemplates();
  }, []);

  // Filter and sort templates
  const processedTemplates = useMemo(() => {
    let filtered = templates;

    // Apply search filter
    if (searchQuery) {
      const query = searchQuery.toLowerCase();
      filtered = filtered.filter(
        (template) =>
          template.name.toLowerCase().includes(query) ||
          template.description.toLowerCase().includes(query) ||
          template.tags.some((tag) => tag.toLowerCase().includes(query)) ||
          template.author.toLowerCase().includes(query)
      );
    }

    // Apply filters
    if (filters.use_cases.length > 0) {
      filtered = filtered.filter((template) =>
        template.metadata?.use_cases?.some((useCase) =>
          filters.use_cases.includes(useCase)
        )
      );
    }

    if (filters.capabilities.length > 0) {
      filtered = filtered.filter((template) =>
        template.metadata?.capabilities?.some((capability) =>
          filters.capabilities.includes(capability)
        )
      );
    }

    if (filters.robot_targets.length > 0) {
      filtered = filtered.filter((template) =>
        template.metadata?.robot_targets?.some((target) =>
          filters.robot_targets.includes(target)
        )
      );
    }

    if (filters.simulators.length > 0) {
      filtered = filtered.filter((template) =>
        template.metadata?.simulators?.some((simulator) =>
          filters.simulators.includes(simulator)
        )
      );
    }

    if (filters.ros_distros.length > 0) {
      filtered = filtered.filter((template) =>
        template.metadata?.ros_distros?.some((distro) =>
          filters.ros_distros.includes(distro)
        )
      );
    }

    if (filters.rmw_implementations.length > 0) {
      filtered = filtered.filter((template) =>
        template.metadata?.rmw_implementations?.some((rmw) =>
          filters.rmw_implementations.includes(rmw)
        )
      );
    }

    if (filters.licenses.length > 0) {
      filtered = filtered.filter((template) =>
        filters.licenses.includes(template.license)
      );
    }

    if (filters.difficulty.length > 0) {
      filtered = filtered.filter(
        (template) =>
          template.metadata?.difficulty &&
          filters.difficulty.includes(template.metadata.difficulty)
      );
    }

    if (filters.tags.length > 0) {
      filtered = filtered.filter((template) =>
        template.tags.some((tag) => filters.tags.includes(tag))
      );
    }

    if (filters.requires_gpu) {
      filtered = filtered.filter((template) => template.requires_gpu);
    }

    if (filters.official_only) {
      filtered = filtered.filter((template) => template.is_official);
    }

    if (filters.verified_only) {
      filtered = filtered.filter((template) => template.is_verified);
    }

    // Apply sorting
    filtered.sort((a, b) => {
      let comparison = 0;
      switch (sortBy) {
        case 'name':
          comparison = a.name.localeCompare(b.name);
          break;
        case 'created_at':
          comparison =
            new Date(a.created_at).getTime() - new Date(b.created_at).getTime();
          break;
        case 'updated_at':
          comparison =
            new Date(a.updated_at).getTime() - new Date(b.updated_at).getTime();
          break;
        case 'rating':
          comparison = (a.rating || 0) - (b.rating || 0);
          break;
        case 'installs_7d':
          comparison = (a.installs_7d || 0) - (b.installs_7d || 0);
          break;
        default:
          return 0;
      }
      return sortOrder === 'desc' ? -comparison : comparison;
    });

    return filtered;
  }, [templates, searchQuery, filters, sortBy, sortOrder]);

  // Handlers
  const handlePreview = (template: Template) => {
    setPreviewTemplate(template);
    setPreviewOpen(true);
  };

  const handleLaunch = async (template: Template) => {
    const name = window.prompt(
      'New project name from template:',
      `${template.name}-copy`
    );
    if (!name) return;
    try {
      // POST clone with custom name
      const cloned = await ApiService.post<any>(
        `/projects/${template.id}/clone`,
        { name }
      );
      navigate(`/projects/${cloned.id}`);
    } catch (err) {
      console.error('Failed to create project from template:', err);
      alert(
        err instanceof Error
          ? err.message
          : 'Failed to create project from template'
      );
    }
  };

  const handleBookmark = (templateId: string, bookmarked: boolean) => {
    setBookmarkedTemplates((prev) => {
      const newSet = new Set(prev);
      if (bookmarked) {
        newSet.add(templateId);
      } else {
        newSet.delete(templateId);
      }
      return newSet;
    });
  };

  const handleClearSearch = () => {
    setSearchQuery('');
  };

  const handleClearFilters = () => {
    setFilters(DEFAULT_TEMPLATE_FILTERS);
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

  const handleFilterTabChange = (
    event: React.SyntheticEvent,
    newValue: FilterTab
  ) => {
    setActiveFilterTab(newValue);
  };

  // Loading state
  if (loading) {
    return (
      <Box
        display="flex"
        justifyContent="center"
        alignItems="center"
        minHeight="400px"
      >
        <CircularProgress />
      </Box>
    );
  }

  // Error state
  if (error) {
    return (
      <Box
        display="flex"
        justifyContent="center"
        alignItems="center"
        minHeight="400px"
      >
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

            {/* Sort */}
            <TextField
              select
              label="Sort by"
              value={sortBy}
              onChange={(e) => setSortBy(e.target.value as SortOption)}
              size="small"
              sx={{ minWidth: 120 }}
            >
              <option value="updated_at">Recently Updated</option>
              <option value="created_at">Newest</option>
              <option value="name">Name</option>
              <option value="rating">Rating</option>
              <option value="installs_7d">Popular</option>
            </TextField>

            {/* View Mode */}
            <Box
              sx={{
                display: 'flex',
                border: 1,
                borderColor: 'divider',
                borderRadius: 1,
              }}
            >
              <Tooltip title="Grid view">
                <IconButton
                  size="small"
                  onClick={() => setViewMode('grid')}
                  color={viewMode === 'grid' ? 'primary' : 'default'}
                >
                  <GridIcon />
                </IconButton>
              </Tooltip>
              <Tooltip title="List view">
                <IconButton
                  size="small"
                  onClick={() => setViewMode('list')}
                  color={viewMode === 'list' ? 'primary' : 'default'}
                >
                  <ListIcon />
                </IconButton>
              </Tooltip>
            </Box>

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
                        (newFilters[
                          key as keyof TemplateFiltersType
                        ] as string[]) = (
                          newFilters[
                            key as keyof TemplateFiltersType
                          ] as string[]
                        ).filter((item) => item !== v);
                        setFilters(newFilters);
                      }}
                    />
                  ));
                } else if (typeof value === 'boolean' && value) {
                  return (
                    <Chip
                      key={key}
                      label={key.replace(/_/g, ' ')}
                      size="small"
                      onDelete={() => {
                        setFilters({ ...filters, [key]: false });
                      }}
                    />
                  );
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
              <TemplateCard
                template={template}
                onPreview={handlePreview}
                onLaunch={handleLaunch}
                onBookmark={handleBookmark}
                isBookmarked={bookmarkedTemplates.has(template.id)}
              />
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
              Try adjusting your search or filters to find what you're looking
              for.
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
          <TemplateFilters
            open={filtersOpen}
            onClose={() => setFiltersOpen(false)}
            filters={filters}
            onFiltersChange={setFilters}
            stats={stats}
            availableTags={getAvailableTags()}
          />
        </Drawer>

        {/* Preview Modal */}
        <TemplatePreview
          template={previewTemplate}
          open={previewOpen}
          onClose={() => {
            setPreviewOpen(false);
            setPreviewTemplate(null);
          }}
          onLaunch={handleLaunch}
          onBookmark={handleBookmark}
          isBookmarked={
            previewTemplate
              ? bookmarkedTemplates.has(previewTemplate.id)
              : false
          }
        />
      </Box>
    );
  }

  // Desktop layout
  return (
    <Box>
      {/* Main Content: Left Sidebar + Cards Area */}
      <Box sx={{ display: 'flex', alignItems: 'flex-start', gap: 3 }}>
        {/* Left Sidebar (Single structured section) */}
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
                        (newFilters[
                          key as keyof TemplateFiltersType
                        ] as string[]) = (
                          newFilters[
                            key as keyof TemplateFiltersType
                          ] as string[]
                        ).filter((item) => item !== v);
                        setFilters(newFilters);
                      }}
                    />
                  ));
                } else if (typeof value === 'boolean' && value) {
                  return (
                    <Chip
                      key={key}
                      label={key.replace(/_/g, ' ')}
                      size="small"
                      onDelete={() => setFilters({ ...filters, [key]: false })}
                    />
                  );
                }
                return null;
              })}
              <Button size="small" onClick={handleClearFilters}>
                Clear all
              </Button>
            </Box>
          )}

          {/* Filters (single block, no internal scroll) */}
          <Box
            sx={{
              border: 1,
              borderColor: 'divider',
              borderRadius: 2,
              bgcolor: 'background.paper',
              p: 2,
            }}
          >
            <TemplateFilters
              open={true}
              onClose={() => {}}
              filters={filters}
              onFiltersChange={setFilters}
              stats={stats}
              availableTags={getAvailableTags()}
              permanent={true}
              activeTab={activeFilterTab}
            />
          </Box>
        </Box>

        {/* Cards Area */}
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

              <TextField
                select
                label="Sort by"
                value={sortBy}
                onChange={(e) => setSortBy(e.target.value as SortOption)}
                size="small"
                sx={{ minWidth: 140 }}
              >
                <MenuItem value="name">Name</MenuItem>
                <MenuItem value="created_at">Created</MenuItem>
                <MenuItem value="updated_at">Updated</MenuItem>
                <MenuItem value="rating">Rating</MenuItem>
                <MenuItem value="installs_7d">Installs</MenuItem>
              </TextField>

              <ToggleButtonGroup
                value={sortOrder}
                exclusive
                onChange={(_, value) => value && setSortOrder(value)}
                size="small"
              >
                <ToggleButton value="asc">↑</ToggleButton>
                <ToggleButton value="desc">↓</ToggleButton>
              </ToggleButtonGroup>
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
                  <TemplateCard
                    template={template}
                    onPreview={handlePreview}
                    onLaunch={handleLaunch}
                    onBookmark={handleBookmark}
                    isBookmarked={bookmarkedTemplates.has(template.id)}
                  />
                </Grid>
              ))}
            </Grid>
          ) : (
            <Box sx={{ textAlign: 'center', py: 8, color: 'text.secondary' }}>
              <Typography variant="h6" gutterBottom>
                No templates found
              </Typography>
              <Typography variant="body2">
                Try adjusting your search or filters to find what you're looking
                for.
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
        <TemplateFilters
          open={filtersOpen}
          onClose={() => setFiltersOpen(false)}
          filters={filters}
          onFiltersChange={setFilters}
          stats={stats}
          availableTags={getAvailableTags()}
        />
      </Drawer>

      {/* Preview Modal */}
      <TemplatePreview
        template={previewTemplate}
        open={previewOpen}
        onClose={() => {
          setPreviewOpen(false);
          setPreviewTemplate(null);
        }}
        onLaunch={handleLaunch}
        onBookmark={handleBookmark}
        isBookmarked={
          previewTemplate ? bookmarkedTemplates.has(previewTemplate.id) : false
        }
      />
    </Box>
  );
};

export default Templates;
