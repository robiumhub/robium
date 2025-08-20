import React, { useState, useEffect, useCallback } from 'react';
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
  Divider,
  Pagination,
  Stack,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Checkbox,
  FormControlLabel,
  Slider,
  Rating,
  Tabs,
  Tab,
  Paper,
  Alert,
  CircularProgress,
  Badge,
  Switch,
  FormGroup,
} from '@mui/material';
import {
  Search as SearchIcon,
  FilterList as FilterIcon,
  Info as InfoIcon,
  Category as CategoryIcon,
  Tag as TagIcon,
  Inventory as PackageIcon,
  AccountTree as DependencyIcon,
  ExpandMore as ExpandMoreIcon,
  Compare as CompareIcon,
  Star as StarIcon,
  StarBorder as StarBorderIcon,
  CheckCircle as CheckCircleIcon,
  Warning as WarningIcon,
  Error as ErrorIcon,
  Speed as SpeedIcon,
  Memory as MemoryIcon,
  Storage as StorageIcon,
  Security as SecurityIcon,
  Language as LanguageIcon,
  Build as BuildIcon,
  Clear as ClearIcon,
  Save as SaveIcon,
  Share as ShareIcon,
} from '@mui/icons-material';
import { ApiService } from '../services/api';

// Types for algorithm search and filtering
interface AlgorithmSearchCriteria {
  text?: string;
  categories?: string[];
  complexity?: string[];
  status?: string[];
  language?: string[];
  tags?: string[];
  metaCategory?: string;
  taskCategory?: string;
  minScore?: number;
  maxScore?: number;
  hasDependencies?: boolean;
  hasDocumentation?: boolean;
  hasTests?: boolean;
}

interface AlgorithmSearchResult {
  algorithm: AlgorithmDocumentation;
  relevanceScore: number;
  matchedCriteria: string[];
  compatibilityScore?: number;
  compatibilityIssues?: string[];
}

interface AlgorithmDocumentation {
  id: string;
  name: string;
  version: string;
  taskDefinition: {
    title: string;
    description: string;
    problemStatement: string;
    useCases?: string[];
  };
  inputOutputSpecification: {
    inputs: Array<{
      name: string;
      type: string;
      description: string;
      required: boolean;
    }>;
    outputs: Array<{
      name: string;
      type: string;
      description: string;
    }>;
  };
  parameters: {
    configurable: Array<{
      name: string;
      type: string;
      description: string;
      default?: any;
    }>;
  };
  dependencies: {
    rosPackages: Array<{
      name: string;
      version?: string;
      purpose: string;
    }>;
    systemLibraries?: string[];
    pythonPackages?: Array<{
      name: string;
      version?: string;
    }>;
  };
  performance?: {
    complexity?: string;
    executionTime?: {
      typical?: string;
    };
    memoryUsage?: {
      typical?: string;
    };
  };
  implementation?: {
    language: string;
    architecture?: string;
  };
  metadata: {
    createdAt: string;
    updatedAt: string;
    status: string;
    tags?: string[];
    categories?: string[];
  };
}

interface FacetedSearchState {
  categories: string[];
  complexities: string[];
  statuses: string[];
  languages: string[];
  tags: string[];
  metaCategories: string[];
  taskCategories: string[];
}

interface TabPanelProps {
  children?: React.ReactNode;
  index: number;
  value: number;
}

function TabPanel(props: TabPanelProps) {
  const { children, value, index, ...other } = props;

  return (
    <div
      role="tabpanel"
      hidden={value !== index}
      id={`search-tabpanel-${index}`}
      aria-labelledby={`search-tab-${index}`}
      {...other}
    >
      {value === index && <Box sx={{ p: 3 }}>{children}</Box>}
    </div>
  );
}

interface AlgorithmSearchInterfaceProps {
  onAlgorithmSelect?: (algorithm: AlgorithmDocumentation) => void;
  onAlgorithmCompare?: (algorithms: AlgorithmDocumentation[]) => void;
  showComparisonMode?: boolean;
  maxSelections?: number;
}

const AlgorithmSearchInterface: React.FC<AlgorithmSearchInterfaceProps> = ({
  onAlgorithmSelect,
  onAlgorithmCompare,
  showComparisonMode = false,
  maxSelections = 5,
}) => {
  // State management
  const [searchCriteria, setSearchCriteria] = useState<AlgorithmSearchCriteria>({});
  const [searchResults, setSearchResults] = useState<AlgorithmSearchResult[]>([]);
  const [filteredResults, setFilteredResults] = useState<AlgorithmSearchResult[]>([]);
  const [displayedResults, setDisplayedResults] = useState<AlgorithmSearchResult[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [currentPage, setCurrentPage] = useState(1);
  const [resultsPerPage] = useState(12);
  const [selectedAlgorithm, setSelectedAlgorithm] = useState<AlgorithmDocumentation | null>(null);
  const [dialogOpen, setDialogOpen] = useState(false);
  const [tabValue, setTabValue] = useState(0);
  const [showAdvancedFilters, setShowAdvancedFilters] = useState(false);
  const [selectedAlgorithms, setSelectedAlgorithms] = useState<AlgorithmDocumentation[]>([]);
  const [facetedSearchState, setFacetedSearchState] = useState<FacetedSearchState>({
    categories: [],
    complexities: [],
    statuses: [],
    languages: [],
    tags: [],
    metaCategories: [],
    taskCategories: [],
  });

  // Fetch search results
  const performSearch = useCallback(async (criteria: AlgorithmSearchCriteria) => {
    setLoading(true);
    setError(null);
    
    try {
      const response = await ApiService.post<{
        results: AlgorithmSearchResult[];
        total: number;
        facets: FacetedSearchState;
      }>('/algorithms/search', criteria);
      
      setSearchResults(response.results);
      setFacetedSearchState(response.facets);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Search failed');
      setSearchResults([]);
    } finally {
      setLoading(false);
    }
  }, []);

  // Apply filters to search results
  useEffect(() => {
    let filtered = searchResults;

    // Apply text search
    if (searchCriteria.text) {
      const searchTerm = searchCriteria.text.toLowerCase();
      filtered = filtered.filter(result =>
        result.algorithm.name.toLowerCase().includes(searchTerm) ||
        result.algorithm.taskDefinition.description.toLowerCase().includes(searchTerm) ||
        result.algorithm.metadata.tags?.some(tag => tag.toLowerCase().includes(searchTerm)) ||
        result.matchedCriteria.some(criteria => criteria.toLowerCase().includes(searchTerm))
      );
    }

    // Apply category filters
    if (searchCriteria.categories && searchCriteria.categories.length > 0) {
      filtered = filtered.filter(result =>
        result.algorithm.metadata.categories?.some(cat => 
          searchCriteria.categories!.includes(cat)
        )
      );
    }

    // Apply complexity filters
    if (searchCriteria.complexity && searchCriteria.complexity.length > 0) {
      filtered = filtered.filter(result =>
        result.algorithm.performance?.complexity &&
        searchCriteria.complexity!.includes(result.algorithm.performance.complexity)
      );
    }

    // Apply status filters
    if (searchCriteria.status && searchCriteria.status.length > 0) {
      filtered = filtered.filter(result =>
        searchCriteria.status!.includes(result.algorithm.metadata.status)
      );
    }

    // Apply language filters
    if (searchCriteria.language && searchCriteria.language.length > 0) {
      filtered = filtered.filter(result =>
        result.algorithm.implementation?.language &&
        searchCriteria.language!.includes(result.algorithm.implementation.language)
      );
    }

    // Apply score filters
    if (searchCriteria.minScore !== undefined) {
      filtered = filtered.filter(result => result.relevanceScore >= searchCriteria.minScore!);
    }
    if (searchCriteria.maxScore !== undefined) {
      filtered = filtered.filter(result => result.relevanceScore <= searchCriteria.maxScore!);
    }

    setFilteredResults(filtered);
    setCurrentPage(1);
  }, [searchResults, searchCriteria]);

  // Handle pagination
  useEffect(() => {
    const startIndex = (currentPage - 1) * resultsPerPage;
    const endIndex = startIndex + resultsPerPage;
    setDisplayedResults(filteredResults.slice(startIndex, endIndex));
  }, [filteredResults, currentPage, resultsPerPage]);

  // Handle search input changes
  const handleSearchChange = (field: keyof AlgorithmSearchCriteria, value: any) => {
    setSearchCriteria(prev => ({
      ...prev,
      [field]: value,
    }));
  };

  // Handle search submission
  const handleSearch = () => {
    performSearch(searchCriteria);
  };

  // Handle algorithm selection
  const handleAlgorithmClick = (algorithm: AlgorithmDocumentation) => {
    if (showComparisonMode) {
      if (selectedAlgorithms.find(a => a.id === algorithm.id)) {
        setSelectedAlgorithms(prev => prev.filter(a => a.id !== algorithm.id));
      } else if (selectedAlgorithms.length < maxSelections) {
        setSelectedAlgorithms(prev => [...prev, algorithm]);
      }
    } else {
      setSelectedAlgorithm(algorithm);
      setDialogOpen(true);
      onAlgorithmSelect?.(algorithm);
    }
  };

  // Handle algorithm comparison
  const handleCompareAlgorithms = () => {
    if (selectedAlgorithms.length >= 2) {
      onAlgorithmCompare?.(selectedAlgorithms);
    }
  };

  // Clear all filters
  const handleClearFilters = () => {
    setSearchCriteria({});
    setSelectedAlgorithms([]);
  };

  // Get algorithm status color
  const getStatusColor = (status: string) => {
    const colors: { [key: string]: string } = {
      stable: '#4caf50',
      beta: '#ff9800',
      experimental: '#f44336',
      deprecated: '#9e9e9e',
    };
    return colors[status] || '#757575';
  };

  // Get complexity color
  const getComplexityColor = (complexity: string) => {
    const colors: { [key: string]: string } = {
      'O(1)': '#4caf50',
      'O(log n)': '#8bc34a',
      'O(n)': '#ff9800',
      'O(n log n)': '#ff5722',
      'O(n²)': '#f44336',
      'O(n³)': '#9c27b0',
      'O(2ⁿ)': '#e91e63',
      'O(n!)': '#3f51b5',
    };
    return colors[complexity] || '#757575';
  };

  const totalPages = Math.ceil(filteredResults.length / resultsPerPage);

  return (
    <Box>
      {/* Search Header */}
      <Box sx={{ mb: 3 }}>
        <Typography variant="h4" component="h1" gutterBottom>
          Algorithm Search & Discovery
        </Typography>
        <Typography variant="body1" color="text.secondary" gutterBottom>
          Find and compare algorithms for your robotics projects
        </Typography>
      </Box>

      {/* Search Controls */}
      <Card sx={{ mb: 3 }}>
        <CardContent>
          <Grid container spacing={2} alignItems="center">
            <Grid item xs={12} md={6}>
              <TextField
                fullWidth
                placeholder="Search algorithms by name, description, or tags..."
                value={searchCriteria.text || ''}
                onChange={(e) => handleSearchChange('text', e.target.value)}
                InputProps={{
                  startAdornment: (
                    <InputAdornment position="start">
                      <SearchIcon />
                    </InputAdornment>
                  ),
                  endAdornment: (
                    <InputAdornment position="end">
                      <IconButton
                        onClick={() => handleSearchChange('text', '')}
                        size="small"
                      >
                        <ClearIcon />
                      </IconButton>
                    </InputAdornment>
                  ),
                }}
              />
            </Grid>
            <Grid item xs={12} md={3}>
              <FormControl fullWidth>
                <InputLabel>Meta Category</InputLabel>
                <Select
                  value={searchCriteria.metaCategory || ''}
                  label="Meta Category"
                  onChange={(e) => handleSearchChange('metaCategory', e.target.value)}
                >
                  <MenuItem value="">All Categories</MenuItem>
                  {facetedSearchState.metaCategories.map(category => (
                    <MenuItem key={category} value={category}>
                      {category.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase())}
                    </MenuItem>
                  ))}
                </Select>
              </FormControl>
            </Grid>
            <Grid item xs={12} md={3}>
              <Button
                fullWidth
                variant="contained"
                onClick={handleSearch}
                disabled={loading}
                startIcon={loading ? <CircularProgress size={20} /> : <SearchIcon />}
              >
                {loading ? 'Searching...' : 'Search'}
              </Button>
            </Grid>
          </Grid>

          {/* Advanced Filters Toggle */}
          <Box sx={{ mt: 2, display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <Button
              startIcon={<FilterIcon />}
              onClick={() => setShowAdvancedFilters(!showAdvancedFilters)}
              variant="outlined"
              size="small"
            >
              {showAdvancedFilters ? 'Hide' : 'Show'} Advanced Filters
            </Button>
            
            {showComparisonMode && (
              <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                <Typography variant="body2" color="text.secondary">
                  {selectedAlgorithms.length} of {maxSelections} selected
                </Typography>
                <Button
                  variant="contained"
                  color="secondary"
                  onClick={handleCompareAlgorithms}
                  disabled={selectedAlgorithms.length < 2}
                  startIcon={<CompareIcon />}
                >
                  Compare ({selectedAlgorithms.length})
                </Button>
              </Box>
            )}
          </Box>

          {/* Advanced Filters */}
          {showAdvancedFilters && (
            <Box sx={{ mt: 2 }}>
              <Tabs value={tabValue} onChange={(_, newValue) => setTabValue(newValue)}>
                <Tab label="Categories & Tags" />
                <Tab label="Performance" />
                <Tab label="Implementation" />
                <Tab label="Compatibility" />
              </Tabs>

              <TabPanel value={tabValue} index={0}>
                <Grid container spacing={2}>
                  <Grid item xs={12} md={6}>
                    <Typography variant="h6" gutterBottom>Categories</Typography>
                    <FormGroup>
                      {facetedSearchState.categories.map(category => (
                        <FormControlLabel
                          key={category}
                          control={
                            <Checkbox
                              checked={searchCriteria.categories?.includes(category) || false}
                              onChange={(e) => {
                                const current = searchCriteria.categories || [];
                                const updated = e.target.checked
                                  ? [...current, category]
                                  : current.filter(c => c !== category);
                                handleSearchChange('categories', updated);
                              }}
                            />
                          }
                          label={category}
                        />
                      ))}
                    </FormGroup>
                  </Grid>
                  <Grid item xs={12} md={6}>
                    <Typography variant="h6" gutterBottom>Tags</Typography>
                    <FormGroup>
                      {facetedSearchState.tags.slice(0, 10).map(tag => (
                        <FormControlLabel
                          key={tag}
                          control={
                            <Checkbox
                              checked={searchCriteria.tags?.includes(tag) || false}
                              onChange={(e) => {
                                const current = searchCriteria.tags || [];
                                const updated = e.target.checked
                                  ? [...current, tag]
                                  : current.filter(t => t !== tag);
                                handleSearchChange('tags', updated);
                              }}
                            />
                          }
                          label={tag}
                        />
                      ))}
                    </FormGroup>
                  </Grid>
                </Grid>
              </TabPanel>

              <TabPanel value={tabValue} index={1}>
                <Grid container spacing={2}>
                  <Grid item xs={12} md={6}>
                    <Typography variant="h6" gutterBottom>Complexity</Typography>
                    <FormGroup>
                      {facetedSearchState.complexities.map(complexity => (
                        <FormControlLabel
                          key={complexity}
                          control={
                            <Checkbox
                              checked={searchCriteria.complexity?.includes(complexity) || false}
                              onChange={(e) => {
                                const current = searchCriteria.complexity || [];
                                const updated = e.target.checked
                                  ? [...current, complexity]
                                  : current.filter(c => c !== complexity);
                                handleSearchChange('complexity', updated);
                              }}
                            />
                          }
                          label={
                            <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                              <Chip
                                label={complexity}
                                size="small"
                                sx={{
                                  backgroundColor: getComplexityColor(complexity),
                                  color: 'white',
                                  fontSize: '0.75rem'
                                }}
                              />
                            </Box>
                          }
                        />
                      ))}
                    </FormGroup>
                  </Grid>
                  <Grid item xs={12} md={6}>
                    <Typography variant="h6" gutterBottom>Relevance Score</Typography>
                    <Box sx={{ px: 2 }}>
                      <Slider
                        value={[searchCriteria.minScore || 0, searchCriteria.maxScore || 100]}
                        onChange={(_, value) => {
                          const [min, max] = value as number[];
                          handleSearchChange('minScore', min);
                          handleSearchChange('maxScore', max);
                        }}
                        valueLabelDisplay="auto"
                        min={0}
                        max={100}
                      />
                      <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                        <Typography variant="caption">0</Typography>
                        <Typography variant="caption">100</Typography>
                      </Box>
                    </Box>
                  </Grid>
                </Grid>
              </TabPanel>

              <TabPanel value={tabValue} index={2}>
                <Grid container spacing={2}>
                  <Grid item xs={12} md={6}>
                    <Typography variant="h6" gutterBottom>Status</Typography>
                    <FormGroup>
                      {facetedSearchState.statuses.map(status => (
                        <FormControlLabel
                          key={status}
                          control={
                            <Checkbox
                              checked={searchCriteria.status?.includes(status) || false}
                              onChange={(e) => {
                                const current = searchCriteria.status || [];
                                const updated = e.target.checked
                                  ? [...current, status]
                                  : current.filter(s => s !== status);
                                handleSearchChange('status', updated);
                              }}
                            />
                          }
                          label={
                            <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                              <Chip
                                label={status}
                                size="small"
                                sx={{
                                  backgroundColor: getStatusColor(status),
                                  color: 'white',
                                  fontSize: '0.75rem'
                                }}
                              />
                            </Box>
                          }
                        />
                      ))}
                    </FormGroup>
                  </Grid>
                  <Grid item xs={12} md={6}>
                    <Typography variant="h6" gutterBottom>Language</Typography>
                    <FormGroup>
                      {facetedSearchState.languages.map(language => (
                        <FormControlLabel
                          key={language}
                          control={
                            <Checkbox
                              checked={searchCriteria.language?.includes(language) || false}
                              onChange={(e) => {
                                const current = searchCriteria.language || [];
                                const updated = e.target.checked
                                  ? [...current, language]
                                  : current.filter(l => l !== language);
                                handleSearchChange('language', updated);
                              }}
                            />
                          }
                          label={language}
                        />
                      ))}
                    </FormGroup>
                  </Grid>
                </Grid>
              </TabPanel>

              <TabPanel value={tabValue} index={3}>
                <Grid container spacing={2}>
                  <Grid item xs={12} md={6}>
                    <FormControlLabel
                      control={
                        <Switch
                          checked={searchCriteria.hasDependencies || false}
                          onChange={(e) => handleSearchChange('hasDependencies', e.target.checked)}
                        />
                      }
                      label="Has Dependencies"
                    />
                    <FormControlLabel
                      control={
                        <Switch
                          checked={searchCriteria.hasDocumentation || false}
                          onChange={(e) => handleSearchChange('hasDocumentation', e.target.checked)}
                        />
                      }
                      label="Has Documentation"
                    />
                    <FormControlLabel
                      control={
                        <Switch
                          checked={searchCriteria.hasTests || false}
                          onChange={(e) => handleSearchChange('hasTests', e.target.checked)}
                        />
                      }
                      label="Has Tests"
                    />
                  </Grid>
                </Grid>
              </TabPanel>
            </Box>
          )}

          {/* Clear Filters */}
          <Box sx={{ mt: 2, display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <Button
              onClick={handleClearFilters}
              startIcon={<ClearIcon />}
              variant="outlined"
              size="small"
            >
              Clear All Filters
            </Button>
            <Typography variant="body2" color="text.secondary">
              {filteredResults.length} results found
            </Typography>
          </Box>
        </CardContent>
      </Card>

      {/* Error Display */}
      {error && (
        <Alert severity="error" sx={{ mb: 3 }}>
          {error}
        </Alert>
      )}

      {/* Search Results */}
      {!loading && filteredResults.length > 0 && (
        <>
          <Grid container spacing={3}>
            {displayedResults.map((result) => {
              const algorithm = result.algorithm;
              const isSelected = selectedAlgorithms.find(a => a.id === algorithm.id);
              
              return (
                <Grid item xs={12} sm={6} md={4} lg={3} key={algorithm.id}>
                  <Card
                    sx={{
                      height: '100%',
                      display: 'flex',
                      flexDirection: 'column',
                      cursor: 'pointer',
                      border: isSelected ? 2 : 1,
                      borderColor: isSelected ? 'primary.main' : 'divider',
                      '&:hover': {
                        boxShadow: 4,
                        transform: 'translateY(-2px)',
                        transition: 'all 0.2s ease-in-out'
                      }
                    }}
                    onClick={() => handleAlgorithmClick(algorithm)}
                  >
                    <CardContent sx={{ flexGrow: 1 }}>
                      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', mb: 2 }}>
                        <Typography variant="h6" component="h2" gutterBottom>
                          {algorithm.name.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase())}
                        </Typography>
                        <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
                          {isSelected && <CheckCircleIcon color="primary" />}
                          <Tooltip title="Algorithm Details">
                            <IconButton size="small">
                              <InfoIcon />
                            </IconButton>
                          </Tooltip>
                        </Box>
                      </Box>
                      
                      <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
                        {algorithm.taskDefinition.description}
                      </Typography>

                      {/* Relevance Score */}
                      <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
                        <StarIcon sx={{ fontSize: 16, mr: 0.5, color: 'warning.main' }} />
                        <Typography variant="caption" color="text.secondary">
                          Score: {result.relevanceScore.toFixed(1)}
                        </Typography>
                      </Box>

                      {/* Status */}
                      <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
                        <Chip
                          label={algorithm.metadata.status}
                          size="small"
                          sx={{
                            backgroundColor: getStatusColor(algorithm.metadata.status),
                            color: 'white',
                            fontSize: '0.75rem'
                          }}
                        />
                      </Box>

                      {/* Complexity */}
                      {algorithm.performance?.complexity && (
                        <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
                          <SpeedIcon sx={{ fontSize: 16, mr: 0.5, color: 'text.secondary' }} />
                          <Chip
                            label={algorithm.performance.complexity}
                            size="small"
                            sx={{
                              backgroundColor: getComplexityColor(algorithm.performance.complexity),
                              color: 'white',
                              fontSize: '0.75rem'
                            }}
                          />
                        </Box>
                      )}

                      {/* Language */}
                      {algorithm.implementation?.language && (
                        <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
                          <LanguageIcon sx={{ fontSize: 16, mr: 0.5, color: 'text.secondary' }} />
                          <Typography variant="caption" color="text.secondary">
                            {algorithm.implementation.language}
                          </Typography>
                        </Box>
                      )}

                      {/* Dependencies */}
                      <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
                        <PackageIcon sx={{ fontSize: 16, mr: 0.5, color: 'text.secondary' }} />
                        <Typography variant="caption" color="text.secondary">
                          {algorithm.dependencies.rosPackages.length} ROS packages
                        </Typography>
                      </Box>

                      {/* Tags */}
                      <Box sx={{ mt: 2 }}>
                        {algorithm.metadata.tags?.slice(0, 3).map((tag) => (
                          <Chip
                            key={tag}
                            label={tag}
                            size="small"
                            variant="outlined"
                            sx={{ mr: 0.5, mb: 0.5 }}
                          />
                        ))}
                        {algorithm.metadata.tags && algorithm.metadata.tags.length > 3 && (
                          <Chip
                            label={`+${algorithm.metadata.tags.length - 3} more`}
                            size="small"
                            variant="outlined"
                            sx={{ mr: 0.5, mb: 0.5 }}
                          />
                        )}
                      </Box>

                      <Typography variant="caption" color="text.secondary" sx={{ mt: 2, display: 'block' }}>
                        Version: {algorithm.version}
                      </Typography>
                    </CardContent>
                  </Card>
                </Grid>
              );
            })}
          </Grid>

          {/* Pagination */}
          {totalPages > 1 && (
            <Box sx={{ mt: 4, display: 'flex', justifyContent: 'center' }}>
              <Stack spacing={2}>
                <Pagination
                  count={totalPages}
                  page={currentPage}
                  onChange={(_, value) => setCurrentPage(value)}
                  color="primary"
                  size="large"
                  showFirstButton
                  showLastButton
                />
                <Typography variant="body2" color="text.secondary" textAlign="center">
                  Page {currentPage} of {totalPages}
                </Typography>
              </Stack>
            </Box>
          )}
        </>
      )}

      {/* No Results */}
      {!loading && filteredResults.length === 0 && searchResults.length > 0 && (
        <Box sx={{ textAlign: 'center', py: 4 }}>
          <Typography variant="h6" color="text.secondary" gutterBottom>
            No algorithms match your current filters
          </Typography>
          <Button onClick={handleClearFilters} variant="outlined">
            Clear Filters
          </Button>
        </Box>
      )}

      {/* Loading State */}
      {loading && (
        <Box sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', py: 4 }}>
          <CircularProgress />
          <Typography sx={{ ml: 2 }}>Searching algorithms...</Typography>
        </Box>
      )}

      {/* Algorithm Details Dialog */}
      <Dialog open={dialogOpen} onClose={() => setDialogOpen(false)} maxWidth="md" fullWidth>
        {selectedAlgorithm && (
          <>
            <DialogTitle>
              <Typography variant="h5">
                {selectedAlgorithm.name.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase())}
              </Typography>
              <Typography variant="body2" color="text.secondary">
                Version {selectedAlgorithm.version}
              </Typography>
            </DialogTitle>
            <DialogContent>
              <Typography variant="body1" sx={{ mb: 2 }}>
                {selectedAlgorithm.taskDefinition.description}
              </Typography>

              <Box sx={{ mb: 2 }}>
                <Typography variant="h6" gutterBottom>
                  <CategoryIcon sx={{ fontSize: 20, mr: 1, verticalAlign: 'middle' }} />
                  Status
                </Typography>
                <Chip
                  label={selectedAlgorithm.metadata.status}
                  sx={{
                    backgroundColor: getStatusColor(selectedAlgorithm.metadata.status),
                    color: 'white'
                  }}
                />
              </Box>

              {selectedAlgorithm.performance?.complexity && (
                <Box sx={{ mb: 2 }}>
                  <Typography variant="h6" gutterBottom>
                    <SpeedIcon sx={{ fontSize: 20, mr: 1, verticalAlign: 'middle' }} />
                    Performance
                  </Typography>
                  <Chip
                    label={selectedAlgorithm.performance.complexity}
                    sx={{
                      backgroundColor: getComplexityColor(selectedAlgorithm.performance.complexity),
                      color: 'white'
                    }}
                  />
                </Box>
              )}

              <Box sx={{ mb: 2 }}>
                <Typography variant="h6" gutterBottom>
                  <PackageIcon sx={{ fontSize: 20, mr: 1, verticalAlign: 'middle' }} />
                  ROS Packages ({selectedAlgorithm.dependencies.rosPackages.length})
                </Typography>
                <List dense>
                  {selectedAlgorithm.dependencies.rosPackages.map((pkg) => (
                    <ListItem key={pkg.name}>
                      <ListItemText 
                        primary={pkg.name}
                        secondary={pkg.purpose}
                      />
                    </ListItem>
                  ))}
                </List>
              </Box>

              {selectedAlgorithm.metadata.tags && selectedAlgorithm.metadata.tags.length > 0 && (
                <Box>
                  <Typography variant="h6" gutterBottom>
                    <TagIcon sx={{ fontSize: 20, mr: 1, verticalAlign: 'middle' }} />
                    Tags
                  </Typography>
                  <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
                    {selectedAlgorithm.metadata.tags.map((tag) => (
                      <Chip key={tag} label={tag} size="small" />
                    ))}
                  </Box>
                </Box>
              )}
            </DialogContent>
            <DialogActions>
              <Button onClick={() => setDialogOpen(false)}>Close</Button>
              <Button variant="contained" color="primary">
                Add to Project
              </Button>
            </DialogActions>
          </>
        )}
      </Dialog>
    </Box>
  );
};

export default AlgorithmSearchInterface; 