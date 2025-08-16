import React, { useEffect, useState, useMemo } from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
  Chip,
  TextField,
  InputAdornment,
  IconButton,
  ToggleButtonGroup,
  ToggleButton,
  useTheme,
  useMediaQuery,
} from '@mui/material';
import {
  Search as SearchIcon,
  Clear as ClearIcon,
  ViewModule as ViewModuleIcon,
  ViewList as ViewListIcon,
} from '@mui/icons-material';
import { RobotsService, Robot } from '../services/robotsService';

type SortOption = 'name' | 'module_count';
type ViewMode = 'grid' | 'list';

const Robots: React.FC = () => {
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('md'));
  const [robots, setRobots] = useState<Robot[]>([]);
  const [loading, setLoading] = useState(true);
  const [searchQuery, setSearchQuery] = useState('');
  const [sortBy, setSortBy] = useState<SortOption>('name');
  const [viewMode, setViewMode] = useState<ViewMode>('grid');

  useEffect(() => {
    const load = async () => {
      try {
        setLoading(true);
        const robotsData = await RobotsService.getRobots();
        setRobots(robotsData);
      } catch (e) {
        console.error('Failed to load robots:', e);
        setRobots([]);
      } finally {
        setLoading(false);
      }
    };
    load();
  }, []);

  // Filter and sort robots
  const filteredAndSortedRobots = useMemo(() => {
    let filtered = robots.filter(
      (robot) =>
        robot.name.toLowerCase().includes(searchQuery.toLowerCase()) ||
        robot.code.toLowerCase().includes(searchQuery.toLowerCase())
    );

    // Sort robots
    filtered.sort((a, b) => {
      switch (sortBy) {
        case 'name':
          return a.name.localeCompare(b.name);
        case 'module_count':
          return b.module_count - a.module_count;
        default:
          return 0;
      }
    });

    return filtered;
  }, [robots, searchQuery, sortBy]);

  const handleClearSearch = () => {
    setSearchQuery('');
  };

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
            {filteredAndSortedRobots.length} of {robots.length} robots
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
              <ViewModuleIcon />
            </ToggleButton>
            <ToggleButton value="list">
              <ViewListIcon />
            </ToggleButton>
          </ToggleButtonGroup>
        </Box>
      </Box>

      {/* Search and Sort Controls */}
      <Box
        sx={{
          display: 'flex',
          gap: 2,
          mb: 3,
          flexWrap: 'wrap',
          alignItems: 'center',
        }}
      >
        <TextField
          placeholder="Search robots..."
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
                <IconButton size="small" onClick={handleClearSearch} edge="end">
                  <ClearIcon />
                </IconButton>
              </InputAdornment>
            ),
          }}
          size="small"
          sx={{ minWidth: 200 }}
        />

        <ToggleButtonGroup
          value={sortBy}
          exclusive
          onChange={(_, newSort) => newSort && setSortBy(newSort)}
          size="small"
        >
          <ToggleButton value="name">Name</ToggleButton>
          <ToggleButton value="module_count">Modules</ToggleButton>
        </ToggleButtonGroup>
      </Box>

      {loading ? (
        <Typography>Loading robots...</Typography>
      ) : filteredAndSortedRobots.length === 0 ? (
        <Typography color="text.secondary">
          {searchQuery
            ? 'No robots found matching your search.'
            : 'No robots found.'}
        </Typography>
      ) : (
        <Box
          sx={{
            display: 'grid',
            gridTemplateColumns: {
              xs: '1fr',
              sm: viewMode === 'list' ? '1fr' : 'repeat(2, 1fr)',
              md: viewMode === 'list' ? '1fr' : 'repeat(3, 1fr)',
              lg: viewMode === 'list' ? '1fr' : 'repeat(4, 1fr)',
            },
            gap: 3,
          }}
        >
          {filteredAndSortedRobots.map((r) => (
            <Card key={r.code}>
              <CardContent>
                <Typography variant="h6">{r.name}</Typography>
                <Typography
                  variant="body2"
                  color="text.secondary"
                  sx={{ mb: 1 }}
                >
                  {r.code}
                </Typography>
                <Chip
                  label={`${r.module_count} modules`}
                  size="small"
                  sx={{ mt: 1 }}
                />
              </CardContent>
            </Card>
          ))}
        </Box>
      )}
    </Box>
  );
};

export default Robots;
