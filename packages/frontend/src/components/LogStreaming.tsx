import React, { useState, useEffect, useRef } from 'react';
import {
  Box,
  Card,
  CardContent,
  CardHeader,
  Typography,
  TextField,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  IconButton,
  Chip,
  Button,
  Switch,
  FormControlLabel,
  Divider,
  List,
  ListItem,
  ListItemText,
  ListItemIcon,
  ListItemSecondaryAction,
  Badge,
  Tooltip,
  Snackbar,
  Alert,
  CircularProgress,
  InputAdornment,
  useTheme,
  useMediaQuery
} from '@mui/material';
import {
  Search as SearchIcon,
  Clear as ClearIcon,
  FilterList as FilterIcon,
  Refresh as RefreshIcon,
  Download as DownloadIcon,
  Settings as SettingsIcon,
  Error as ErrorIcon,
  Warning as WarningIcon,
  Info as InfoIcon,
  Code as CodeIcon,
  Timeline as TimelineIcon,
  Cloud as CloudIcon,
  Memory as MemoryIcon,
  Speed as SpeedIcon,
  Storage as StorageIcon,
  NetworkCheck as NetworkIcon,
  CheckCircle as SuccessIcon,
  ExpandMore as ExpandMoreIcon,
  Add as AddIcon,
  Delete as DeleteIcon,
  Edit as EditIcon,
  Visibility as ViewIcon,
  Upload as UploadIcon,
  Folder as FolderIcon,
  Build as BuildIcon,
  Security as SecurityIcon,
  Notifications as NotificationsIcon,
  Help as HelpIcon,
  Close as CloseIcon,
  Save as SaveIcon,
  Cancel as CancelIcon,
  RestartAlt as RestartIcon,
  Pause as PauseIcon,
  SkipNext as SkipNextIcon,
  SkipPrevious as SkipPreviousIcon,
  VolumeUp as VolumeUpIcon,
  VolumeOff as VolumeOffIcon,
  Fullscreen as FullscreenIcon,
  FullscreenExit as FullscreenExitIcon,
  ZoomIn as ZoomInIcon,
  ZoomOut as ZoomOutIcon,
  MoreVert as MoreIcon,
  Keyboard as KeyboardIcon,
  Mouse as MouseIcon,
  TouchApp as TouchIcon,
  VisibilityOff as VisibilityOffIcon,
  Lock as LockIcon,
  LockOpen as LockOpenIcon,
  Person as PersonIcon,
  Group as GroupIcon,
  Public as PublicIcon,
  Share as ShareIcon,
  Link as LinkIcon,
  ContentCopy as CopyIcon,
  QrCode as QrCodeIcon,
  History as HistoryIcon,
  Schedule as ScheduleIcon,
  AccessTime as AccessTimeIcon,
  Update as UpdateIcon,
  Sync as SyncIcon,
  Autorenew as AutorenewIcon,
  TrendingUp as TrendingUpIcon,
  TrendingDown as TrendingDownIcon,
  ShowChart as ChartIcon,
  BarChart as BarChartIcon,
  PieChart as PieChartIcon,
  ScatterPlot as ScatterPlotIcon,
  BubbleChart as BubbleChartIcon,
  MultilineChart as MultilineChartIcon,
  InsertChart as InsertChartIcon,
  Assessment as AssessmentIcon,
  Analytics as AnalyticsIcon,
  DataUsage as DataUsageIcon,
  Power as PowerIcon,
  PowerOff as PowerOffIcon,
  PowerSettingsNew as PowerSettingsIcon,
  BatteryFull as BatteryIcon
} from '@mui/icons-material';

// Mock data for development
const mockLogs: LogEntry[] = [
  { id: 1, timestamp: '2024-01-15T10:30:15Z', level: 'info', source: 'ros2-navigation', message: 'Navigation node started successfully', container: 'ros2-navigation' },
  { id: 2, timestamp: '2024-01-15T10:30:16Z', level: 'info', source: 'ros2-navigation', message: 'Loading navigation parameters', container: 'ros2-navigation' },
  { id: 3, timestamp: '2024-01-15T10:30:17Z', level: 'warning', source: 'gazebo-sim', message: 'Physics engine running at reduced performance', container: 'gazebo-sim' },
  { id: 4, timestamp: '2024-01-15T10:30:18Z', level: 'error', source: 'rviz-viz', message: 'Failed to connect to ROS master', container: 'rviz-viz' },
  { id: 5, timestamp: '2024-01-15T10:30:19Z', level: 'info', source: 'ros2-navigation', message: 'Map loaded successfully', container: 'ros2-navigation' },
  { id: 6, timestamp: '2024-01-15T10:30:20Z', level: 'debug', source: 'ros2-navigation', message: 'Processing sensor data', container: 'ros2-navigation' },
  { id: 7, timestamp: '2024-01-15T10:30:21Z', level: 'info', source: 'gazebo-sim', message: 'Simulation environment initialized', container: 'gazebo-sim' },
  { id: 8, timestamp: '2024-01-15T10:30:22Z', level: 'warning', source: 'rviz-viz', message: 'Display refresh rate below optimal', container: 'rviz-viz' },
  { id: 9, timestamp: '2024-01-15T10:30:23Z', level: 'info', source: 'ros2-navigation', message: 'Path planning completed', container: 'ros2-navigation' },
  { id: 10, timestamp: '2024-01-15T10:30:24Z', level: 'error', source: 'gazebo-sim', message: 'Collision detection failed', container: 'gazebo-sim' }
];

interface LogEntry {
  id: number;
  timestamp: string;
  level: 'error' | 'warning' | 'info' | 'debug';
  source: string;
  message: string;
  container: string;
}

interface LogStreamingProps {
  onLogAction?: (action: string, logId: number) => void;
  onFilterChange?: (filters: any) => void;
  autoRefresh?: boolean;
  maxLogs?: number;
}

const LogStreaming: React.FC<LogStreamingProps> = ({
  onLogAction,
  onFilterChange,
  autoRefresh = true,
  maxLogs = 1000
}) => {
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('md'));
  
  const [logs, setLogs] = useState<LogEntry[]>(mockLogs);
  const [filteredLogs, setFilteredLogs] = useState<LogEntry[]>(mockLogs);
  const [filters, setFilters] = useState({
    level: 'all',
    source: 'all',
    container: 'all',
    search: '',
    timeRange: 'all'
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);
  const [showSettings, setShowSettings] = useState(false);
  const [autoScroll, setAutoScroll] = useState(true);
  const [logBuffer, setLogBuffer] = useState<LogEntry[]>([]);
  const [isStreaming, setIsStreaming] = useState(false);
  const [logCount, setLogCount] = useState(0);
  const [errorCount, setErrorCount] = useState(0);
  const [warningCount, setWarningCount] = useState(0);
  
  const logContainerRef = useRef<HTMLDivElement>(null);

  // Simulate real-time log streaming
  useEffect(() => {
    if (!isStreaming) return;

    const interval = setInterval(() => {
      const newLog: LogEntry = {
        id: Date.now(),
        timestamp: new Date().toISOString(),
        level: ['info', 'warning', 'error', 'debug'][Math.floor(Math.random() * 4)] as any,
        source: ['ros2-navigation', 'gazebo-sim', 'rviz-viz'][Math.floor(Math.random() * 3)],
        message: `Real-time log entry ${Math.random().toString(36).substring(7)}`,
        container: ['ros2-navigation', 'gazebo-sim', 'rviz-viz'][Math.floor(Math.random() * 3)]
      };

      setLogs(prev => {
        const updated = [newLog, ...prev.slice(0, maxLogs - 1)];
        return updated;
      });

      setLogCount(prev => prev + 1);
      if (newLog.level === 'error') setErrorCount(prev => prev + 1);
      if (newLog.level === 'warning') setWarningCount(prev => prev + 1);
    }, 2000);

    return () => clearInterval(interval);
  }, [isStreaming, maxLogs]);

  // Apply filters
  useEffect(() => {
    const filtered = logs.filter(log => {
      if (filters.level !== 'all' && log.level !== filters.level) return false;
      if (filters.source !== 'all' && log.source !== filters.source) return false;
      if (filters.container !== 'all' && log.container !== filters.container) return false;
      if (filters.search && !log.message.toLowerCase().includes(filters.search.toLowerCase())) return false;
      return true;
    });

    setFilteredLogs(filtered);
    
    if (onFilterChange) {
      onFilterChange(filters);
    }
  }, [logs, filters, onFilterChange]);

  // Auto-scroll to bottom
  useEffect(() => {
    if (autoScroll && logContainerRef.current) {
      logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight;
    }
  }, [filteredLogs, autoScroll]);

  const handleFilterChange = (key: string, value: string) => {
    setFilters(prev => ({ ...prev, [key]: value }));
  };

  const handleClearFilters = () => {
    setFilters({
      level: 'all',
      source: 'all',
      container: 'all',
      search: '',
      timeRange: 'all'
    });
  };

  const handleStartStreaming = () => {
    setIsStreaming(true);
    setSuccess('Log streaming started');
  };

  const handleStopStreaming = () => {
    setIsStreaming(false);
    setSuccess('Log streaming stopped');
  };

  const handleClearLogs = () => {
    setLogs([]);
    setLogCount(0);
    setErrorCount(0);
    setWarningCount(0);
    setSuccess('Logs cleared');
  };

  const handleExportLogs = () => {
    const logText = filteredLogs.map(log => 
      `${log.timestamp} [${log.level.toUpperCase()}] ${log.source}: ${log.message}`
    ).join('\n');
    
    const blob = new Blob([logText], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `logs_${new Date().toISOString().split('T')[0]}.txt`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    
    setSuccess('Logs exported successfully');
  };

  const getLevelColor = (level: string) => {
    switch (level) {
      case 'error': return 'error';
      case 'warning': return 'warning';
      case 'info': return 'info';
      case 'debug': return 'default';
      default: return 'default';
    }
  };

  const getLevelIcon = (level: string) => {
    switch (level) {
      case 'error': return <ErrorIcon color="error" />;
      case 'warning': return <WarningIcon color="warning" />;
      case 'info': return <InfoIcon color="info" />;
      case 'debug': return <CodeIcon />;
      default: return <InfoIcon />;
    }
  };

  const sources = Array.from(new Set(logs.map(log => log.source)));
  const containers = Array.from(new Set(logs.map(log => log.container)));

  return (
    <Box>
      {/* Log Statistics */}
      <Box sx={{ mb: 3 }}>
        <Box sx={{ display: 'flex', gap: 2, flexWrap: 'wrap', mb: 2 }}>
          <Chip 
            icon={<TimelineIcon />} 
            label={`Total: ${logCount}`} 
            color="primary" 
            variant="outlined"
          />
          <Chip 
            icon={<ErrorIcon />} 
            label={`Errors: ${errorCount}`} 
            color="error" 
            variant="outlined"
          />
          <Chip 
            icon={<WarningIcon />} 
            label={`Warnings: ${warningCount}`} 
            color="warning" 
            variant="outlined"
          />
          <Chip 
            icon={<InfoIcon />} 
            label={`Info: ${logCount - errorCount - warningCount}`} 
            color="info" 
            variant="outlined"
          />
        </Box>
      </Box>

      {/* Log Controls */}
      <Card sx={{ mb: 3 }}>
        <CardHeader
          title="Log Streaming Controls"
          subheader="Manage real-time log streaming and filtering"
          action={
            <Box sx={{ display: 'flex', gap: 1 }}>
              <Button
                variant={isStreaming ? "outlined" : "contained"}
                startIcon={isStreaming ? <PauseIcon /> : <TimelineIcon />}
                onClick={isStreaming ? handleStopStreaming : handleStartStreaming}
                color={isStreaming ? "warning" : "primary"}
              >
                {isStreaming ? 'Stop Streaming' : 'Start Streaming'}
              </Button>
              <Button
                variant="outlined"
                startIcon={<RefreshIcon />}
                onClick={() => window.location.reload()}
              >
                Refresh
              </Button>
              <Button
                variant="outlined"
                startIcon={<DownloadIcon />}
                onClick={handleExportLogs}
              >
                Export
              </Button>
              <Button
                variant="outlined"
                startIcon={<DeleteIcon />}
                onClick={handleClearLogs}
                color="error"
              >
                Clear
              </Button>
            </Box>
          }
        />
        <CardContent>
          {/* Filters */}
          <Box sx={{ mb: 3 }}>
            <Box sx={{ display: 'flex', gap: 2, flexWrap: 'wrap', alignItems: 'center', mb: 2 }}>
              <FormControl size="small" sx={{ minWidth: 120 }}>
                <InputLabel>Log Level</InputLabel>
                <Select
                  value={filters.level}
                  onChange={(e) => handleFilterChange('level', e.target.value)}
                  label="Log Level"
                >
                  <MenuItem value="all">All Levels</MenuItem>
                  <MenuItem value="error">Error</MenuItem>
                  <MenuItem value="warning">Warning</MenuItem>
                  <MenuItem value="info">Info</MenuItem>
                  <MenuItem value="debug">Debug</MenuItem>
                </Select>
              </FormControl>

              <FormControl size="small" sx={{ minWidth: 120 }}>
                <InputLabel>Source</InputLabel>
                <Select
                  value={filters.source}
                  onChange={(e) => handleFilterChange('source', e.target.value)}
                  label="Source"
                >
                  <MenuItem value="all">All Sources</MenuItem>
                  {sources.map(source => (
                    <MenuItem key={source} value={source}>{source}</MenuItem>
                  ))}
                </Select>
              </FormControl>

              <FormControl size="small" sx={{ minWidth: 120 }}>
                <InputLabel>Container</InputLabel>
                <Select
                  value={filters.container}
                  onChange={(e) => handleFilterChange('container', e.target.value)}
                  label="Container"
                >
                  <MenuItem value="all">All Containers</MenuItem>
                  {containers.map(container => (
                    <MenuItem key={container} value={container}>{container}</MenuItem>
                  ))}
                </Select>
              </FormControl>

              <TextField
                size="small"
                placeholder="Search logs..."
                value={filters.search}
                onChange={(e) => handleFilterChange('search', e.target.value)}
                InputProps={{
                  startAdornment: (
                    <InputAdornment position="start">
                      <SearchIcon />
                    </InputAdornment>
                  ),
                  endAdornment: filters.search && (
                    <InputAdornment position="end">
                      <IconButton
                        size="small"
                        onClick={() => handleFilterChange('search', '')}
                      >
                        <ClearIcon />
                      </IconButton>
                    </InputAdornment>
                  )
                }}
                sx={{ minWidth: 200 }}
              />

              <Button
                variant="outlined"
                startIcon={<FilterIcon />}
                onClick={handleClearFilters}
                size="small"
              >
                Clear Filters
              </Button>
            </Box>

            <FormControlLabel
              control={
                <Switch
                  checked={autoScroll}
                  onChange={(e) => setAutoScroll(e.target.checked)}
                />
              }
              label="Auto-scroll to latest logs"
            />
          </Box>
        </CardContent>
      </Card>

      {/* Log Display */}
      <Card>
        <CardHeader
          title={`Real-time Logs (${filteredLogs.length} entries)`}
          subheader="Live log streaming with filtering and search capabilities"
        />
        <CardContent>
          <Box
            ref={logContainerRef}
            sx={{
              maxHeight: 600,
              overflow: 'auto',
              border: 1,
              borderColor: 'divider',
              borderRadius: 1,
              bgcolor: 'grey.50'
            }}
          >
            {filteredLogs.length === 0 ? (
              <Box sx={{ p: 3, textAlign: 'center' }}>
                <Typography variant="body2" color="text.secondary">
                  No logs found matching the current filters
                </Typography>
              </Box>
            ) : (
              <List dense>
                {filteredLogs.map((log) => (
                  <ListItem
                    key={log.id}
                    sx={{
                      borderBottom: 1,
                      borderColor: 'divider',
                      '&:hover': { bgcolor: 'action.hover' },
                      '&:last-child': { borderBottom: 0 }
                    }}
                  >
                    <ListItemIcon>
                      {getLevelIcon(log.level)}
                    </ListItemIcon>
                    <ListItemText
                      primary={
                        <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 0.5 }}>
                          <Typography variant="body2" component="span">
                            {new Date(log.timestamp).toLocaleString()}
                          </Typography>
                          <Chip 
                            label={log.level.toUpperCase()} 
                            size="small"
                            color={getLevelColor(log.level) as any}
                          />
                          <Chip 
                            label={log.source} 
                            size="small"
                            variant="outlined"
                          />
                          <Chip 
                            label={log.container} 
                            size="small"
                            variant="outlined"
                            color="secondary"
                          />
                        </Box>
                      }
                      secondary={
                        <Typography variant="body2" sx={{ fontFamily: 'monospace' }}>
                          {log.message}
                        </Typography>
                      }
                    />
                    <ListItemSecondaryAction>
                      <Tooltip title="Copy log entry">
                        <IconButton
                          size="small"
                          onClick={() => {
                            navigator.clipboard.writeText(
                              `${log.timestamp} [${log.level.toUpperCase()}] ${log.source}: ${log.message}`
                            );
                            setSuccess('Log entry copied to clipboard');
                          }}
                        >
                          <CopyIcon />
                        </IconButton>
                      </Tooltip>
                    </ListItemSecondaryAction>
                  </ListItem>
                ))}
              </List>
            )}
          </Box>
        </CardContent>
      </Card>

      {/* Notifications */}
      <Snackbar
        open={!!error}
        autoHideDuration={6000}
        onClose={() => setError(null)}
      >
        <Alert onClose={() => setError(null)} severity="error">
          {error}
        </Alert>
      </Snackbar>

      <Snackbar
        open={!!success}
        autoHideDuration={6000}
        onClose={() => setSuccess(null)}
      >
        <Alert onClose={() => setSuccess(null)} severity="success">
          {success}
        </Alert>
      </Snackbar>
    </Box>
  );
};

export default LogStreaming; 