import React, { useState, useEffect } from 'react';
import {
  Box,
  Container,
  Typography,
  Paper,
  Tabs,
  Tab,
  Grid,
  Card,
  CardContent,
  CardHeader,
  IconButton,
  Chip,
  Alert,
  CircularProgress,
  Button,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  TextField,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
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
  Fab,
  Snackbar,
  LinearProgress,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  TablePagination,
  InputAdornment,
  Slider,
  RadioGroup,
  Radio,
  Checkbox,
  FormGroup,
  Stack,
  Avatar,
  Drawer,
  AppBar,
  Toolbar,
  useTheme,
  useMediaQuery,
} from '@mui/material';
import {
  PlayArrow as PlayIcon,
  Stop as StopIcon,
  Refresh as RefreshIcon,
  Settings as SettingsIcon,
  Terminal as TerminalIcon,
  BugReport as DebugIcon,
  Timeline as TimelineIcon,
  Memory as MemoryIcon,
  Storage as StorageIcon,
  Speed as SpeedIcon,
  Warning as WarningIcon,
  Error as ErrorIcon,
  Info as InfoIcon,
  CheckCircle as SuccessIcon,
  ExpandMore as ExpandMoreIcon,
  Add as AddIcon,
  Delete as DeleteIcon,
  Edit as EditIcon,
  Visibility as ViewIcon,
  Download as DownloadIcon,
  Upload as UploadIcon,
  Folder as FolderIcon,
  Code as CodeIcon,
  Build as BuildIcon,
  Cloud as CloudIcon,
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
  FilterList as FilterIcon,
  Search as SearchIcon,
  Clear as ClearIcon,
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
  BatteryFull as BatteryIcon,
} from '@mui/icons-material';

// Mock data for development
const mockContainers = [
  {
    id: 'container-1',
    name: 'ros2-navigation',
    status: 'running',
    image: 'ros2:latest',
    ports: ['8080:8080', '11311:11311'],
    cpu: 45.2,
    memory: 67.8,
    disk: 12.3,
    network: 2.1,
    uptime: '2h 15m',
    health: 'healthy',
  },
  {
    id: 'container-2',
    name: 'gazebo-sim',
    status: 'stopped',
    image: 'gazebo:latest',
    ports: ['11345:11345'],
    cpu: 0,
    memory: 0,
    disk: 8.7,
    network: 0,
    uptime: '0m',
    health: 'unknown',
  },
  {
    id: 'container-3',
    name: 'rviz-viz',
    status: 'running',
    image: 'rviz:latest',
    ports: ['8081:8081'],
    cpu: 23.1,
    memory: 34.5,
    disk: 5.2,
    network: 1.8,
    uptime: '1h 30m',
    health: 'healthy',
  },
];

const mockLogs = [
  {
    id: 1,
    timestamp: '2024-01-15T10:30:15Z',
    level: 'info',
    source: 'ros2-navigation',
    message: 'Navigation node started successfully',
  },
  {
    id: 2,
    timestamp: '2024-01-15T10:30:16Z',
    level: 'info',
    source: 'ros2-navigation',
    message: 'Loading navigation parameters',
  },
  {
    id: 3,
    timestamp: '2024-01-15T10:30:17Z',
    level: 'warning',
    source: 'gazebo-sim',
    message: 'Physics engine running at reduced performance',
  },
  {
    id: 4,
    timestamp: '2024-01-15T10:30:18Z',
    level: 'error',
    source: 'rviz-viz',
    message: 'Failed to connect to ROS master',
  },
  {
    id: 5,
    timestamp: '2024-01-15T10:30:19Z',
    level: 'info',
    source: 'ros2-navigation',
    message: 'Map loaded successfully',
  },
];

const mockRosbags = [
  {
    id: 1,
    name: 'navigation_test_2024-01-15.bag',
    size: '256MB',
    duration: '15m 30s',
    topics: 8,
    messages: 15420,
    date: '2024-01-15',
  },
  {
    id: 2,
    name: 'sensor_data_2024-01-14.bag',
    size: '512MB',
    duration: '32m 15s',
    topics: 12,
    messages: 28940,
    date: '2024-01-14',
  },
  {
    id: 3,
    name: 'debug_session_2024-01-13.bag',
    size: '128MB',
    duration: '8m 45s',
    topics: 6,
    messages: 8230,
    date: '2024-01-13',
  },
];

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
      id={`execution-tabpanel-${index}`}
      aria-labelledby={`execution-tab-${index}`}
      {...other}
    >
      {value === index && <Box sx={{ p: 3 }}>{children}</Box>}
    </div>
  );
}

function a11yProps(index: number) {
  return {
    id: `execution-tab-${index}`,
    'aria-controls': `execution-tabpanel-${index}`,
  };
}

const ExecutionEnvironment: React.FC = () => {
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('md'));

  const [tabValue, setTabValue] = useState(0);
  const [containers, setContainers] = useState(mockContainers);
  const [logs, setLogs] = useState(mockLogs);
  const [rosbags, setRosbags] = useState(mockRosbags);
  const [selectedContainer, setSelectedContainer] = useState<string | null>(
    null
  );
  const [logFilter, setLogFilter] = useState({
    level: 'all',
    source: 'all',
    search: '',
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);
  const [showSettings, setShowSettings] = useState(false);
  const [showTerminal, setShowTerminal] = useState(false);
  const [showDebugger, setShowDebugger] = useState(false);
  const [showRosbagPlayer, setShowRosbagPlayer] = useState(false);

  const handleTabChange = (event: React.SyntheticEvent, newValue: number) => {
    setTabValue(newValue);
  };

  const handleContainerAction = async (
    containerId: string,
    action: 'start' | 'stop' | 'restart' | 'delete'
  ) => {
    setIsLoading(true);
    setError(null);

    try {
      // Simulate API call
      await new Promise((resolve) => setTimeout(resolve, 1000));

      setContainers((prev) =>
        prev.map((container) => {
          if (container.id === containerId) {
            switch (action) {
              case 'start':
                return { ...container, status: 'running', uptime: '0m' };
              case 'stop':
                return {
                  ...container,
                  status: 'stopped',
                  uptime: '0m',
                  cpu: 0,
                  memory: 0,
                  network: 0,
                };
              case 'restart':
                return { ...container, status: 'running', uptime: '0m' };
              default:
                return container;
            }
          }
          return container;
        })
      );

      setSuccess(`Container ${action}ed successfully`);
    } catch (err) {
      setError(`Failed to ${action} container: ${err}`);
    } finally {
      setIsLoading(false);
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'running':
        return 'success';
      case 'stopped':
        return 'error';
      case 'starting':
        return 'warning';
      case 'stopping':
        return 'warning';
      default:
        return 'default';
    }
  };

  const getHealthColor = (health: string) => {
    switch (health) {
      case 'healthy':
        return 'success';
      case 'unhealthy':
        return 'error';
      case 'starting':
        return 'warning';
      default:
        return 'default';
    }
  };

  const filteredLogs = logs.filter((log) => {
    if (logFilter.level !== 'all' && log.level !== logFilter.level)
      return false;
    if (logFilter.source !== 'all' && log.source !== logFilter.source)
      return false;
    if (
      logFilter.search &&
      !log.message.toLowerCase().includes(logFilter.search.toLowerCase())
    )
      return false;
    return true;
  });

  return (
    <Container maxWidth="xl">
      {/* Header */}
      <Box sx={{ mb: 4 }}>
        <Typography variant="h4" component="h1" gutterBottom>
          Execution Environment
        </Typography>
        <Typography variant="body1" color="text.secondary">
          Manage containers, monitor resources, and debug your robotics
          applications
        </Typography>
      </Box>

      {/* Status Overview */}
      <Grid container spacing={3} sx={{ mb: 4 }}>
        <Grid item xs={12} sm={6} md={3}>
          <Card>
            <CardContent>
              <Box
                sx={{
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'space-between',
                }}
              >
                <Box>
                  <Typography color="text.secondary" gutterBottom>
                    Running Containers
                  </Typography>
                  <Typography variant="h4" component="div">
                    {containers.filter((c) => c.status === 'running').length}
                  </Typography>
                </Box>
                <CloudIcon color="primary" sx={{ fontSize: 40 }} />
              </Box>
            </CardContent>
          </Card>
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <Card>
            <CardContent>
              <Box
                sx={{
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'space-between',
                }}
              >
                <Box>
                  <Typography color="text.secondary" gutterBottom>
                    Total CPU Usage
                  </Typography>
                  <Typography variant="h4" component="div">
                    {containers.reduce((sum, c) => sum + c.cpu, 0).toFixed(1)}%
                  </Typography>
                </Box>
                <SpeedIcon color="primary" sx={{ fontSize: 40 }} />
              </Box>
            </CardContent>
          </Card>
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <Card>
            <CardContent>
              <Box
                sx={{
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'space-between',
                }}
              >
                <Box>
                  <Typography color="text.secondary" gutterBottom>
                    Memory Usage
                  </Typography>
                  <Typography variant="h4" component="div">
                    {containers
                      .reduce((sum, c) => sum + c.memory, 0)
                      .toFixed(1)}
                    %
                  </Typography>
                </Box>
                <MemoryIcon color="primary" sx={{ fontSize: 40 }} />
              </Box>
            </CardContent>
          </Card>
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <Card>
            <CardContent>
              <Box
                sx={{
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'space-between',
                }}
              >
                <Box>
                  <Typography color="text.secondary" gutterBottom>
                    Active Logs
                  </Typography>
                  <Typography variant="h4" component="div">
                    {logs.length}
                  </Typography>
                </Box>
                <TimelineIcon color="primary" sx={{ fontSize: 40 }} />
              </Box>
            </CardContent>
          </Card>
        </Grid>
      </Grid>

      {/* Main Content */}
      <Paper sx={{ width: '100%' }}>
        <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
          <Tabs
            value={tabValue}
            onChange={handleTabChange}
            aria-label="execution environment tabs"
            variant={isMobile ? 'scrollable' : 'fullWidth'}
            scrollButtons={isMobile ? 'auto' : false}
          >
            <Tab
              label="Containers"
              icon={<CloudIcon />}
              iconPosition="start"
              {...a11yProps(0)}
            />
            <Tab
              label="Terminal"
              icon={<TerminalIcon />}
              iconPosition="start"
              {...a11yProps(1)}
            />
            <Tab
              label="Logs"
              icon={<TimelineIcon />}
              iconPosition="start"
              {...a11yProps(2)}
            />
            <Tab
              label="Resources"
              icon={<MemoryIcon />}
              iconPosition="start"
              {...a11yProps(3)}
            />
            <Tab
              label="Rosbag"
              icon={<FolderIcon />}
              iconPosition="start"
              {...a11yProps(4)}
            />
            <Tab
              label="Debug"
              icon={<DebugIcon />}
              iconPosition="start"
              {...a11yProps(5)}
            />
          </Tabs>
        </Box>

        {/* Containers Tab */}
        <TabPanel value={tabValue} index={0}>
          <Box sx={{ mb: 3 }}>
            <Typography variant="h6" gutterBottom>
              Container Management
            </Typography>
            <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
              Manage your Docker containers and monitor their status
            </Typography>
          </Box>

          <Grid container spacing={3}>
            {containers.map((container) => (
              <Grid item xs={12} md={6} lg={4} key={container.id}>
                <Card>
                  <CardHeader
                    title={container.name}
                    subheader={`Image: ${container.image}`}
                    action={
                      <Box>
                        <Chip
                          label={container.status}
                          color={getStatusColor(container.status) as any}
                          size="small"
                          sx={{ mr: 1 }}
                        />
                        <Chip
                          label={container.health}
                          color={getHealthColor(container.health) as any}
                          size="small"
                        />
                      </Box>
                    }
                  />
                  <CardContent>
                    <Box sx={{ mb: 2 }}>
                      <Typography variant="body2" color="text.secondary">
                        Ports: {container.ports.join(', ')}
                      </Typography>
                      <Typography variant="body2" color="text.secondary">
                        Uptime: {container.uptime}
                      </Typography>
                    </Box>

                    <Box sx={{ mb: 2 }}>
                      <Typography variant="body2" gutterBottom>
                        CPU: {container.cpu}%
                      </Typography>
                      <LinearProgress
                        variant="determinate"
                        value={container.cpu}
                        sx={{ mb: 1 }}
                      />

                      <Typography variant="body2" gutterBottom>
                        Memory: {container.memory}%
                      </Typography>
                      <LinearProgress
                        variant="determinate"
                        value={container.memory}
                        sx={{ mb: 1 }}
                      />

                      <Typography variant="body2" gutterBottom>
                        Network: {container.network} MB/s
                      </Typography>
                      <LinearProgress
                        variant="determinate"
                        value={Math.min(container.network * 10, 100)}
                      />
                    </Box>

                    <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                      <Button
                        size="small"
                        variant="outlined"
                        startIcon={<PlayIcon />}
                        onClick={() =>
                          handleContainerAction(container.id, 'start')
                        }
                        disabled={container.status === 'running' || isLoading}
                      >
                        Start
                      </Button>
                      <Button
                        size="small"
                        variant="outlined"
                        startIcon={<StopIcon />}
                        onClick={() =>
                          handleContainerAction(container.id, 'stop')
                        }
                        disabled={container.status === 'stopped' || isLoading}
                      >
                        Stop
                      </Button>
                      <Button
                        size="small"
                        variant="outlined"
                        startIcon={<RestartIcon />}
                        onClick={() =>
                          handleContainerAction(container.id, 'restart')
                        }
                        disabled={isLoading}
                      >
                        Restart
                      </Button>
                      <Button
                        size="small"
                        variant="outlined"
                        startIcon={<TerminalIcon />}
                        onClick={() => {
                          setSelectedContainer(container.id);
                          setShowTerminal(true);
                        }}
                      >
                        Terminal
                      </Button>
                    </Box>
                  </CardContent>
                </Card>
              </Grid>
            ))}
          </Grid>
        </TabPanel>

        {/* Terminal Tab */}
        <TabPanel value={tabValue} index={1}>
          <Box sx={{ mb: 3 }}>
            <Typography variant="h6" gutterBottom>
              Web Terminal
            </Typography>
            <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
              Access container terminals and execute commands
            </Typography>
          </Box>

          <Card>
            <CardContent>
              <Box sx={{ mb: 3 }}>
                <FormControl fullWidth sx={{ mb: 2 }}>
                  <InputLabel>Select Container</InputLabel>
                  <Select
                    value={selectedContainer || ''}
                    onChange={(e) => setSelectedContainer(e.target.value)}
                    label="Select Container"
                  >
                    {containers.map((container) => (
                      <MenuItem key={container.id} value={container.id}>
                        {container.name} ({container.status})
                      </MenuItem>
                    ))}
                  </Select>
                </FormControl>

                {selectedContainer && (
                  <Box
                    sx={{
                      border: 1,
                      borderColor: 'divider',
                      borderRadius: 1,
                      p: 2,
                      minHeight: 300,
                    }}
                  >
                    <Typography
                      variant="body2"
                      color="text.secondary"
                      sx={{ mb: 2 }}
                    >
                      Terminal for{' '}
                      {containers.find((c) => c.id === selectedContainer)?.name}
                    </Typography>
                    <Box
                      sx={{
                        bgcolor: 'black',
                        color: 'lime',
                        p: 2,
                        borderRadius: 1,
                        fontFamily: 'monospace',
                        minHeight: 200,
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'center',
                      }}
                    >
                      <Typography variant="body2">
                        Terminal interface will be implemented with xterm.js
                      </Typography>
                    </Box>
                  </Box>
                )}
              </Box>
            </CardContent>
          </Card>
        </TabPanel>

        {/* Logs Tab */}
        <TabPanel value={tabValue} index={2}>
          <Box sx={{ mb: 3 }}>
            <Typography variant="h6" gutterBottom>
              Real-time Logs
            </Typography>
            <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
              Monitor application logs with filtering and search capabilities
            </Typography>
          </Box>

          <Card>
            <CardContent>
              {/* Log Filters */}
              <Box sx={{ mb: 3 }}>
                <Grid container spacing={2} alignItems="center">
                  <Grid item xs={12} sm={3}>
                    <FormControl fullWidth size="small">
                      <InputLabel>Log Level</InputLabel>
                      <Select
                        value={logFilter.level}
                        onChange={(e) =>
                          setLogFilter((prev) => ({
                            ...prev,
                            level: e.target.value,
                          }))
                        }
                        label="Log Level"
                      >
                        <MenuItem value="all">All Levels</MenuItem>
                        <MenuItem value="error">Error</MenuItem>
                        <MenuItem value="warning">Warning</MenuItem>
                        <MenuItem value="info">Info</MenuItem>
                        <MenuItem value="debug">Debug</MenuItem>
                      </Select>
                    </FormControl>
                  </Grid>
                  <Grid item xs={12} sm={3}>
                    <FormControl fullWidth size="small">
                      <InputLabel>Source</InputLabel>
                      <Select
                        value={logFilter.source}
                        onChange={(e) =>
                          setLogFilter((prev) => ({
                            ...prev,
                            source: e.target.value,
                          }))
                        }
                        label="Source"
                      >
                        <MenuItem value="all">All Sources</MenuItem>
                        {Array.from(new Set(logs.map((log) => log.source))).map(
                          (source) => (
                            <MenuItem key={source} value={source}>
                              {source}
                            </MenuItem>
                          )
                        )}
                      </Select>
                    </FormControl>
                  </Grid>
                  <Grid item xs={12} sm={6}>
                    <TextField
                      fullWidth
                      size="small"
                      placeholder="Search logs..."
                      value={logFilter.search}
                      onChange={(e) =>
                        setLogFilter((prev) => ({
                          ...prev,
                          search: e.target.value,
                        }))
                      }
                      InputProps={{
                        startAdornment: (
                          <InputAdornment position="start">
                            <SearchIcon />
                          </InputAdornment>
                        ),
                        endAdornment: logFilter.search && (
                          <InputAdornment position="end">
                            <IconButton
                              size="small"
                              onClick={() =>
                                setLogFilter((prev) => ({
                                  ...prev,
                                  search: '',
                                }))
                              }
                            >
                              <ClearIcon />
                            </IconButton>
                          </InputAdornment>
                        ),
                      }}
                    />
                  </Grid>
                </Grid>
              </Box>

              {/* Log List */}
              <Box sx={{ maxHeight: 400, overflow: 'auto' }}>
                {filteredLogs.map((log) => (
                  <Box
                    key={log.id}
                    sx={{
                      p: 2,
                      borderBottom: 1,
                      borderColor: 'divider',
                      '&:hover': { bgcolor: 'action.hover' },
                    }}
                  >
                    <Box
                      sx={{
                        display: 'flex',
                        alignItems: 'center',
                        gap: 2,
                        mb: 1,
                      }}
                    >
                      {log.level === 'error' && <ErrorIcon color="error" />}
                      {log.level === 'warning' && (
                        <WarningIcon color="warning" />
                      )}
                      {log.level === 'info' && <InfoIcon color="info" />}
                      {log.level === 'debug' && <CodeIcon />}

                      <Typography variant="body2" color="text.secondary">
                        {new Date(log.timestamp).toLocaleString()}
                      </Typography>

                      <Chip
                        label={log.level.toUpperCase()}
                        size="small"
                        color={
                          log.level === 'error'
                            ? 'error'
                            : log.level === 'warning'
                              ? 'warning'
                              : 'default'
                        }
                      />

                      <Chip
                        label={log.source}
                        size="small"
                        variant="outlined"
                      />
                    </Box>
                    <Typography variant="body2">{log.message}</Typography>
                  </Box>
                ))}
              </Box>
            </CardContent>
          </Card>
        </TabPanel>

        {/* Resources Tab */}
        <TabPanel value={tabValue} index={3}>
          <Box sx={{ mb: 3 }}>
            <Typography variant="h6" gutterBottom>
              Resource Monitoring
            </Typography>
            <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
              Monitor system resources and performance metrics
            </Typography>
          </Box>

          <Grid container spacing={3}>
            <Grid item xs={12} md={6}>
              <Card>
                <CardHeader title="CPU Usage" />
                <CardContent>
                  <Box sx={{ mb: 2 }}>
                    <Typography variant="h4" gutterBottom>
                      {containers.reduce((sum, c) => sum + c.cpu, 0).toFixed(1)}
                      %
                    </Typography>
                    <LinearProgress
                      variant="determinate"
                      value={containers.reduce((sum, c) => sum + c.cpu, 0)}
                      sx={{ height: 8, borderRadius: 4 }}
                    />
                  </Box>

                  {containers.map((container) => (
                    <Box key={container.id} sx={{ mb: 1 }}>
                      <Box
                        sx={{
                          display: 'flex',
                          justifyContent: 'space-between',
                          mb: 0.5,
                        }}
                      >
                        <Typography variant="body2">
                          {container.name}
                        </Typography>
                        <Typography variant="body2">
                          {container.cpu}%
                        </Typography>
                      </Box>
                      <LinearProgress
                        variant="determinate"
                        value={container.cpu}
                        sx={{ height: 4, borderRadius: 2 }}
                      />
                    </Box>
                  ))}
                </CardContent>
              </Card>
            </Grid>

            <Grid item xs={12} md={6}>
              <Card>
                <CardHeader title="Memory Usage" />
                <CardContent>
                  <Box sx={{ mb: 2 }}>
                    <Typography variant="h4" gutterBottom>
                      {containers
                        .reduce((sum, c) => sum + c.memory, 0)
                        .toFixed(1)}
                      %
                    </Typography>
                    <LinearProgress
                      variant="determinate"
                      value={containers.reduce((sum, c) => sum + c.memory, 0)}
                      sx={{ height: 8, borderRadius: 4 }}
                    />
                  </Box>

                  {containers.map((container) => (
                    <Box key={container.id} sx={{ mb: 1 }}>
                      <Box
                        sx={{
                          display: 'flex',
                          justifyContent: 'space-between',
                          mb: 0.5,
                        }}
                      >
                        <Typography variant="body2">
                          {container.name}
                        </Typography>
                        <Typography variant="body2">
                          {container.memory}%
                        </Typography>
                      </Box>
                      <LinearProgress
                        variant="determinate"
                        value={container.memory}
                        sx={{ height: 4, borderRadius: 2 }}
                      />
                    </Box>
                  ))}
                </CardContent>
              </Card>
            </Grid>

            <Grid item xs={12}>
              <Card>
                <CardHeader title="Network Usage" />
                <CardContent>
                  <Grid container spacing={2}>
                    {containers.map((container) => (
                      <Grid item xs={12} sm={6} md={4} key={container.id}>
                        <Box
                          sx={{
                            p: 2,
                            border: 1,
                            borderColor: 'divider',
                            borderRadius: 1,
                          }}
                        >
                          <Typography variant="subtitle2" gutterBottom>
                            {container.name}
                          </Typography>
                          <Typography variant="h6" color="primary">
                            {container.network} MB/s
                          </Typography>
                        </Box>
                      </Grid>
                    ))}
                  </Grid>
                </CardContent>
              </Card>
            </Grid>
          </Grid>
        </TabPanel>

        {/* Rosbag Tab */}
        <TabPanel value={tabValue} index={4}>
          <Box sx={{ mb: 3 }}>
            <Typography variant="h6" gutterBottom>
              Rosbag Management
            </Typography>
            <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
              Manage and play ROS bag files for data analysis and debugging
            </Typography>
          </Box>

          <Card>
            <CardContent>
              <Box sx={{ mb: 3 }}>
                <Button
                  variant="contained"
                  startIcon={<AddIcon />}
                  onClick={() => setShowRosbagPlayer(true)}
                >
                  Upload Rosbag
                </Button>
              </Box>

              <TableContainer>
                <Table>
                  <TableHead>
                    <TableRow>
                      <TableCell>Name</TableCell>
                      <TableCell>Size</TableCell>
                      <TableCell>Duration</TableCell>
                      <TableCell>Topics</TableCell>
                      <TableCell>Messages</TableCell>
                      <TableCell>Date</TableCell>
                      <TableCell>Actions</TableCell>
                    </TableRow>
                  </TableHead>
                  <TableBody>
                    {rosbags.map((rosbag) => (
                      <TableRow key={rosbag.id}>
                        <TableCell>{rosbag.name}</TableCell>
                        <TableCell>{rosbag.size}</TableCell>
                        <TableCell>{rosbag.duration}</TableCell>
                        <TableCell>{rosbag.topics}</TableCell>
                        <TableCell>
                          {rosbag.messages.toLocaleString()}
                        </TableCell>
                        <TableCell>{rosbag.date}</TableCell>
                        <TableCell>
                          <Box sx={{ display: 'flex', gap: 1 }}>
                            <IconButton
                              size="small"
                              onClick={() => setShowRosbagPlayer(true)}
                            >
                              <PlayIcon />
                            </IconButton>
                            <IconButton size="small">
                              <DownloadIcon />
                            </IconButton>
                            <IconButton size="small">
                              <DeleteIcon />
                            </IconButton>
                          </Box>
                        </TableCell>
                      </TableRow>
                    ))}
                  </TableBody>
                </Table>
              </TableContainer>
            </CardContent>
          </Card>
        </TabPanel>

        {/* Debug Tab */}
        <TabPanel value={tabValue} index={5}>
          <Box sx={{ mb: 3 }}>
            <Typography variant="h6" gutterBottom>
              Debugging Tools
            </Typography>
            <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
              Access debugging tools and performance analysis
            </Typography>
          </Box>

          <Grid container spacing={3}>
            <Grid item xs={12} md={6}>
              <Card>
                <CardHeader title="GDB Debugger" />
                <CardContent>
                  <Typography
                    variant="body2"
                    color="text.secondary"
                    sx={{ mb: 2 }}
                  >
                    Attach GDB debugger to running processes
                  </Typography>
                  <Button
                    variant="outlined"
                    startIcon={<DebugIcon />}
                    onClick={() => setShowDebugger(true)}
                  >
                    Start Debug Session
                  </Button>
                </CardContent>
              </Card>
            </Grid>

            <Grid item xs={12} md={6}>
              <Card>
                <CardHeader title="Valgrind Analysis" />
                <CardContent>
                  <Typography
                    variant="body2"
                    color="text.secondary"
                    sx={{ mb: 2 }}
                  >
                    Run memory analysis and profiling
                  </Typography>
                  <Button variant="outlined" startIcon={<MemoryIcon />}>
                    Start Analysis
                  </Button>
                </CardContent>
              </Card>
            </Grid>

            <Grid item xs={12} md={6}>
              <Card>
                <CardHeader title="RViz Visualization" />
                <CardContent>
                  <Typography
                    variant="body2"
                    color="text.secondary"
                    sx={{ mb: 2 }}
                  >
                    Launch RViz for 3D visualization
                  </Typography>
                  <Button variant="outlined" startIcon={<ViewIcon />}>
                    Launch RViz
                  </Button>
                </CardContent>
              </Card>
            </Grid>

            <Grid item xs={12} md={6}>
              <Card>
                <CardHeader title="Gazebo Simulation" />
                <CardContent>
                  <Typography
                    variant="body2"
                    color="text.secondary"
                    sx={{ mb: 2 }}
                  >
                    Launch Gazebo simulation environment
                  </Typography>
                  <Button variant="outlined" startIcon={<BuildIcon />}>
                    Launch Gazebo
                  </Button>
                </CardContent>
              </Card>
            </Grid>
          </Grid>
        </TabPanel>
      </Paper>

      {/* Floating Action Button */}
      <Fab
        color="primary"
        aria-label="add"
        sx={{ position: 'fixed', bottom: 16, right: 16 }}
        onClick={() => setShowSettings(true)}
      >
        <SettingsIcon />
      </Fab>

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

      {/* Settings Dialog */}
      <Dialog
        open={showSettings}
        onClose={() => setShowSettings(false)}
        maxWidth="md"
        fullWidth
      >
        <DialogTitle>Execution Environment Settings</DialogTitle>
        <DialogContent>
          <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
            Configure execution environment settings and preferences
          </Typography>
          {/* Settings content will be implemented */}
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowSettings(false)}>Cancel</Button>
          <Button onClick={() => setShowSettings(false)} variant="contained">
            Save Settings
          </Button>
        </DialogActions>
      </Dialog>

      {/* Terminal Dialog */}
      <Dialog
        open={showTerminal}
        onClose={() => setShowTerminal(false)}
        maxWidth="lg"
        fullWidth
      >
        <DialogTitle>
          Terminal - {containers.find((c) => c.id === selectedContainer)?.name}
        </DialogTitle>
        <DialogContent>
          <Box
            sx={{
              bgcolor: 'black',
              color: 'lime',
              p: 2,
              borderRadius: 1,
              fontFamily: 'monospace',
              minHeight: 400,
            }}
          >
            <Typography variant="body2">
              Terminal interface will be implemented with xterm.js
            </Typography>
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowTerminal(false)}>Close</Button>
        </DialogActions>
      </Dialog>

      {/* Debugger Dialog */}
      <Dialog
        open={showDebugger}
        onClose={() => setShowDebugger(false)}
        maxWidth="lg"
        fullWidth
      >
        <DialogTitle>GDB Debugger</DialogTitle>
        <DialogContent>
          <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
            Debug interface will be implemented
          </Typography>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowDebugger(false)}>Close</Button>
        </DialogActions>
      </Dialog>

      {/* Rosbag Player Dialog */}
      <Dialog
        open={showRosbagPlayer}
        onClose={() => setShowRosbagPlayer(false)}
        maxWidth="md"
        fullWidth
      >
        <DialogTitle>Rosbag Player</DialogTitle>
        <DialogContent>
          <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
            Rosbag player interface will be implemented
          </Typography>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowRosbagPlayer(false)}>Close</Button>
        </DialogActions>
      </Dialog>
    </Container>
  );
};

export default ExecutionEnvironment;
