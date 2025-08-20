import React, { useState, useEffect } from 'react';
import {
  Box,
  Card,
  CardContent,
  CardHeader,
  Typography,
  Button,
  IconButton,
  Chip,
  LinearProgress,
  Grid,
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
  Alert,
  Snackbar,
  CircularProgress,
  Tooltip,
  Divider,
  List,
  ListItem,
  ListItemText,
  ListItemIcon,
  ListItemSecondaryAction,
  Badge,
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
  useTheme,
  useMediaQuery
} from '@mui/material';
import {
  PlayArrow as PlayIcon,
  Stop as StopIcon,
  Refresh as RefreshIcon,
  Pause as PauseIcon,
  SkipNext as SkipNextIcon,
  SkipPrevious as SkipPreviousIcon,
  Settings as SettingsIcon,
  Cloud as CloudIcon,
  Memory as MemoryIcon,
  Speed as SpeedIcon,
  Storage as StorageIcon,
  NetworkCheck as NetworkIcon,
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
  Security as SecurityIcon,
  Notifications as NotificationsIcon,
  Help as HelpIcon,
  Close as CloseIcon,
  Save as SaveIcon,
  Cancel as CancelIcon,
  RestartAlt as RestartIcon,
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
  Private as PrivateIcon,
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
  Timeline as TimelineIcon,
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
  BatteryChargingFull as BatteryChargingIcon,
  BatteryStd as BatteryStdIcon,
  BatteryUnknown as BatteryUnknownIcon,
  BatteryAlert as BatteryAlertIcon,
  BatteryLow as BatteryLowIcon,
  BatteryVeryLow as BatteryVeryLowIcon,
  BatterySaver as BatterySaverIcon,
  Battery90 as Battery90Icon,
  Battery80 as Battery80Icon,
  Battery60 as Battery60Icon,
  Battery50 as Battery50Icon,
  Battery30 as Battery30Icon,
  Battery20 as Battery20Icon,
  Battery10 as Battery10Icon,
  Battery6Bar as Battery6BarIcon,
  Battery5Bar as Battery5BarIcon,
  Battery4Bar as Battery4BarIcon,
  Battery3Bar as Battery3BarIcon,
  Battery2Bar as Battery2BarIcon,
  Battery1Bar as Battery1BarIcon,
  Battery0Bar as Battery0BarIcon,
  BatteryCharging20 as BatteryCharging20Icon,
  BatteryCharging30 as BatteryCharging30Icon,
  BatteryCharging50 as BatteryCharging50Icon,
  BatteryCharging60 as BatteryCharging60Icon,
  BatteryCharging80 as BatteryCharging80Icon,
  BatteryCharging90 as BatteryCharging90Icon,
  BatteryChargingFull as BatteryChargingFullIcon,
  BatteryChargingStd as BatteryChargingStdIcon,
  BatteryChargingUnknown as BatteryChargingUnknownIcon,
  BatteryChargingAlert as BatteryChargingAlertIcon,
  BatteryChargingLow as BatteryChargingLowIcon,
  BatteryChargingVeryLow as BatteryChargingVeryLowIcon,
  BatteryChargingSaver as BatteryChargingSaverIcon,
  BatteryCharging90 as BatteryCharging90Icon,
  BatteryCharging80 as BatteryCharging80Icon,
  BatteryCharging60 as BatteryCharging60Icon,
  BatteryCharging50 as BatteryCharging50Icon,
  BatteryCharging30 as BatteryCharging30Icon,
  BatteryCharging20 as BatteryCharging20Icon,
  BatteryCharging6Bar as BatteryCharging6BarIcon,
  BatteryCharging5Bar as BatteryCharging5BarIcon,
  BatteryCharging4Bar as BatteryCharging4BarIcon,
  BatteryCharging3Bar as BatteryCharging3BarIcon,
  BatteryCharging2Bar as BatteryCharging2BarIcon,
  BatteryCharging1Bar as BatteryCharging1BarIcon,
  BatteryCharging0Bar as BatteryCharging0BarIcon
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
    environment: 'development',
    autoRestart: true,
    resourceLimits: {
      cpu: '2.0',
      memory: '4GB',
      disk: '10GB'
    }
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
    environment: 'development',
    autoRestart: false,
    resourceLimits: {
      cpu: '4.0',
      memory: '8GB',
      disk: '20GB'
    }
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
    environment: 'development',
    autoRestart: true,
    resourceLimits: {
      cpu: '1.0',
      memory: '2GB',
      disk: '5GB'
    }
  }
];

interface Container {
  id: string;
  name: string;
  status: 'running' | 'stopped' | 'starting' | 'stopping' | 'error';
  image: string;
  ports: string[];
  cpu: number;
  memory: number;
  disk: number;
  network: number;
  uptime: string;
  health: 'healthy' | 'unhealthy' | 'starting' | 'unknown';
  environment: string;
  autoRestart: boolean;
  resourceLimits: {
    cpu: string;
    memory: string;
    disk: string;
  };
}

interface ExecutionControlPanelProps {
  onContainerAction?: (containerId: string, action: string) => void;
  onSettingsChange?: (settings: any) => void;
}

const ExecutionControlPanel: React.FC<ExecutionControlPanelProps> = ({
  onContainerAction,
  onSettingsChange
}) => {
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('md'));
  
  const [containers, setContainers] = useState<Container[]>(mockContainers);
  const [selectedContainer, setSelectedContainer] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);
  const [showSettings, setShowSettings] = useState(false);
  const [showContainerDetails, setShowContainerDetails] = useState(false);
  const [bulkSelection, setBulkSelection] = useState<string[]>([]);
  const [filterStatus, setFilterStatus] = useState<string>('all');
  const [searchTerm, setSearchTerm] = useState<string>('');

  const handleContainerAction = async (containerId: string, action: 'start' | 'stop' | 'restart' | 'delete' | 'pause' | 'resume') => {
    setIsLoading(true);
    setError(null);
    
    try {
      // Simulate API call
      await new Promise(resolve => setTimeout(resolve, 1000));
      
      setContainers(prev => prev.map(container => {
        if (container.id === containerId) {
          switch (action) {
            case 'start':
              return { ...container, status: 'running', uptime: '0m' };
            case 'stop':
              return { ...container, status: 'stopped', uptime: '0m', cpu: 0, memory: 0, network: 0 };
            case 'restart':
              return { ...container, status: 'running', uptime: '0m' };
            case 'pause':
              return { ...container, status: 'stopped', uptime: container.uptime };
            case 'resume':
              return { ...container, status: 'running', uptime: container.uptime };
            default:
              return container;
          }
        }
        return container;
      }));
      
      setSuccess(`Container ${action}ed successfully`);
      
      // Call parent callback if provided
      if (onContainerAction) {
        onContainerAction(containerId, action);
      }
    } catch (err) {
      setError(`Failed to ${action} container: ${err}`);
    } finally {
      setIsLoading(false);
    }
  };

  const handleBulkAction = async (action: 'start' | 'stop' | 'restart' | 'delete') => {
    if (bulkSelection.length === 0) return;
    
    setIsLoading(true);
    setError(null);
    
    try {
      // Simulate bulk API call
      await new Promise(resolve => setTimeout(resolve, 2000));
      
      setContainers(prev => prev.map(container => {
        if (bulkSelection.includes(container.id)) {
          switch (action) {
            case 'start':
              return { ...container, status: 'running', uptime: '0m' };
            case 'stop':
              return { ...container, status: 'stopped', uptime: '0m', cpu: 0, memory: 0, network: 0 };
            case 'restart':
              return { ...container, status: 'running', uptime: '0m' };
            default:
              return container;
          }
        }
        return container;
      }));
      
      setSuccess(`${bulkSelection.length} containers ${action}ed successfully`);
      setBulkSelection([]);
    } catch (err) {
      setError(`Failed to ${action} containers: ${err}`);
    } finally {
      setIsLoading(false);
    }
  };

  const handleBulkSelection = (containerId: string) => {
    setBulkSelection(prev => 
      prev.includes(containerId) 
        ? prev.filter(id => id !== containerId)
        : [...prev, containerId]
    );
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'running': return 'success';
      case 'stopped': return 'error';
      case 'starting': return 'warning';
      case 'stopping': return 'warning';
      case 'error': return 'error';
      default: return 'default';
    }
  };

  const getHealthColor = (health: string) => {
    switch (health) {
      case 'healthy': return 'success';
      case 'unhealthy': return 'error';
      case 'starting': return 'warning';
      default: return 'default';
    }
  };

  const filteredContainers = containers.filter(container => {
    if (filterStatus !== 'all' && container.status !== filterStatus) return false;
    if (searchTerm && !container.name.toLowerCase().includes(searchTerm.toLowerCase())) return false;
    return true;
  });

  const runningContainers = containers.filter(c => c.status === 'running').length;
  const totalContainers = containers.length;
  const totalCpuUsage = containers.reduce((sum, c) => sum + c.cpu, 0);
  const totalMemoryUsage = containers.reduce((sum, c) => sum + c.memory, 0);

  return (
    <Box>
      {/* System Overview */}
      <Grid container spacing={3} sx={{ mb: 4 }}>
        <Grid item xs={12} sm={6} md={3}>
          <Card>
            <CardContent>
              <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
                <Box>
                  <Typography color="text.secondary" gutterBottom>
                    Running Containers
                  </Typography>
                  <Typography variant="h4" component="div">
                    {runningContainers}/{totalContainers}
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
              <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
                <Box>
                  <Typography color="text.secondary" gutterBottom>
                    Total CPU Usage
                  </Typography>
                  <Typography variant="h4" component="div">
                    {totalCpuUsage.toFixed(1)}%
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
              <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
                <Box>
                  <Typography color="text.secondary" gutterBottom>
                    Memory Usage
                  </Typography>
                  <Typography variant="h4" component="div">
                    {totalMemoryUsage.toFixed(1)}%
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
              <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
                <Box>
                  <Typography color="text.secondary" gutterBottom>
                    System Health
                  </Typography>
                  <Typography variant="h4" component="div" color="success.main">
                    Good
                  </Typography>
                </Box>
                <CheckCircleIcon color="success" sx={{ fontSize: 40 }} />
              </Box>
            </CardContent>
          </Card>
        </Grid>
      </Grid>

      {/* Control Panel */}
      <Card sx={{ mb: 4 }}>
        <CardHeader
          title="Execution Control Panel"
          subheader="Manage container lifecycle and system resources"
          action={
            <Box sx={{ display: 'flex', gap: 1 }}>
              <Button
                variant="outlined"
                startIcon={<SettingsIcon />}
                onClick={() => setShowSettings(true)}
              >
                Settings
              </Button>
              <Button
                variant="contained"
                startIcon={<RefreshIcon />}
                onClick={() => window.location.reload()}
              >
                Refresh
              </Button>
            </Box>
          }
        />
        <CardContent>
          {/* Bulk Actions */}
          {bulkSelection.length > 0 && (
            <Box sx={{ mb: 3, p: 2, bgcolor: 'action.hover', borderRadius: 1 }}>
              <Typography variant="subtitle2" gutterBottom>
                Bulk Actions ({bulkSelection.length} selected)
              </Typography>
              <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                <Button
                  size="small"
                  variant="outlined"
                  startIcon={<PlayIcon />}
                  onClick={() => handleBulkAction('start')}
                  disabled={isLoading}
                >
                  Start All
                </Button>
                <Button
                  size="small"
                  variant="outlined"
                  startIcon={<StopIcon />}
                  onClick={() => handleBulkAction('stop')}
                  disabled={isLoading}
                >
                  Stop All
                </Button>
                <Button
                  size="small"
                  variant="outlined"
                  startIcon={<RestartIcon />}
                  onClick={() => handleBulkAction('restart')}
                  disabled={isLoading}
                >
                  Restart All
                </Button>
                <Button
                  size="small"
                  variant="outlined"
                  startIcon={<DeleteIcon />}
                  onClick={() => handleBulkAction('delete')}
                  disabled={isLoading}
                  color="error"
                >
                  Delete All
                </Button>
                <Button
                  size="small"
                  variant="text"
                  onClick={() => setBulkSelection([])}
                >
                  Clear Selection
                </Button>
              </Box>
            </Box>
          )}

          {/* Filters */}
          <Box sx={{ mb: 3 }}>
            <Grid container spacing={2} alignItems="center">
              <Grid item xs={12} sm={4}>
                <TextField
                  fullWidth
                  size="small"
                  placeholder="Search containers..."
                  value={searchTerm}
                  onChange={(e) => setSearchTerm(e.target.value)}
                  InputProps={{
                    startAdornment: (
                      <InputAdornment position="start">
                        <SearchIcon />
                      </InputAdornment>
                    ),
                    endAdornment: searchTerm && (
                      <InputAdornment position="end">
                        <IconButton
                          size="small"
                          onClick={() => setSearchTerm('')}
                        >
                          <ClearIcon />
                        </IconButton>
                      </InputAdornment>
                    )
                  }}
                />
              </Grid>
              <Grid item xs={12} sm={3}>
                <FormControl fullWidth size="small">
                  <InputLabel>Status</InputLabel>
                  <Select
                    value={filterStatus}
                    onChange={(e) => setFilterStatus(e.target.value)}
                    label="Status"
                  >
                    <MenuItem value="all">All Status</MenuItem>
                    <MenuItem value="running">Running</MenuItem>
                    <MenuItem value="stopped">Stopped</MenuItem>
                    <MenuItem value="starting">Starting</MenuItem>
                    <MenuItem value="stopping">Stopping</MenuItem>
                    <MenuItem value="error">Error</MenuItem>
                  </Select>
                </FormControl>
              </Grid>
              <Grid item xs={12} sm={5}>
                <Box sx={{ display: 'flex', gap: 1, justifyContent: 'flex-end' }}>
                  <Button
                    variant="outlined"
                    startIcon={<AddIcon />}
                    onClick={() => setShowContainerDetails(true)}
                  >
                    New Container
                  </Button>
                </Box>
              </Grid>
            </Grid>
          </Box>

          {/* Container List */}
          <Grid container spacing={3}>
            {filteredContainers.map((container) => (
              <Grid item xs={12} md={6} lg={4} key={container.id}>
                <Card>
                  <CardHeader
                    title={
                      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                        <Checkbox
                          checked={bulkSelection.includes(container.id)}
                          onChange={() => handleBulkSelection(container.id)}
                          size="small"
                        />
                        {container.name}
                      </Box>
                    }
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
                      <Typography variant="body2" color="text.secondary">
                        Environment: {container.environment}
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
                      <Tooltip title="Start Container">
                        <IconButton
                          size="small"
                          onClick={() => handleContainerAction(container.id, 'start')}
                          disabled={container.status === 'running' || isLoading}
                          color="success"
                        >
                          <PlayIcon />
                        </IconButton>
                      </Tooltip>
                      <Tooltip title="Stop Container">
                        <IconButton
                          size="small"
                          onClick={() => handleContainerAction(container.id, 'stop')}
                          disabled={container.status === 'stopped' || isLoading}
                          color="error"
                        >
                          <StopIcon />
                        </IconButton>
                      </Tooltip>
                      <Tooltip title="Restart Container">
                        <IconButton
                          size="small"
                          onClick={() => handleContainerAction(container.id, 'restart')}
                          disabled={isLoading}
                          color="warning"
                        >
                          <RestartIcon />
                        </IconButton>
                      </Tooltip>
                      <Tooltip title="Pause Container">
                        <IconButton
                          size="small"
                          onClick={() => handleContainerAction(container.id, 'pause')}
                          disabled={container.status !== 'running' || isLoading}
                        >
                          <PauseIcon />
                        </IconButton>
                      </Tooltip>
                      <Tooltip title="Container Details">
                        <IconButton
                          size="small"
                          onClick={() => {
                            setSelectedContainer(container.id);
                            setShowContainerDetails(true);
                          }}
                        >
                          <ViewIcon />
                        </IconButton>
                      </Tooltip>
                      <Tooltip title="Delete Container">
                        <IconButton
                          size="small"
                          onClick={() => handleContainerAction(container.id, 'delete')}
                          disabled={isLoading}
                          color="error"
                        >
                          <DeleteIcon />
                        </IconButton>
                      </Tooltip>
                    </Box>
                  </CardContent>
                </Card>
              </Grid>
            ))}
          </Grid>

          {filteredContainers.length === 0 && (
            <Box sx={{ textAlign: 'center', py: 4 }}>
              <Typography variant="h6" color="text.secondary" gutterBottom>
                No containers found
              </Typography>
              <Typography variant="body2" color="text.secondary">
                {searchTerm || filterStatus !== 'all' 
                  ? 'Try adjusting your search or filter criteria'
                  : 'Create your first container to get started'
                }
              </Typography>
            </Box>
          )}
        </CardContent>
      </Card>

      {/* Loading Overlay */}
      {isLoading && (
        <Box
          sx={{
            position: 'fixed',
            top: 0,
            left: 0,
            right: 0,
            bottom: 0,
            bgcolor: 'rgba(0, 0, 0, 0.5)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            zIndex: 9999
          }}
        >
          <Box sx={{ textAlign: 'center', color: 'white' }}>
            <CircularProgress color="inherit" sx={{ mb: 2 }} />
            <Typography>Processing...</Typography>
          </Box>
        </Box>
      )}

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
        <DialogTitle>Execution Control Settings</DialogTitle>
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

      {/* Container Details Dialog */}
      <Dialog
        open={showContainerDetails}
        onClose={() => setShowContainerDetails(false)}
        maxWidth="md"
        fullWidth
      >
        <DialogTitle>
          {selectedContainer ? 'Container Details' : 'New Container'}
        </DialogTitle>
        <DialogContent>
          <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
            {selectedContainer ? 'View and edit container details' : 'Create a new container'}
          </Typography>
          {/* Container details form will be implemented */}
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowContainerDetails(false)}>Cancel</Button>
          <Button onClick={() => setShowContainerDetails(false)} variant="contained">
            {selectedContainer ? 'Save Changes' : 'Create Container'}
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default ExecutionControlPanel; 