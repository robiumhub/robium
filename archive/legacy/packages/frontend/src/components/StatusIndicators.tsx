import React, { useState, useEffect } from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
  Chip,
  LinearProgress,
  IconButton,
  Tooltip,
  Badge,
  Alert,
  Snackbar,
  Grid,
  Paper,
  Divider,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  Collapse,
  Button,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
} from '@mui/material';
import {
  CheckCircle,
  Error,
  Warning,
  Info,
  Refresh,
  ExpandMore,
  ExpandLess,
  Notifications,
  NotificationsActive,
  NotificationsOff,
  Memory,
  Storage,
  Speed,
  NetworkCheck,
  Security,
  CloudDone,
  CloudOff,
  CloudQueue,
  TrendingUp,
  TrendingDown,
  Timeline,
  Assessment,
  Settings,
  Close,
} from '@mui/icons-material';

// Types for status indicators
export interface SystemStatus {
  overall: 'healthy' | 'warning' | 'error' | 'unknown';
  containers: ContainerStatus[];
  resources: ResourceStatus;
  services: ServiceStatus[];
  notifications: Notification[];
  lastUpdated: Date;
}

export interface ContainerStatus {
  id: string;
  name: string;
  status: 'running' | 'stopped' | 'starting' | 'stopping' | 'error' | 'unknown';
  health: 'healthy' | 'unhealthy' | 'starting' | 'unknown';
  cpu: number;
  memory: number;
  network: number;
  uptime: string;
  lastCheck: Date;
}

export interface ResourceStatus {
  cpu: {
    usage: number;
    cores: number;
    temperature: number;
    status: 'normal' | 'warning' | 'critical';
  };
  memory: {
    usage: number;
    total: number;
    available: number;
    status: 'normal' | 'warning' | 'critical';
  };
  disk: {
    usage: number;
    total: number;
    available: number;
    status: 'normal' | 'warning' | 'critical';
  };
  network: {
    rx: number;
    tx: number;
    status: 'normal' | 'warning' | 'critical';
  };
}

export interface ServiceStatus {
  name: string;
  status: 'running' | 'stopped' | 'error' | 'unknown';
  health: 'healthy' | 'unhealthy' | 'starting' | 'unknown';
  uptime: string;
  lastCheck: Date;
}

export interface Notification {
  id: string;
  type: 'info' | 'warning' | 'error' | 'success';
  title: string;
  message: string;
  timestamp: Date;
  read: boolean;
  action?: {
    label: string;
    onClick: () => void;
  };
}

interface StatusIndicatorsProps {
  systemStatus?: SystemStatus;
  onRefresh?: () => void;
  onNotificationAction?: (notificationId: string, action: string) => void;
  onMarkNotificationRead?: (notificationId: string) => void;
  onClearNotifications?: () => void;
  expanded?: boolean;
  onToggleExpanded?: () => void;
}

const StatusIndicators: React.FC<StatusIndicatorsProps> = ({
  systemStatus,
  onRefresh,
  onNotificationAction,
  onMarkNotificationRead,
  onClearNotifications,
  expanded = false,
  onToggleExpanded,
}) => {
  const [notificationsOpen, setNotificationsOpen] = useState(false);
  const [detailsOpen, setDetailsOpen] = useState(false);
  const [snackbarOpen, setSnackbarOpen] = useState(false);
  const [snackbarMessage, setSnackbarMessage] = useState('');

  // Mock data for demonstration
  const mockSystemStatus: SystemStatus = {
    overall: 'healthy',
    containers: [
      {
        id: 'container-1',
        name: 'ros2-navigation',
        status: 'running',
        health: 'healthy',
        cpu: 25.5,
        memory: 45.2,
        network: 12.8,
        uptime: '2h 15m',
        lastCheck: new Date(),
      },
      {
        id: 'container-2',
        name: 'gazebo-sim',
        status: 'running',
        health: 'healthy',
        cpu: 18.3,
        memory: 32.1,
        network: 8.5,
        uptime: '1h 45m',
        lastCheck: new Date(),
      },
      {
        id: 'container-3',
        name: 'rviz-viz',
        status: 'stopped',
        health: 'unknown',
        cpu: 0,
        memory: 0,
        network: 0,
        uptime: '0m',
        lastCheck: new Date(),
      },
    ],
    resources: {
      cpu: {
        usage: 43.8,
        cores: 8,
        temperature: 65,
        status: 'normal',
      },
      memory: {
        usage: 77.3,
        total: 16384,
        available: 3726,
        status: 'warning',
      },
      disk: {
        usage: 45.2,
        total: 1000000,
        available: 548000,
        status: 'normal',
      },
      network: {
        rx: 1250,
        tx: 890,
        status: 'normal',
      },
    },
    services: [
      {
        name: 'Docker Engine',
        status: 'running',
        health: 'healthy',
        uptime: '5d 12h',
        lastCheck: new Date(),
      },
      {
        name: 'WebSocket Server',
        status: 'running',
        health: 'healthy',
        uptime: '2h 30m',
        lastCheck: new Date(),
      },
      {
        name: 'Database',
        status: 'running',
        health: 'healthy',
        uptime: '1d 8h',
        lastCheck: new Date(),
      },
    ],
    notifications: [
      {
        id: 'notif-1',
        type: 'warning',
        title: 'High Memory Usage',
        message: 'System memory usage is at 77.3%. Consider stopping unused containers.',
        timestamp: new Date(Date.now() - 5 * 60 * 1000), // 5 minutes ago
        read: false,
        action: {
          label: 'View Containers',
          onClick: () => {
            setSnackbarMessage('Opening container management...');
            setSnackbarOpen(true);
          },
        },
      },
      {
        id: 'notif-2',
        type: 'info',
        title: 'Container Started',
        message: 'Container "ros2-navigation" has been successfully started.',
        timestamp: new Date(Date.now() - 15 * 60 * 1000), // 15 minutes ago
        read: true,
      },
      {
        id: 'notif-3',
        type: 'success',
        title: 'System Health Check',
        message: 'All system components are operating normally.',
        timestamp: new Date(Date.now() - 30 * 60 * 1000), // 30 minutes ago
        read: true,
      },
    ],
    lastUpdated: new Date(),
  };

  const status = systemStatus || mockSystemStatus;

  // Calculate overall system health
  const getOverallStatus = () => {
    const containerErrors = status.containers.filter(c => c.status === 'error').length;
    const serviceErrors = status.services.filter(s => s.status === 'error').length;
    const resourceWarnings = [
      status.resources.cpu.status,
      status.resources.memory.status,
      status.resources.disk.status,
      status.resources.network.status,
    ].filter(s => s === 'critical').length;

    if (containerErrors > 0 || serviceErrors > 0 || resourceWarnings > 0) {
      return 'error';
    } else if (status.resources.memory.status === 'warning') {
      return 'warning';
    }
    return 'healthy';
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'healthy':
      case 'running':
      case 'normal':
        return 'success';
      case 'warning':
      case 'starting':
      case 'stopping':
        return 'warning';
      case 'error':
      case 'critical':
      case 'stopped':
      case 'unhealthy':
        return 'error';
      default:
        return 'primary';
    }
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'healthy':
      case 'running':
      case 'normal':
        return <CheckCircle />;
      case 'warning':
      case 'starting':
      case 'stopping':
        return <Warning />;
      case 'error':
      case 'critical':
      case 'stopped':
      case 'unhealthy':
        return <Error />;
      default:
        return <Info />;
    }
  };

  const formatBytes = (bytes: number) => {
    if (bytes === 0) return '0 B';
    const k = 1024;
    const sizes = ['B', 'KB', 'MB', 'GB', 'TB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
  };

  const formatUptime = (uptime: string) => {
    return uptime;
  };

  const unreadNotifications = status.notifications.filter(n => !n.read).length;

  const handleNotificationClick = (notification: Notification) => {
    if (!notification.read && onMarkNotificationRead) {
      onMarkNotificationRead(notification.id);
    }
    if (notification.action) {
      notification.action.onClick();
    }
  };

  const handleRefresh = () => {
    if (onRefresh) {
      onRefresh();
      setSnackbarMessage('Status refreshed successfully');
      setSnackbarOpen(true);
    }
  };

  return (
    <Box>
      {/* Main Status Overview */}
      <Card sx={{ mb: 2 }}>
        <CardContent>
          <Box display="flex" alignItems="center" justifyContent="space-between" mb={2}>
            <Typography variant="h6" component="h2">
              System Status
            </Typography>
            <Box display="flex" alignItems="center" gap={1}>
              <Tooltip title="Refresh Status">
                <IconButton onClick={handleRefresh} size="small">
                  <Refresh />
                </IconButton>
              </Tooltip>
              <Tooltip title="Notifications">
                <Badge badgeContent={unreadNotifications} color="error">
                  <IconButton onClick={() => setNotificationsOpen(true)} size="small">
                    <Notifications />
                  </IconButton>
                </Badge>
              </Tooltip>
              <Tooltip title={expanded ? "Collapse" : "Expand"}>
                <IconButton onClick={onToggleExpanded} size="small">
                  {expanded ? <ExpandLess /> : <ExpandMore />}
                </IconButton>
              </Tooltip>
            </Box>
          </Box>

          {/* Overall Status */}
          <Box display="flex" alignItems="center" gap={2} mb={2}>
            <Chip
              icon={getStatusIcon(getOverallStatus())}
              label={`Overall: ${getOverallStatus().toUpperCase()}`}
              color={getStatusColor(getOverallStatus())}
              variant="outlined"
            />
            <Typography variant="body2" color="text.secondary">
              Last updated: {status.lastUpdated.toLocaleTimeString()}
            </Typography>
          </Box>

          {/* Quick Stats */}
          <Box display="flex" flexWrap="wrap" gap={2}>
            <Box flex="1" minWidth="120px" textAlign="center">
              <Typography variant="h6" color="primary">
                {status.containers.filter(c => c.status === 'running').length}
              </Typography>
              <Typography variant="caption" color="text.secondary">
                Running Containers
              </Typography>
            </Box>
            <Box flex="1" minWidth="120px" textAlign="center">
              <Typography variant="h6" color="primary">
                {status.services.filter(s => s.status === 'running').length}
              </Typography>
              <Typography variant="caption" color="text.secondary">
                Active Services
              </Typography>
            </Box>
            <Box flex="1" minWidth="120px" textAlign="center">
              <Typography variant="h6" color="primary">
                {unreadNotifications}
              </Typography>
              <Typography variant="caption" color="text.secondary">
                Unread Alerts
              </Typography>
            </Box>
            <Box flex="1" minWidth="120px" textAlign="center">
              <Typography variant="h6" color="primary">
                {Math.round(status.resources.cpu.usage)}%
              </Typography>
              <Typography variant="caption" color="text.secondary">
                CPU Usage
              </Typography>
            </Box>
          </Box>
        </CardContent>
      </Card>

      {/* Expanded Details */}
      <Collapse in={expanded}>
        <Box display="flex" flexDirection="column" gap={2}>
          <Box display="flex" flexDirection={{ xs: 'column', md: 'row' }} gap={2}>
            {/* Container Status */}
            <Box flex="1">
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    Container Status
                  </Typography>
                  <List dense>
                    {status.containers.map((container) => (
                      <ListItem key={container.id} divider>
                        <ListItemIcon>
                          {getStatusIcon(container.status)}
                        </ListItemIcon>
                        <ListItemText
                          primary={container.name}
                          secondary={
                            <Box>
                              <Typography variant="caption" display="block">
                                Status: {container.status} | Health: {container.health} | Uptime: {container.uptime}
                              </Typography>
                              <Box display="flex" gap={1} mt={0.5}>
                                <Chip size="small" label={`CPU: ${container.cpu}%`} />
                                <Chip size="small" label={`Memory: ${container.memory}%`} />
                                <Chip size="small" label={`Network: ${container.network}%`} />
                              </Box>
                            </Box>
                          }
                        />
                      </ListItem>
                    ))}
                  </List>
                </CardContent>
              </Card>
            </Box>

            {/* Resource Status */}
            <Box flex="1">
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    Resource Usage
                  </Typography>
                  
                  {/* CPU */}
                  <Box mb={2}>
                    <Box display="flex" justifyContent="space-between" alignItems="center" mb={1}>
                      <Typography variant="body2">CPU Usage</Typography>
                      <Chip
                        size="small"
                        icon={getStatusIcon(status.resources.cpu.status)}
                        label={`${Math.round(status.resources.cpu.usage)}%`}
                        color={getStatusColor(status.resources.cpu.status)}
                      />
                    </Box>
                    <LinearProgress
                      variant="determinate"
                      value={status.resources.cpu.usage}
                      color={getStatusColor(status.resources.cpu.status)}
                      sx={{ height: 8, borderRadius: 4 }}
                    />
                    <Typography variant="caption" color="text.secondary">
                      {status.resources.cpu.cores} cores | {status.resources.cpu.temperature}°C
                    </Typography>
                  </Box>

                  {/* Memory */}
                  <Box mb={2}>
                    <Box display="flex" justifyContent="space-between" alignItems="center" mb={1}>
                      <Typography variant="body2">Memory Usage</Typography>
                      <Chip
                        size="small"
                        icon={getStatusIcon(status.resources.memory.status)}
                        label={`${Math.round(status.resources.memory.usage)}%`}
                        color={getStatusColor(status.resources.memory.status)}
                      />
                    </Box>
                    <LinearProgress
                      variant="determinate"
                      value={status.resources.memory.usage}
                      color={getStatusColor(status.resources.memory.status)}
                      sx={{ height: 8, borderRadius: 4 }}
                    />
                    <Typography variant="caption" color="text.secondary">
                      {formatBytes(status.resources.memory.available)} available of {formatBytes(status.resources.memory.total)}
                    </Typography>
                  </Box>

                  {/* Disk */}
                  <Box mb={2}>
                    <Box display="flex" justifyContent="space-between" alignItems="center" mb={1}>
                      <Typography variant="body2">Disk Usage</Typography>
                      <Chip
                        size="small"
                        icon={getStatusIcon(status.resources.disk.status)}
                        label={`${Math.round(status.resources.disk.usage)}%`}
                        color={getStatusColor(status.resources.disk.status)}
                      />
                    </Box>
                    <LinearProgress
                      variant="determinate"
                      value={status.resources.disk.usage}
                      color={getStatusColor(status.resources.disk.status)}
                      sx={{ height: 8, borderRadius: 4 }}
                    />
                    <Typography variant="caption" color="text.secondary">
                      {formatBytes(status.resources.disk.available)} available of {formatBytes(status.resources.disk.total)}
                    </Typography>
                  </Box>

                  {/* Network */}
                  <Box>
                    <Box display="flex" justifyContent="space-between" alignItems="center" mb={1}>
                      <Typography variant="body2">Network Activity</Typography>
                      <Chip
                        size="small"
                        icon={getStatusIcon(status.resources.network.status)}
                        label="Active"
                        color={getStatusColor(status.resources.network.status)}
                      />
                    </Box>
                    <Typography variant="caption" color="text.secondary">
                      RX: {formatBytes(status.resources.network.rx)}/s | TX: {formatBytes(status.resources.network.tx)}/s
                    </Typography>
                  </Box>
                </CardContent>
              </Card>
            </Box>
          </Box>

          {/* Service Status */}
          <Box>
            <Card>
              <CardContent>
                <Typography variant="h6" gutterBottom>
                  Service Status
                </Typography>
                <Box display="flex" flexWrap="wrap" gap={2}>
                  {status.services.map((service) => (
                    <Box key={service.name} flex="1" minWidth="200px">
                      <Paper variant="outlined" sx={{ p: 2 }}>
                        <Box display="flex" alignItems="center" gap={1} mb={1}>
                          {getStatusIcon(service.status)}
                          <Typography variant="subtitle2">{service.name}</Typography>
                        </Box>
                        <Typography variant="caption" display="block" color="text.secondary">
                          Status: {service.status} | Health: {service.health}
                        </Typography>
                        <Typography variant="caption" display="block" color="text.secondary">
                          Uptime: {service.uptime}
                        </Typography>
                      </Paper>
                    </Box>
                  ))}
                </Box>
              </CardContent>
            </Card>
          </Box>
        </Box>
      </Collapse>

      {/* Notifications Dialog */}
      <Dialog
        open={notificationsOpen}
        onClose={() => setNotificationsOpen(false)}
        maxWidth="md"
        fullWidth
      >
        <DialogTitle>
          <Box display="flex" alignItems="center" justifyContent="space-between">
            <Typography variant="h6">System Notifications</Typography>
            <Box>
              {onClearNotifications && (
                <Button
                  size="small"
                  onClick={() => {
                    onClearNotifications();
                    setSnackbarMessage('All notifications cleared');
                    setSnackbarOpen(true);
                  }}
                >
                  Clear All
                </Button>
              )}
              <IconButton onClick={() => setNotificationsOpen(false)} size="small">
                <Close />
              </IconButton>
            </Box>
          </Box>
        </DialogTitle>
        <DialogContent>
          <List>
            {status.notifications.length === 0 ? (
              <ListItem>
                <ListItemText
                  primary="No notifications"
                  secondary="All systems are operating normally"
                />
              </ListItem>
            ) : (
              status.notifications.map((notification) => (
                                 <ListItem
                   key={notification.id}
                   divider
                   sx={{
                     backgroundColor: notification.read ? 'transparent' : 'action.hover',
                     borderRadius: 1,
                     mb: 1,
                     cursor: 'pointer',
                     '&:hover': {
                       backgroundColor: 'action.hover',
                     },
                   }}
                   onClick={() => handleNotificationClick(notification)}
                 >
                  <ListItemIcon>
                    {getStatusIcon(notification.type)}
                  </ListItemIcon>
                  <ListItemText
                    primary={notification.title}
                    secondary={
                      <Box>
                        <Typography variant="body2">{notification.message}</Typography>
                        <Typography variant="caption" color="text.secondary">
                          {notification.timestamp.toLocaleString()}
                        </Typography>
                        {notification.action && (
                          <Button
                            size="small"
                            variant="outlined"
                            sx={{ mt: 1 }}
                            onClick={(e) => {
                              e.stopPropagation();
                              notification.action!.onClick();
                            }}
                          >
                            {notification.action.label}
                          </Button>
                        )}
                      </Box>
                    }
                  />
                </ListItem>
              ))
            )}
          </List>
        </DialogContent>
      </Dialog>

      {/* Snackbar for feedback */}
      <Snackbar
        open={snackbarOpen}
        autoHideDuration={3000}
        onClose={() => setSnackbarOpen(false)}
        message={snackbarMessage}
      />
    </Box>
  );
};

export default StatusIndicators; 