import React, { useState, useEffect } from 'react';
import {
  Box,
  Typography,
  Card,
  CardContent,
  Grid,
  Tabs,
  Tab,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Paper,
  Button,
  Chip,
  Avatar,
  IconButton,
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
  CircularProgress,
  LinearProgress,
  List,
  ListItem,
  ListItemText,
  ListItemAvatar,
  ListItemSecondaryAction,
  Divider,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Tooltip,
  Badge,
} from '@mui/material';
import {
  Dashboard as DashboardIcon,
  People as PeopleIcon,
  Folder as ProjectIcon,
  Settings as SettingsIcon,
  Security as SecurityIcon,
  Analytics as AnalyticsIcon,
  Storage as StorageIcon,
  Memory as MemoryIcon,
  Speed as SpeedIcon,
  Warning as WarningIcon,
  CheckCircle as CheckCircleIcon,
  Error as ErrorIcon,
  Info as InfoIcon,
  Edit as EditIcon,
  Delete as DeleteIcon,
  Block as BlockIcon,
  LockOpen as UnblockIcon,
  ExpandMore as ExpandMoreIcon,
  Refresh as RefreshIcon,
  Download as DownloadIcon,
  Upload as UploadIcon,
  Visibility as VisibilityIcon,
  VisibilityOff as VisibilityOffIcon,
  AdminPanelSettings as AdminIcon,
  Person as PersonIcon,
  Group as GroupIcon,
  SupervisedUserCircle as SuperUserIcon,
} from '@mui/icons-material';

// Types
interface User {
  id: string;
  username: string;
  email: string;
  role: 'admin' | 'superuser' | 'user';
  status: 'active' | 'inactive' | 'suspended';
  lastLogin: string;
  createdAt: string;
  projectCount: number;
  storageUsed: number;
}

interface Project {
  id: string;
  name: string;
  owner: string;
  status: 'active' | 'inactive' | 'archived';
  category: string;
  createdAt: string;
  lastModified: string;
  moduleCount: number;
  storageUsed: number;
  isPublic: boolean;
}

interface SystemMetrics {
  totalUsers: number;
  activeUsers: number;
  totalProjects: number;
  activeProjects: number;
  totalStorage: number;
  usedStorage: number;
  cpuUsage: number;
  memoryUsage: number;
  networkUsage: number;
  systemStatus: 'healthy' | 'warning' | 'critical';
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
      id={`admin-tabpanel-${index}`}
      aria-labelledby={`admin-tab-${index}`}
      {...other}
    >
      {value === index && <Box sx={{ p: 3 }}>{children}</Box>}
    </div>
  );
}

const AdminDashboard: React.FC = () => {
  const [tabValue, setTabValue] = useState(0);
  const [loading, setLoading] = useState(false);
  const [users, setUsers] = useState<User[]>([]);
  const [projects, setProjects] = useState<Project[]>([]);
  const [metrics, setMetrics] = useState<SystemMetrics | null>(null);
  const [selectedUser, setSelectedUser] = useState<User | null>(null);
  const [selectedProject, setSelectedProject] = useState<Project | null>(null);
  const [showUserDialog, setShowUserDialog] = useState(false);
  const [showProjectDialog, setShowProjectDialog] = useState(false);
  const [showSettingsDialog, setShowSettingsDialog] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    loadDashboardData();
  }, []);

  const loadDashboardData = async () => {
    setLoading(true);
    try {
      // Mock data - in real app this would come from API
      const mockUsers: User[] = [
        {
          id: '1',
          username: 'admin',
          email: 'admin@robium.com',
          role: 'admin',
          status: 'active',
          lastLogin: '2024-01-15T10:30:00Z',
          createdAt: '2023-01-01T00:00:00Z',
          projectCount: 15,
          storageUsed: 2048,
        },
        {
          id: '2',
          username: 'alice',
          email: 'alice@robium.com',
          role: 'superuser',
          status: 'active',
          lastLogin: '2024-01-14T15:45:00Z',
          createdAt: '2023-03-15T00:00:00Z',
          projectCount: 8,
          storageUsed: 1024,
        },
        {
          id: '3',
          username: 'bob',
          email: 'bob@robium.com',
          role: 'user',
          status: 'active',
          lastLogin: '2024-01-13T09:20:00Z',
          createdAt: '2023-06-20T00:00:00Z',
          projectCount: 3,
          storageUsed: 512,
        },
        {
          id: '4',
          username: 'charlie',
          email: 'charlie@robium.com',
          role: 'user',
          status: 'suspended',
          lastLogin: '2024-01-10T14:15:00Z',
          createdAt: '2023-08-10T00:00:00Z',
          projectCount: 1,
          storageUsed: 256,
        },
      ];

      const mockProjects: Project[] = [
        {
          id: '1',
          name: 'Autonomous Navigation System',
          owner: 'admin',
          status: 'active',
          category: 'Navigation',
          createdAt: '2023-12-01T00:00:00Z',
          lastModified: '2024-01-15T10:30:00Z',
          moduleCount: 5,
          storageUsed: 512,
          isPublic: true,
        },
        {
          id: '2',
          name: 'Robot Manipulation Project',
          owner: 'alice',
          status: 'active',
          category: 'Manipulation',
          createdAt: '2023-11-15T00:00:00Z',
          lastModified: '2024-01-14T15:45:00Z',
          moduleCount: 3,
          storageUsed: 256,
          isPublic: false,
        },
        {
          id: '3',
          name: 'Computer Vision Pipeline',
          owner: 'bob',
          status: 'active',
          category: 'Perception',
          createdAt: '2023-10-20T00:00:00Z',
          lastModified: '2024-01-13T09:20:00Z',
          moduleCount: 4,
          storageUsed: 384,
          isPublic: true,
        },
        {
          id: '4',
          name: 'Test Project',
          owner: 'charlie',
          status: 'inactive',
          category: 'Testing',
          createdAt: '2023-09-05T00:00:00Z',
          lastModified: '2024-01-10T14:15:00Z',
          moduleCount: 1,
          storageUsed: 64,
          isPublic: false,
        },
      ];

      const mockMetrics: SystemMetrics = {
        totalUsers: 4,
        activeUsers: 3,
        totalProjects: 4,
        activeProjects: 3,
        totalStorage: 10000,
        usedStorage: 3840,
        cpuUsage: 45,
        memoryUsage: 62,
        networkUsage: 28,
        systemStatus: 'healthy',
      };

      setUsers(mockUsers);
      setProjects(mockProjects);
      setMetrics(mockMetrics);
      setLoading(false);
    } catch (err) {
      setError('Failed to load dashboard data');
      setLoading(false);
    }
  };

  const handleUserAction = async (userId: string, action: 'activate' | 'suspend' | 'delete') => {
    try {
      // TODO: Implement API calls
      console.log(`User action: ${action} for user ${userId}`);
      await loadDashboardData(); // Refresh data
    } catch (err) {
      setError(`Failed to ${action} user`);
    }
  };

  const handleProjectAction = async (projectId: string, action: 'activate' | 'archive' | 'delete') => {
    try {
      // TODO: Implement API calls
      console.log(`Project action: ${action} for project ${projectId}`);
      await loadDashboardData(); // Refresh data
    } catch (err) {
      setError(`Failed to ${action} project`);
    }
  };

  const getRoleIcon = (role: string) => {
    switch (role) {
      case 'admin':
        return <AdminIcon color="error" />;
      case 'superuser':
        return <SuperUserIcon color="warning" />;
      case 'user':
        return <PersonIcon color="primary" />;
      default:
        return <PersonIcon />;
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'active':
        return 'success';
      case 'inactive':
        return 'default';
      case 'suspended':
        return 'error';
      default:
        return 'default';
    }
  };

  const getSystemStatusColor = (status: string) => {
    switch (status) {
      case 'healthy':
        return 'success';
      case 'warning':
        return 'warning';
      case 'critical':
        return 'error';
      default:
        return 'default';
    }
  };

  const formatBytes = (bytes: number) => {
    if (bytes === 0) return '0 Bytes';
    const k = 1024;
    const sizes = ['Bytes', 'KB', 'MB', 'GB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
  };

  const formatDate = (dateString: string) => {
    return new Date(dateString).toLocaleDateString();
  };

  if (loading) {
    return (
      <Box sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '50vh' }}>
        <CircularProgress />
      </Box>
    );
  }

  return (
    <Box>
      {/* Header */}
      <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', mb: 3 }}>
        <Typography variant="h4" component="h1">
          Admin Dashboard
        </Typography>
        <Box sx={{ display: 'flex', gap: 2 }}>
          <Button
            variant="outlined"
            startIcon={<RefreshIcon />}
            onClick={loadDashboardData}
          >
            Refresh
          </Button>
          <Button
            variant="contained"
            startIcon={<SettingsIcon />}
            onClick={() => setShowSettingsDialog(true)}
          >
            System Settings
          </Button>
        </Box>
      </Box>

      {/* Error Alert */}
      {error && (
        <Alert severity="error" sx={{ mb: 3 }} onClose={() => setError(null)}>
          {error}
        </Alert>
      )}

      {/* System Overview Cards */}
      {metrics && (
        <Grid container spacing={3} sx={{ mb: 3 }}>
          <Grid item xs={12} sm={6} md={3}>
            <Card>
              <CardContent>
                <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
                  <Box>
                    <Typography color="textSecondary" gutterBottom>
                      Total Users
                    </Typography>
                    <Typography variant="h4">
                      {metrics.totalUsers}
                    </Typography>
                    <Typography variant="body2" color="textSecondary">
                      {metrics.activeUsers} active
                    </Typography>
                  </Box>
                  <PeopleIcon color="primary" sx={{ fontSize: 40 }} />
                </Box>
              </CardContent>
            </Card>
          </Grid>

          <Grid item xs={12} sm={6} md={3}>
            <Card>
              <CardContent>
                <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
                  <Box>
                    <Typography color="textSecondary" gutterBottom>
                      Total Projects
                    </Typography>
                    <Typography variant="h4">
                      {metrics.totalProjects}
                    </Typography>
                    <Typography variant="body2" color="textSecondary">
                      {metrics.activeProjects} active
                    </Typography>
                  </Box>
                  <ProjectIcon color="primary" sx={{ fontSize: 40 }} />
                </Box>
              </CardContent>
            </Card>
          </Grid>

          <Grid item xs={12} sm={6} md={3}>
            <Card>
              <CardContent>
                <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
                  <Box>
                    <Typography color="textSecondary" gutterBottom>
                      Storage Used
                    </Typography>
                    <Typography variant="h4">
                      {formatBytes(metrics.usedStorage)}
                    </Typography>
                    <Typography variant="body2" color="textSecondary">
                      {((metrics.usedStorage / metrics.totalStorage) * 100).toFixed(1)}% of total
                    </Typography>
                  </Box>
                  <StorageIcon color="primary" sx={{ fontSize: 40 }} />
                </Box>
              </CardContent>
            </Card>
          </Grid>

          <Grid item xs={12} sm={6} md={3}>
            <Card>
              <CardContent>
                <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
                  <Box>
                    <Typography color="textSecondary" gutterBottom>
                      System Status
                    </Typography>
                    <Chip
                      label={metrics.systemStatus}
                      color={getSystemStatusColor(metrics.systemStatus) as any}
                      sx={{ mb: 1 }}
                    />
                    <Typography variant="body2" color="textSecondary">
                      CPU: {metrics.cpuUsage}% | RAM: {metrics.memoryUsage}%
                    </Typography>
                  </Box>
                  <AnalyticsIcon color="primary" sx={{ fontSize: 40 }} />
                </Box>
              </CardContent>
            </Card>
          </Grid>
        </Grid>
      )}

      {/* Main Dashboard Tabs */}
      <Card>
        <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
          <Tabs value={tabValue} onChange={(_, newValue) => setTabValue(newValue)}>
            <Tab icon={<PeopleIcon />} label="User Management" />
            <Tab icon={<ProjectIcon />} label="Project Oversight" />
            <Tab icon={<AnalyticsIcon />} label="System Metrics" />
            <Tab icon={<SecurityIcon />} label="Security & Access" />
          </Tabs>
        </Box>

        {/* User Management Tab */}
        <TabPanel value={tabValue} index={0}>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
            <Typography variant="h6">
              User Management
            </Typography>
            <Button variant="contained" startIcon={<PeopleIcon />}>
              Add User
            </Button>
          </Box>

          <TableContainer component={Paper}>
            <Table>
              <TableHead>
                <TableRow>
                  <TableCell>User</TableCell>
                  <TableCell>Role</TableCell>
                  <TableCell>Status</TableCell>
                  <TableCell>Projects</TableCell>
                  <TableCell>Storage Used</TableCell>
                  <TableCell>Last Login</TableCell>
                  <TableCell>Actions</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {users.map((user) => (
                  <TableRow key={user.id}>
                    <TableCell>
                      <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
                        <Avatar>{user.username.charAt(0).toUpperCase()}</Avatar>
                        <Box>
                          <Typography variant="subtitle2">{user.username}</Typography>
                          <Typography variant="body2" color="textSecondary">
                            {user.email}
                          </Typography>
                        </Box>
                      </Box>
                    </TableCell>
                    <TableCell>
                      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                        {getRoleIcon(user.role)}
                        <Chip label={user.role} size="small" />
                      </Box>
                    </TableCell>
                    <TableCell>
                      <Chip
                        label={user.status}
                        color={getStatusColor(user.status) as any}
                        size="small"
                      />
                    </TableCell>
                    <TableCell>{user.projectCount}</TableCell>
                    <TableCell>{formatBytes(user.storageUsed)}</TableCell>
                    <TableCell>{formatDate(user.lastLogin)}</TableCell>
                    <TableCell>
                      <Box sx={{ display: 'flex', gap: 1 }}>
                        <Tooltip title="Edit User">
                          <IconButton size="small" onClick={() => setSelectedUser(user)}>
                            <EditIcon />
                          </IconButton>
                        </Tooltip>
                        {user.status === 'active' ? (
                          <Tooltip title="Suspend User">
                            <IconButton
                              size="small"
                              color="warning"
                              onClick={() => handleUserAction(user.id, 'suspend')}
                            >
                              <BlockIcon />
                            </IconButton>
                          </Tooltip>
                        ) : (
                          <Tooltip title="Activate User">
                            <IconButton
                              size="small"
                              color="success"
                              onClick={() => handleUserAction(user.id, 'activate')}
                            >
                              <UnblockIcon />
                            </IconButton>
                          </Tooltip>
                        )}
                        <Tooltip title="Delete User">
                          <IconButton
                            size="small"
                            color="error"
                            onClick={() => handleUserAction(user.id, 'delete')}
                          >
                            <DeleteIcon />
                          </IconButton>
                        </Tooltip>
                      </Box>
                    </TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          </TableContainer>
        </TabPanel>

        {/* Project Oversight Tab */}
        <TabPanel value={tabValue} index={1}>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
            <Typography variant="h6">
              Project Oversight
            </Typography>
            <Button variant="contained" startIcon={<ProjectIcon />}>
              View All Projects
            </Button>
          </Box>

          <TableContainer component={Paper}>
            <Table>
              <TableHead>
                <TableRow>
                  <TableCell>Project</TableCell>
                  <TableCell>Owner</TableCell>
                  <TableCell>Status</TableCell>
                  <TableCell>Category</TableCell>
                  <TableCell>Modules</TableCell>
                  <TableCell>Storage</TableCell>
                  <TableCell>Visibility</TableCell>
                  <TableCell>Last Modified</TableCell>
                  <TableCell>Actions</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {projects.map((project) => (
                  <TableRow key={project.id}>
                    <TableCell>
                      <Typography variant="subtitle2">{project.name}</Typography>
                    </TableCell>
                    <TableCell>{project.owner}</TableCell>
                    <TableCell>
                      <Chip
                        label={project.status}
                        color={getStatusColor(project.status) as any}
                        size="small"
                      />
                    </TableCell>
                    <TableCell>
                      <Chip label={project.category} size="small" variant="outlined" />
                    </TableCell>
                    <TableCell>{project.moduleCount}</TableCell>
                    <TableCell>{formatBytes(project.storageUsed)}</TableCell>
                    <TableCell>
                      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                        {project.isPublic ? (
                          <>
                            <VisibilityIcon fontSize="small" />
                            <Typography variant="body2">Public</Typography>
                          </>
                        ) : (
                          <>
                            <VisibilityOffIcon fontSize="small" />
                            <Typography variant="body2">Private</Typography>
                          </>
                        )}
                      </Box>
                    </TableCell>
                    <TableCell>{formatDate(project.lastModified)}</TableCell>
                    <TableCell>
                      <Box sx={{ display: 'flex', gap: 1 }}>
                        <Tooltip title="View Project">
                          <IconButton size="small">
                            <VisibilityIcon />
                          </IconButton>
                        </Tooltip>
                        {project.status === 'active' ? (
                          <Tooltip title="Archive Project">
                            <IconButton
                              size="small"
                              color="warning"
                              onClick={() => handleProjectAction(project.id, 'archive')}
                            >
                              <ProjectIcon />
                            </IconButton>
                          </Tooltip>
                        ) : (
                          <Tooltip title="Activate Project">
                            <IconButton
                              size="small"
                              color="success"
                              onClick={() => handleProjectAction(project.id, 'activate')}
                            >
                              <ProjectIcon />
                            </IconButton>
                          </Tooltip>
                        )}
                        <Tooltip title="Delete Project">
                          <IconButton
                            size="small"
                            color="error"
                            onClick={() => handleProjectAction(project.id, 'delete')}
                          >
                            <DeleteIcon />
                          </IconButton>
                        </Tooltip>
                      </Box>
                    </TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          </TableContainer>
        </TabPanel>

        {/* System Metrics Tab */}
        <TabPanel value={tabValue} index={2}>
          <Typography variant="h6" gutterBottom>
            System Performance Metrics
          </Typography>

          <Grid container spacing={3}>
            <Grid item xs={12} md={6}>
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    Resource Usage
                  </Typography>
                  
                  <Box sx={{ mb: 3 }}>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 1 }}>
                      <Typography variant="body2">CPU Usage</Typography>
                      <Typography variant="body2">{metrics?.cpuUsage}%</Typography>
                    </Box>
                    <LinearProgress
                      variant="determinate"
                      value={metrics?.cpuUsage || 0}
                      color={metrics && metrics.cpuUsage > 80 ? 'error' : 'primary'}
                    />
                  </Box>

                  <Box sx={{ mb: 3 }}>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 1 }}>
                      <Typography variant="body2">Memory Usage</Typography>
                      <Typography variant="body2">{metrics?.memoryUsage}%</Typography>
                    </Box>
                    <LinearProgress
                      variant="determinate"
                      value={metrics?.memoryUsage || 0}
                      color={metrics && metrics.memoryUsage > 80 ? 'error' : 'primary'}
                    />
                  </Box>

                  <Box sx={{ mb: 3 }}>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 1 }}>
                      <Typography variant="body2">Network Usage</Typography>
                      <Typography variant="body2">{metrics?.networkUsage}%</Typography>
                    </Box>
                    <LinearProgress
                      variant="determinate"
                      value={metrics?.networkUsage || 0}
                      color="primary"
                    />
                  </Box>

                  <Box>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 1 }}>
                      <Typography variant="body2">Storage Usage</Typography>
                      <Typography variant="body2">
                        {metrics ? `${((metrics.usedStorage / metrics.totalStorage) * 100).toFixed(1)}%` : '0%'}
                      </Typography>
                    </Box>
                    <LinearProgress
                      variant="determinate"
                      value={metrics ? (metrics.usedStorage / metrics.totalStorage) * 100 : 0}
                      color={metrics && (metrics.usedStorage / metrics.totalStorage) * 100 > 80 ? 'error' : 'primary'}
                    />
                  </Box>
                </CardContent>
              </Card>
            </Grid>

            <Grid item xs={12} md={6}>
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    System Health
                  </Typography>
                  
                  <List>
                    <ListItem>
                      <ListItemAvatar>
                        <Avatar sx={{ bgcolor: 'success.main' }}>
                          <CheckCircleIcon />
                        </Avatar>
                      </ListItemAvatar>
                      <ListItemText
                        primary="System Status"
                        secondary={metrics?.systemStatus}
                      />
                    </ListItem>
                    
                    <ListItem>
                      <ListItemAvatar>
                        <Avatar sx={{ bgcolor: 'info.main' }}>
                          <MemoryIcon />
                        </Avatar>
                      </ListItemAvatar>
                      <ListItemText
                        primary="Total Storage"
                        secondary={metrics ? formatBytes(metrics.totalStorage) : '0 Bytes'}
                      />
                    </ListItem>
                    
                    <ListItem>
                      <ListItemAvatar>
                        <Avatar sx={{ bgcolor: 'warning.main' }}>
                          <SpeedIcon />
                        </Avatar>
                      </ListItemAvatar>
                      <ListItemText
                        primary="Active Users"
                        secondary={`${metrics?.activeUsers || 0} / ${metrics?.totalUsers || 0}`}
                      />
                    </ListItem>
                    
                    <ListItem>
                      <ListItemAvatar>
                        <Avatar sx={{ bgcolor: 'primary.main' }}>
                          <ProjectIcon />
                        </Avatar>
                      </ListItemAvatar>
                      <ListItemText
                        primary="Active Projects"
                        secondary={`${metrics?.activeProjects || 0} / ${metrics?.totalProjects || 0}`}
                      />
                    </ListItem>
                  </List>
                </CardContent>
              </Card>
            </Grid>
          </Grid>
        </TabPanel>

        {/* Security & Access Tab */}
        <TabPanel value={tabValue} index={3}>
          <Typography variant="h6" gutterBottom>
            Security & Access Control
          </Typography>

          <Grid container spacing={3}>
            <Grid item xs={12} md={6}>
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    Access Control Settings
                  </Typography>
                  
                  <List>
                    <ListItem>
                      <ListItemText
                        primary="Require Two-Factor Authentication"
                        secondary="Enforce 2FA for all users"
                      />
                      <ListItemSecondaryAction>
                        <Switch />
                      </ListItemSecondaryAction>
                    </ListItem>
                    
                    <ListItem>
                      <ListItemText
                        primary="Session Timeout"
                        secondary="Auto-logout after inactivity"
                      />
                      <ListItemSecondaryAction>
                        <Switch defaultChecked />
                      </ListItemSecondaryAction>
                    </ListItem>
                    
                    <ListItem>
                      <ListItemText
                        primary="IP Whitelist"
                        secondary="Restrict access to specific IPs"
                      />
                      <ListItemSecondaryAction>
                        <Switch />
                      </ListItemSecondaryAction>
                    </ListItem>
                    
                    <ListItem>
                      <ListItemText
                        primary="Audit Logging"
                        secondary="Log all administrative actions"
                      />
                      <ListItemSecondaryAction>
                        <Switch defaultChecked />
                      </ListItemSecondaryAction>
                    </ListItem>
                  </List>
                </CardContent>
              </Card>
            </Grid>

            <Grid item xs={12} md={6}>
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    Recent Security Events
                  </Typography>
                  
                  <List>
                    <ListItem>
                      <ListItemAvatar>
                        <Avatar sx={{ bgcolor: 'info.main' }}>
                          <InfoIcon />
                        </Avatar>
                      </ListItemAvatar>
                      <ListItemText
                        primary="User Login"
                        secondary="admin@robium.com logged in from 192.168.1.100"
                      />
                    </ListItem>
                    
                    <ListItem>
                      <ListItemAvatar>
                        <Avatar sx={{ bgcolor: 'warning.main' }}>
                          <WarningIcon />
                        </Avatar>
                      </ListItemAvatar>
                      <ListItemText
                        primary="Failed Login Attempt"
                        secondary="Failed login for user 'test' from 10.0.0.50"
                      />
                    </ListItem>
                    
                    <ListItem>
                      <ListItemAvatar>
                        <Avatar sx={{ bgcolor: 'success.main' }}>
                          <CheckCircleIcon />
                        </Avatar>
                      </ListItemAvatar>
                      <ListItemText
                        primary="Project Created"
                        secondary="User 'alice' created project 'Robot Manipulation'"
                      />
                    </ListItem>
                    
                    <ListItem>
                      <ListItemAvatar>
                        <Avatar sx={{ bgcolor: 'error.main' }}>
                          <ErrorIcon />
                        </Avatar>
                      </ListItemAvatar>
                      <ListItemText
                        primary="User Suspended"
                        secondary="User 'charlie' suspended by admin"
                      />
                    </ListItem>
                  </List>
                </CardContent>
              </Card>
            </Grid>
          </Grid>
        </TabPanel>
      </Card>

      {/* User Edit Dialog */}
      <Dialog open={showUserDialog} onClose={() => setShowUserDialog(false)} maxWidth="sm" fullWidth>
        <DialogTitle>Edit User</DialogTitle>
        <DialogContent>
          {selectedUser && (
            <Box sx={{ pt: 2 }}>
              <TextField
                fullWidth
                label="Username"
                value={selectedUser.username}
                margin="normal"
              />
              <TextField
                fullWidth
                label="Email"
                value={selectedUser.email}
                margin="normal"
              />
              <FormControl fullWidth margin="normal">
                <InputLabel>Role</InputLabel>
                <Select value={selectedUser.role} label="Role">
                  <MenuItem value="user">User</MenuItem>
                  <MenuItem value="superuser">Super User</MenuItem>
                  <MenuItem value="admin">Admin</MenuItem>
                </Select>
              </FormControl>
              <FormControl fullWidth margin="normal">
                <InputLabel>Status</InputLabel>
                <Select value={selectedUser.status} label="Status">
                  <MenuItem value="active">Active</MenuItem>
                  <MenuItem value="inactive">Inactive</MenuItem>
                  <MenuItem value="suspended">Suspended</MenuItem>
                </Select>
              </FormControl>
            </Box>
          )}
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowUserDialog(false)}>Cancel</Button>
          <Button variant="contained">Save Changes</Button>
        </DialogActions>
      </Dialog>

      {/* System Settings Dialog */}
      <Dialog open={showSettingsDialog} onClose={() => setShowSettingsDialog(false)} maxWidth="md" fullWidth>
        <DialogTitle>System Settings</DialogTitle>
        <DialogContent>
          <Accordion>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="h6">General Settings</Typography>
            </AccordionSummary>
            <AccordionDetails>
              <Grid container spacing={2}>
                <Grid item xs={12} md={6}>
                  <TextField
                    fullWidth
                    label="System Name"
                    defaultValue="Robium Platform"
                    margin="normal"
                  />
                </Grid>
                <Grid item xs={12} md={6}>
                  <TextField
                    fullWidth
                    label="Admin Email"
                    defaultValue="admin@robium.com"
                    margin="normal"
                  />
                </Grid>
                <Grid item xs={12}>
                  <FormControlLabel
                    control={<Switch defaultChecked />}
                    label="Enable Email Notifications"
                  />
                </Grid>
              </Grid>
            </AccordionDetails>
          </Accordion>

          <Accordion>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="h6">Storage Settings</Typography>
            </AccordionSummary>
            <AccordionDetails>
              <Grid container spacing={2}>
                <Grid item xs={12} md={6}>
                  <TextField
                    fullWidth
                    label="Max Storage per User (GB)"
                    type="number"
                    defaultValue="10"
                    margin="normal"
                  />
                </Grid>
                <Grid item xs={12} md={6}>
                  <TextField
                    fullWidth
                    label="Max Projects per User"
                    type="number"
                    defaultValue="50"
                    margin="normal"
                  />
                </Grid>
                <Grid item xs={12}>
                  <FormControlLabel
                    control={<Switch defaultChecked />}
                    label="Auto-cleanup inactive projects"
                  />
                </Grid>
              </Grid>
            </AccordionDetails>
          </Accordion>

          <Accordion>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="h6">Security Settings</Typography>
            </AccordionSummary>
            <AccordionDetails>
              <Grid container spacing={2}>
                <Grid item xs={12} md={6}>
                  <TextField
                    fullWidth
                    label="Session Timeout (minutes)"
                    type="number"
                    defaultValue="30"
                    margin="normal"
                  />
                </Grid>
                <Grid item xs={12} md={6}>
                  <TextField
                    fullWidth
                    label="Max Login Attempts"
                    type="number"
                    defaultValue="5"
                    margin="normal"
                  />
                </Grid>
                <Grid item xs={12}>
                  <FormControlLabel
                    control={<Switch />}
                    label="Require Two-Factor Authentication"
                  />
                </Grid>
                <Grid item xs={12}>
                  <FormControlLabel
                    control={<Switch defaultChecked />}
                    label="Enable Audit Logging"
                  />
                </Grid>
              </Grid>
            </AccordionDetails>
          </Accordion>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowSettingsDialog(false)}>Cancel</Button>
          <Button variant="contained">Save Settings</Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default AdminDashboard;
