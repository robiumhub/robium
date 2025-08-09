import React, { useState, useEffect } from 'react';
import {
  Box,
  Typography,
  Card,
  CardContent,
  CardHeader,
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
  Snackbar,
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
  Add as AddIcon,
  Timeline as TimelineIcon,
  Description as TemplateIcon,
  SmartToy as RobotIcon,
} from '@mui/icons-material';
import {
  RobotTypeBarChart,
  RobotCategoryPieChart,
  RobotTrendLineChart,
} from '../components/charts';
import ApiService from '../services/api';

// Types
interface User {
  id: string;
  username: string;
  email: string;
  role: 'user' | 'admin';
  createdAt: string;
  updatedAt: string;
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

interface ProjectOverview {
  summary: {
    totalProjects: number;
    activeProjects: number;
    templateProjects: number;
    archivedProjects: number;
    avgProjectAgeDays: number;
  };
  statusDistribution: Array<{
    is_active: boolean;
    is_template: boolean;
    count: string;
  }>;
  typeDistribution: Array<{
    type: string;
    count: string;
  }>;
  creationTrend: Array<{
    date: string;
    count: string;
  }>;
  mostActiveProjects: Array<{
    id: string;
    name: string;
    owner_id: string;
    owner_name: string;
    created_at: string;
    updated_at: string;
    is_active: boolean;
    is_template: boolean;
    type: string;
    tags: string[];
    hours_since_update: string;
  }>;
  ownerDistribution: Array<{
    username: string;
    email: string;
    project_count: string;
  }>;
  recentActivity: Array<{
    id: string;
    name: string;
    updated_at: string;
    updated_by: string | null;
  }>;
  lastUpdated: string;
}

interface ModuleOverview {
  summary: {
    totalModules: number;
    activeModules: number;
    publicModules: number;
    defaultModules: number;
    avgModuleAgeDays: number;
  };
  statusDistribution: Array<{
    is_active: boolean;
    is_public: boolean;
    is_default: boolean;
    count: string;
  }>;
  typeDistribution: Array<{
    type: string;
    count: string;
  }>;
  categoryDistribution: Array<{
    category: string;
    count: string;
  }>;
  popularModules: Array<{
    id: string;
    name: string;
    type: string;
    category: string;
    is_active: boolean;
    is_public: boolean;
    is_default: boolean;
    created_at: string;
    updated_at: string;
    usage_count: string;
  }>;
  robotSupport: Array<{
    robot_type: string;
    module_count: string;
  }>;
  recentActivity: Array<{
    id: string;
    name: string;
    updated_at: string;
    type: string;
    category: string;
  }>;
  usageDistribution: Array<{
    usage_bucket: string;
    module_count: string;
  }>;
  lastUpdated: string;
}

interface TemplateOverview {
  summary: {
    totalTemplates: number;
    activeTemplates: number;
    avgTemplateAgeDays: number;
  };
  statusDistribution: Array<{
    is_active: boolean;
    count: string;
  }>;
  typeDistribution: Array<{
    type: string;
    count: string;
  }>;
  popularTemplates: Array<{
    id: string;
    name: string;
    type: string;
    is_active: boolean;
    created_at: string;
    updated_at: string;
    owner_id: string;
    owner_name: string;
    tags: string[];
    hours_since_update: string;
  }>;
  recentActivity: Array<{
    id: string;
    name: string;
    updated_at: string;
    type: string;
    updated_by: string;
  }>;
  creationTrend: Array<{
    date: string;
    count: string;
  }>;
  ownerDistribution: Array<{
    username: string;
    email: string;
    template_count: string;
  }>;
  lastUpdated: string;
}

interface RobotOverview {
  summary: {
    totalRobotTypes: number;
    totalModulesWithRobots: number;
    totalProjectsUsingRobots: number;
    avgModulesPerRobot: number;
  };
  typeDistribution: Array<{
    robot_type: string;
    module_count: string;
  }>;
  utilizationMetrics: Array<{
    robot_type: string;
    project_count: string;
  }>;
  popularRobots: Array<{
    robot_type: string;
    module_count: string;
    project_count: string;
  }>;
  deploymentDistribution: Array<{
    deployment_bucket: string;
    robot_count: string;
  }>;
  recentActivity: Array<{
    id: string;
    name: string;
    type: string;
    category: string;
    updated_at: string;
    supported_robots: string[];
  }>;
  categoryDistribution: Array<{
    category: string;
    robot_count: string;
  }>;
  lastUpdated: string;
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
  const [loading, setLoading] = useState(true);
  const [users, setUsers] = useState<User[]>([]);
  const [projects, setProjects] = useState<Project[]>([]);
  const [systemMetrics, setSystemMetrics] = useState<SystemMetrics | null>(
    null
  );
  const [projectOverview, setProjectOverview] =
    useState<ProjectOverview | null>(null);
  const [moduleOverview, setModuleOverview] = useState<ModuleOverview | null>(
    null
  );
  const [templateOverview, setTemplateOverview] =
    useState<TemplateOverview | null>(null);
  const [robotOverview, setRobotOverview] = useState<RobotOverview | null>(
    null
  );
  const [error, setError] = useState<string | null>(null);
  const [snackbar, setSnackbar] = useState<{
    open: boolean;
    message: string;
    severity: 'success' | 'error';
  }>({
    open: false,
    message: '',
    severity: 'success',
  });

  // User management states
  const [selectedUser, setSelectedUser] = useState<User | null>(null);
  const [userActionDialog, setUserActionDialog] = useState(false);
  const [userActionLoading, setUserActionLoading] = useState(false);
  const [createUserDialog, setCreateUserDialog] = useState(false);
  const [createUserLoading, setCreateUserLoading] = useState(false);
  const [createUserForm, setCreateUserForm] = useState({
    username: '',
    email: '',
    password: '',
    role: 'user' as 'user' | 'admin',
  });

  // Robot management states
  const [robots, setRobots] = useState<any[]>([]);
  const [addRobotDialogOpen, setAddRobotDialogOpen] = useState(false);
  const [editRobotDialogOpen, setEditRobotDialogOpen] = useState(false);
  const [deleteRobotDialogOpen, setDeleteRobotDialogOpen] = useState(false);
  const [selectedRobot, setSelectedRobot] = useState<any>(null);
  const [newRobot, setNewRobot] = useState({
    code: '',
    name: '',
    description: '',
  });

  useEffect(() => {
    loadDashboardData();
  }, []);

  // Load robots when robots management tab is selected
  useEffect(() => {
    if (tabValue === 4) {
      // Robots Management tab
      loadRobots();
    }
  }, [tabValue]);

  const loadDashboardData = async () => {
    setLoading(true);
    setError(null);
    try {
      // Load users
      const users = await ApiService.get<User[]>('/admin/users?limit=100');
      setUsers(users || []);

      // Load projects
      const projects = await ApiService.get<Project[]>('/projects');
      setProjects(projects || []);

      // Load system metrics from enhanced dashboard stats
      const stats = await ApiService.get<any>('/dashboard/stats');

      setSystemMetrics({
        totalUsers: stats.totalUsers || 0,
        activeUsers: stats.activeUsers || 0,
        totalProjects: stats.totalProjects || 0,
        activeProjects: stats.activeProjects || 0,
        totalStorage: stats.totalStorage || 0,
        usedStorage: stats.usedStorage || 0,
        cpuUsage: stats.systemHealth?.cpuUsage || 0,
        memoryUsage: stats.systemHealth?.memoryUsage || 0,
        networkUsage: stats.systemHealth?.networkUsage || 0,
        systemStatus: stats.systemHealth?.systemStatus || 'healthy',
      });

      // Load project overview data
      const projectOverviewData = await ApiService.get<ProjectOverview>(
        '/admin/projects/overview'
      );
      setProjectOverview(projectOverviewData);

      // Load module overview data
      const moduleOverviewData = await ApiService.get<ModuleOverview>(
        '/admin/modules/overview'
      );
      setModuleOverview(moduleOverviewData);

      // Load template overview data
      const templateOverviewData = await ApiService.get<TemplateOverview>(
        '/admin/templates/overview'
      );
      setTemplateOverview(templateOverviewData);

      // Load robot overview data
      const robotOverviewData = await ApiService.get<RobotOverview>(
        '/admin/robots/overview'
      );
      setRobotOverview(robotOverviewData);
    } catch (err) {
      setError(
        err instanceof Error ? err.message : 'Failed to load dashboard data'
      );
    } finally {
      setLoading(false);
    }
  };

  const handleUserAction = async (
    userId: string,
    action: 'promote' | 'demote' | 'disable' | 'enable'
  ) => {
    setUserActionLoading(true);
    try {
      await ApiService.post(`/admin/users/${userId}/actions`, { action });

      // Refresh users list
      await loadDashboardData();

      setSnackbar({
        open: true,
        message: `User ${action}d successfully`,
        severity: 'success',
      });
    } catch (err) {
      setSnackbar({
        open: true,
        message:
          err instanceof Error ? err.message : `Failed to ${action} user`,
        severity: 'error',
      });
    } finally {
      setUserActionLoading(false);
      setUserActionDialog(false);
      setSelectedUser(null);
    }
  };

  const handleUserDelete = async (userId: string) => {
    setUserActionLoading(true);
    try {
      await ApiService.delete(`/admin/users/${userId}`);

      // Refresh users list
      await loadDashboardData();

      setSnackbar({
        open: true,
        message: 'User deleted successfully',
        severity: 'success',
      });
    } catch (err) {
      setSnackbar({
        open: true,
        message: err instanceof Error ? err.message : 'Failed to delete user',
        severity: 'error',
      });
    } finally {
      setUserActionLoading(false);
      setUserActionDialog(false);
      setSelectedUser(null);
    }
  };

  const handleProjectAction = async (
    projectId: string,
    action: 'activate' | 'archive' | 'delete'
  ) => {
    try {
      await ApiService.post(`/admin/projects/${projectId}/actions`, { action });

      // Refresh projects list
      await loadDashboardData();

      setSnackbar({
        open: true,
        message: `Project ${action}d successfully`,
        severity: 'success',
      });
    } catch (err) {
      setSnackbar({
        open: true,
        message:
          err instanceof Error ? err.message : `Failed to ${action} project`,
        severity: 'error',
      });
    }
  };

  const handleModuleAction = async (
    moduleId: string,
    action: 'activate' | 'deactivate' | 'delete'
  ) => {
    try {
      await ApiService.post(`/admin/modules/${moduleId}/actions`, { action });

      // Refresh modules list
      await loadDashboardData();

      setSnackbar({
        open: true,
        message: `Module ${action}d successfully`,
        severity: 'success',
      });
    } catch (err) {
      setSnackbar({
        open: true,
        message:
          err instanceof Error ? err.message : `Failed to ${action} module`,
        severity: 'error',
      });
    }
  };

  const getRoleIcon = (role: string) => {
    switch (role) {
      case 'admin':
        return <AdminIcon color="error" />;
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
        return 'warning';
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

  const handleTabChange = (event: React.SyntheticEvent, newValue: number) => {
    setTabValue(newValue);
  };

  const openUserActionDialog = (user: User) => {
    setSelectedUser(user);
    setUserActionDialog(true);
  };

  const closeUserActionDialog = () => {
    setUserActionDialog(false);
    setSelectedUser(null);
  };

  const openCreateUserDialog = () => {
    setCreateUserDialog(true);
    setCreateUserForm({
      username: '',
      email: '',
      password: '',
      role: 'user',
    });
  };

  const closeCreateUserDialog = () => {
    setCreateUserDialog(false);
    setCreateUserForm({
      username: '',
      email: '',
      password: '',
      role: 'user',
    });
  };

  const handleCreateUser = async () => {
    setCreateUserLoading(true);
    try {
      await ApiService.post('/admin/users', createUserForm);

      // Refresh users list
      await loadDashboardData();

      setSnackbar({
        open: true,
        message: 'User created successfully',
        severity: 'success',
      });

      closeCreateUserDialog();
    } catch (err) {
      setSnackbar({
        open: true,
        message: err instanceof Error ? err.message : 'Failed to create user',
        severity: 'error',
      });
    } finally {
      setCreateUserLoading(false);
    }
  };

  const handleCreateUserFormChange = (
    field: keyof typeof createUserForm,
    value: string
  ) => {
    setCreateUserForm((prev) => ({
      ...prev,
      [field]: value,
    }));
  };

  // Robot management functions
  const loadRobots = async () => {
    try {
      const response = await ApiService.get('/admin/robots');
      console.log('Robots data received:', response);
      setRobots(response || []);
    } catch (error) {
      console.error('Failed to load robots:', error);
      setRobots([] as any[]);
    }
  };

  const handleAddRobot = async () => {
    try {
      await ApiService.post('/admin/robots', newRobot);
      setSnackbar({
        open: true,
        message: 'Robot added successfully',
        severity: 'success',
      });
      setAddRobotDialogOpen(false);
      setNewRobot({ code: '', name: '', description: '' });
      await loadRobots();
      await loadDashboardData(); // Refresh dashboard data
    } catch (err) {
      setSnackbar({
        open: true,
        message: err instanceof Error ? err.message : 'Failed to add robot',
        severity: 'error',
      });
    }
  };

  const handleEditRobot = async () => {
    if (!selectedRobot) return;
    try {
      await ApiService.put(`/admin/robots/${selectedRobot.robot_type}`, {
        name: selectedRobot.name,
        description: selectedRobot.description || '',
      });
      setSnackbar({
        open: true,
        message: 'Robot updated successfully',
        severity: 'success',
      });
      setEditRobotDialogOpen(false);
      setSelectedRobot(null);
      await loadRobots();
      await loadDashboardData(); // Refresh dashboard data
    } catch (err) {
      setSnackbar({
        open: true,
        message: err instanceof Error ? err.message : 'Failed to update robot',
        severity: 'error',
      });
    }
  };

  const handleDeleteRobot = async () => {
    if (!selectedRobot) return;
    try {
      await ApiService.delete(`/admin/robots/${selectedRobot.robot_type}`);
      setSnackbar({
        open: true,
        message: 'Robot deleted successfully',
        severity: 'success',
      });
      setDeleteRobotDialogOpen(false);
      setSelectedRobot(null);
      await loadRobots();
      await loadDashboardData(); // Refresh dashboard data
    } catch (err) {
      setSnackbar({
        open: true,
        message: err instanceof Error ? err.message : 'Failed to delete robot',
        severity: 'error',
      });
    }
  };

  const openEditRobotDialog = (robot: any) => {
    setSelectedRobot({
      ...robot,
      name:
        robot.name ||
        (robot.robot_type
          ? robot.robot_type
              .replace(/_/g, ' ')
              .replace(/\b\w/g, (l: string) => l.toUpperCase())
          : robot.code || 'Unknown Robot'),
      description: robot.description || '',
    });
    setEditRobotDialogOpen(true);
  };

  const openDeleteRobotDialog = (robot: any) => {
    setSelectedRobot(robot);
    setDeleteRobotDialogOpen(true);
  };

  if (loading) {
    return (
      <Box
        sx={{
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          height: '50vh',
        }}
      >
        <CircularProgress />
      </Box>
    );
  }

  if (error) {
    return (
      <Box sx={{ p: 3 }}>
        <Alert severity="error">{error}</Alert>
      </Box>
    );
  }

  return (
    <Box>
      {/* Header */}
      <Box
        sx={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          mb: 3,
        }}
      >
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
        </Box>
      </Box>

      {/* Error Alert */}
      {error && (
        <Alert severity="error" sx={{ mb: 3 }} onClose={() => setError(null)}>
          {error}
        </Alert>
      )}

      {/* System Overview Cards */}
      {systemMetrics && (
        <Grid container spacing={3} sx={{ mb: 3 }}>
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
                    <Typography color="textSecondary" gutterBottom>
                      Total Users
                    </Typography>
                    <Typography variant="h4">
                      {systemMetrics.totalUsers}
                    </Typography>
                    <Typography variant="body2" color="textSecondary">
                      {systemMetrics.activeUsers} active
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
                <Box
                  sx={{
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'space-between',
                  }}
                >
                  <Box>
                    <Typography color="textSecondary" gutterBottom>
                      Total Projects
                    </Typography>
                    <Typography variant="h4">
                      {systemMetrics.totalProjects}
                    </Typography>
                    <Typography variant="body2" color="textSecondary">
                      {systemMetrics.activeProjects} active
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
                <Box
                  sx={{
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'space-between',
                  }}
                >
                  <Box>
                    <Typography color="textSecondary" gutterBottom>
                      Storage Used
                    </Typography>
                    <Typography variant="h4">
                      {formatBytes(systemMetrics.usedStorage)}
                    </Typography>
                    <Typography variant="body2" color="textSecondary">
                      {(
                        (systemMetrics.usedStorage /
                          systemMetrics.totalStorage) *
                        100
                      ).toFixed(1)}
                      % of total
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
                <Box
                  sx={{
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'space-between',
                  }}
                >
                  <Box>
                    <Typography color="textSecondary" gutterBottom>
                      System Status
                    </Typography>
                    <Chip
                      label={systemMetrics.systemStatus}
                      color={
                        getSystemStatusColor(systemMetrics.systemStatus) as any
                      }
                      sx={{ mb: 1 }}
                    />
                    <Typography variant="body2" color="textSecondary">
                      CPU: {systemMetrics.cpuUsage}% | RAM:{' '}
                      {systemMetrics.memoryUsage}%
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
          <Tabs value={tabValue} onChange={handleTabChange}>
            <Tab icon={<PeopleIcon />} label="User Management" />
            <Tab icon={<ProjectIcon />} label="Project Oversight" />
            <Tab icon={<SettingsIcon />} label="Modules Oversight" />
            <Tab icon={<TemplateIcon />} label="Templates Oversight" />
            <Tab icon={<RobotIcon />} label="Robots Management" />
            <Tab icon={<RobotIcon />} label="Robots Oversight" />
            <Tab icon={<AnalyticsIcon />} label="System Metrics" />
            <Tab icon={<SecurityIcon />} label="Security & Access" />
          </Tabs>
        </Box>

        {/* User Management Tab */}
        <TabPanel value={tabValue} index={0}>
          <Box
            sx={{
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
              mb: 3,
            }}
          >
            <Typography variant="h6">User Management</Typography>
            <Button
              variant="contained"
              startIcon={<PeopleIcon />}
              onClick={openCreateUserDialog}
            >
              Add User
            </Button>
          </Box>

          <TableContainer component={Paper}>
            <Table>
              <TableHead>
                <TableRow>
                  <TableCell>User</TableCell>
                  <TableCell>Role</TableCell>
                  <TableCell>Created</TableCell>
                  <TableCell>Updated</TableCell>
                  <TableCell>Actions</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {users.map((user) => (
                  <TableRow key={user.id}>
                    <TableCell>
                      <Box
                        sx={{ display: 'flex', alignItems: 'center', gap: 2 }}
                      >
                        <Avatar>{user.username.charAt(0).toUpperCase()}</Avatar>
                        <Box>
                          <Typography variant="subtitle2">
                            {user.username}
                          </Typography>
                          <Typography variant="body2" color="textSecondary">
                            {user.email}
                          </Typography>
                        </Box>
                      </Box>
                    </TableCell>
                    <TableCell>
                      <Box
                        sx={{ display: 'flex', alignItems: 'center', gap: 1 }}
                      >
                        {getRoleIcon(user.role)}
                        <Chip label={user.role} size="small" />
                      </Box>
                    </TableCell>
                    <TableCell>{formatDate(user.createdAt)}</TableCell>
                    <TableCell>{formatDate(user.updatedAt)}</TableCell>
                    <TableCell>
                      <Box sx={{ display: 'flex', gap: 1 }}>
                        <Tooltip title="Edit User">
                          <IconButton
                            size="small"
                            onClick={() => openUserActionDialog(user)}
                          >
                            <EditIcon />
                          </IconButton>
                        </Tooltip>
                        {user.role === 'user' ? (
                          <Tooltip title="Promote to Admin">
                            <IconButton
                              size="small"
                              color="primary"
                              onClick={() =>
                                handleUserAction(user.id, 'promote')
                              }
                            >
                              <AdminIcon />
                            </IconButton>
                          </Tooltip>
                        ) : (
                          <Tooltip title="Demote to User">
                            <IconButton
                              size="small"
                              color="warning"
                              onClick={() =>
                                handleUserAction(user.id, 'demote')
                              }
                            >
                              <PersonIcon />
                            </IconButton>
                          </Tooltip>
                        )}
                        <Tooltip title="Delete User">
                          <IconButton
                            size="small"
                            color="error"
                            onClick={() => handleUserDelete(user.id)}
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
          <Box
            sx={{
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
              mb: 3,
            }}
          >
            <Typography variant="h6">Project Oversight</Typography>
            <Button
              variant="contained"
              startIcon={<RefreshIcon />}
              onClick={loadDashboardData}
            >
              Refresh Data
            </Button>
          </Box>

          {projectOverview ? (
            <>
              {/* Project Summary Cards */}
              <Grid container spacing={3} sx={{ mb: 3 }}>
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
                          <Typography color="textSecondary" gutterBottom>
                            Total Projects
                          </Typography>
                          <Typography variant="h4">
                            {projectOverview.summary.totalProjects}
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
                      <Box
                        sx={{
                          display: 'flex',
                          alignItems: 'center',
                          justifyContent: 'space-between',
                        }}
                      >
                        <Box>
                          <Typography color="textSecondary" gutterBottom>
                            Active Projects
                          </Typography>
                          <Typography variant="h4">
                            {projectOverview.summary.activeProjects}
                          </Typography>
                        </Box>
                        <CheckCircleIcon
                          color="success"
                          sx={{ fontSize: 40 }}
                        />
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
                          <Typography color="textSecondary" gutterBottom>
                            Template Projects
                          </Typography>
                          <Typography variant="h4">
                            {projectOverview.summary.templateProjects}
                          </Typography>
                        </Box>
                        <TemplateIcon color="info" sx={{ fontSize: 40 }} />
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
                          <Typography color="textSecondary" gutterBottom>
                            Avg Age (Days)
                          </Typography>
                          <Typography variant="h4">
                            {projectOverview.summary.avgProjectAgeDays.toFixed(
                              1
                            )}
                          </Typography>
                        </Box>
                        <TimelineIcon color="secondary" sx={{ fontSize: 40 }} />
                      </Box>
                    </CardContent>
                  </Card>
                </Grid>
              </Grid>

              {/* Most Active Projects */}
              <Card sx={{ mb: 3 }}>
                <CardHeader
                  title="Most Active Projects"
                  subheader="Projects with recent activity"
                />
                <CardContent>
                  <TableContainer>
                    <Table>
                      <TableHead>
                        <TableRow>
                          <TableCell>Project Name</TableCell>
                          <TableCell>Owner</TableCell>
                          <TableCell>Type</TableCell>
                          <TableCell>Status</TableCell>
                          <TableCell>Tags</TableCell>
                          <TableCell>Last Updated</TableCell>
                          <TableCell>Actions</TableCell>
                        </TableRow>
                      </TableHead>
                      <TableBody>
                        {projectOverview.mostActiveProjects.map((project) => (
                          <TableRow key={project.id}>
                            <TableCell>
                              <Typography variant="subtitle2">
                                {project.name}
                              </Typography>
                            </TableCell>
                            <TableCell>{project.owner_name}</TableCell>
                            <TableCell>
                              <Chip
                                label={project.type}
                                size="small"
                                variant="outlined"
                              />
                            </TableCell>
                            <TableCell>
                              <Chip
                                label={
                                  project.is_active ? 'Active' : 'Inactive'
                                }
                                color={
                                  project.is_active ? 'success' : 'default'
                                }
                                size="small"
                              />
                            </TableCell>
                            <TableCell>
                              {project.tags && project.tags.length > 0 ? (
                                <Box sx={{ display: 'flex', gap: 0.5 }}>
                                  {project.tags
                                    .slice(0, 2)
                                    .map((tag, index) => (
                                      <Chip
                                        key={index}
                                        label={tag}
                                        size="small"
                                        variant="outlined"
                                      />
                                    ))}
                                  {project.tags.length > 2 && (
                                    <Chip
                                      label={`+${project.tags.length - 2}`}
                                      size="small"
                                      variant="outlined"
                                    />
                                  )}
                                </Box>
                              ) : (
                                <Typography
                                  variant="body2"
                                  color="textSecondary"
                                >
                                  No tags
                                </Typography>
                              )}
                            </TableCell>
                            <TableCell>
                              <Typography variant="body2">
                                {formatDate(project.updated_at)}
                              </Typography>
                              <Typography
                                variant="caption"
                                color="textSecondary"
                              >
                                {parseFloat(project.hours_since_update).toFixed(
                                  1
                                )}
                                h ago
                              </Typography>
                            </TableCell>
                            <TableCell>
                              <Box sx={{ display: 'flex', gap: 1 }}>
                                <Tooltip title="View Project">
                                  <IconButton size="small">
                                    <VisibilityIcon />
                                  </IconButton>
                                </Tooltip>
                                <Tooltip title="Edit Project">
                                  <IconButton size="small">
                                    <EditIcon />
                                  </IconButton>
                                </Tooltip>
                                <Tooltip title="Delete Project">
                                  <IconButton
                                    size="small"
                                    color="error"
                                    onClick={() =>
                                      handleProjectAction(project.id, 'delete')
                                    }
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
                </CardContent>
              </Card>

              {/* Recent Activity */}
              <Card sx={{ mb: 3 }}>
                <CardHeader
                  title="Recent Activity"
                  subheader="Projects updated in the last 7 days"
                />
                <CardContent>
                  <TableContainer>
                    <Table>
                      <TableHead>
                        <TableRow>
                          <TableCell>Project Name</TableCell>
                          <TableCell>Updated By</TableCell>
                          <TableCell>Last Updated</TableCell>
                        </TableRow>
                      </TableHead>
                      <TableBody>
                        {projectOverview.recentActivity.map((activity) => (
                          <TableRow key={activity.id}>
                            <TableCell>
                              <Typography variant="subtitle2">
                                {activity.name}
                              </Typography>
                            </TableCell>
                            <TableCell>
                              {activity.updated_by || 'System'}
                            </TableCell>
                            <TableCell>
                              {formatDate(activity.updated_at)}
                            </TableCell>
                          </TableRow>
                        ))}
                      </TableBody>
                    </Table>
                  </TableContainer>
                </CardContent>
              </Card>

              {/* Owner Distribution */}
              <Card>
                <CardHeader
                  title="Project Distribution by Owner"
                  subheader="Number of projects per user"
                />
                <CardContent>
                  <TableContainer>
                    <Table>
                      <TableHead>
                        <TableRow>
                          <TableCell>Username</TableCell>
                          <TableCell>Email</TableCell>
                          <TableCell>Project Count</TableCell>
                        </TableRow>
                      </TableHead>
                      <TableBody>
                        {projectOverview.ownerDistribution.map(
                          (owner, index) => (
                            <TableRow key={index}>
                              <TableCell>
                                <Typography variant="subtitle2">
                                  {owner.username}
                                </Typography>
                              </TableCell>
                              <TableCell>{owner.email}</TableCell>
                              <TableCell>
                                <Chip
                                  label={owner.project_count}
                                  color="primary"
                                  size="small"
                                />
                              </TableCell>
                            </TableRow>
                          )
                        )}
                      </TableBody>
                    </Table>
                  </TableContainer>
                </CardContent>
              </Card>
            </>
          ) : (
            <Box sx={{ display: 'flex', justifyContent: 'center', p: 4 }}>
              <CircularProgress />
            </Box>
          )}
        </TabPanel>

        {/* Modules Oversight Tab */}
        <TabPanel value={tabValue} index={2}>
          <Box
            sx={{
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
              mb: 3,
            }}
          >
            <Typography variant="h6">Modules Oversight</Typography>
            <Button
              variant="contained"
              startIcon={<RefreshIcon />}
              onClick={loadDashboardData}
            >
              Refresh Data
            </Button>
          </Box>

          {moduleOverview ? (
            <>
              {/* Module Summary Cards */}
              <Grid container spacing={3} sx={{ mb: 3 }}>
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
                          <Typography color="textSecondary" gutterBottom>
                            Total Modules
                          </Typography>
                          <Typography variant="h4">
                            {moduleOverview.summary.totalModules}
                          </Typography>
                        </Box>
                        <SettingsIcon color="primary" sx={{ fontSize: 40 }} />
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
                          <Typography color="textSecondary" gutterBottom>
                            Active Modules
                          </Typography>
                          <Typography variant="h4">
                            {moduleOverview.summary.activeModules}
                          </Typography>
                        </Box>
                        <CheckCircleIcon
                          color="success"
                          sx={{ fontSize: 40 }}
                        />
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
                          <Typography color="textSecondary" gutterBottom>
                            Public Modules
                          </Typography>
                          <Typography variant="h4">
                            {moduleOverview.summary.publicModules}
                          </Typography>
                        </Box>
                        <VisibilityIcon color="info" sx={{ fontSize: 40 }} />
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
                          <Typography color="textSecondary" gutterBottom>
                            Avg Age (Days)
                          </Typography>
                          <Typography variant="h4">
                            {moduleOverview.summary.avgModuleAgeDays.toFixed(1)}
                          </Typography>
                        </Box>
                        <TimelineIcon color="secondary" sx={{ fontSize: 40 }} />
                      </Box>
                    </CardContent>
                  </Card>
                </Grid>
              </Grid>

              {/* Most Popular Modules */}
              <Card sx={{ mb: 3 }}>
                <CardHeader
                  title="Most Popular Modules"
                  subheader="Modules by usage in projects"
                />
                <CardContent>
                  <TableContainer>
                    <Table>
                      <TableHead>
                        <TableRow>
                          <TableCell>Module Name</TableCell>
                          <TableCell>Type</TableCell>
                          <TableCell>Category</TableCell>
                          <TableCell>Status</TableCell>
                          <TableCell>Usage Count</TableCell>
                          <TableCell>Last Updated</TableCell>
                          <TableCell>Actions</TableCell>
                        </TableRow>
                      </TableHead>
                      <TableBody>
                        {moduleOverview.popularModules.map((module) => (
                          <TableRow key={module.id}>
                            <TableCell>
                              <Typography variant="subtitle2">
                                {module.name}
                              </Typography>
                            </TableCell>
                            <TableCell>
                              <Chip
                                label={module.type}
                                size="small"
                                variant="outlined"
                              />
                            </TableCell>
                            <TableCell>
                              <Chip
                                label={module.category}
                                size="small"
                                variant="outlined"
                              />
                            </TableCell>
                            <TableCell>
                              <Box sx={{ display: 'flex', gap: 0.5 }}>
                                <Chip
                                  label={
                                    module.is_active ? 'Active' : 'Inactive'
                                  }
                                  color={
                                    module.is_active ? 'success' : 'default'
                                  }
                                  size="small"
                                />
                                {module.is_public && (
                                  <Chip
                                    label="Public"
                                    color="info"
                                    size="small"
                                  />
                                )}
                                {module.is_default && (
                                  <Chip
                                    label="Default"
                                    color="warning"
                                    size="small"
                                  />
                                )}
                              </Box>
                            </TableCell>
                            <TableCell>
                              <Chip
                                label={module.usage_count}
                                color="primary"
                                size="small"
                              />
                            </TableCell>
                            <TableCell>
                              {formatDate(module.updated_at)}
                            </TableCell>
                            <TableCell>
                              <Box sx={{ display: 'flex', gap: 1 }}>
                                <Tooltip title="View Module">
                                  <IconButton size="small">
                                    <VisibilityIcon />
                                  </IconButton>
                                </Tooltip>
                                <Tooltip title="Edit Module">
                                  <IconButton size="small">
                                    <EditIcon />
                                  </IconButton>
                                </Tooltip>
                                <Tooltip title="Delete Module">
                                  <IconButton
                                    size="small"
                                    color="error"
                                    onClick={() =>
                                      handleModuleAction(module.id, 'delete')
                                    }
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
                </CardContent>
              </Card>

              {/* Recent Activity */}
              <Card sx={{ mb: 3 }}>
                <CardHeader
                  title="Recent Module Activity"
                  subheader="Modules updated in the last 30 days"
                />
                <CardContent>
                  <TableContainer>
                    <Table>
                      <TableHead>
                        <TableRow>
                          <TableCell>Module Name</TableCell>
                          <TableCell>Type</TableCell>
                          <TableCell>Category</TableCell>
                          <TableCell>Last Updated</TableCell>
                        </TableRow>
                      </TableHead>
                      <TableBody>
                        {moduleOverview.recentActivity.map((activity) => (
                          <TableRow key={activity.id}>
                            <TableCell>
                              <Typography variant="subtitle2">
                                {activity.name}
                              </Typography>
                            </TableCell>
                            <TableCell>
                              <Chip
                                label={activity.type}
                                size="small"
                                variant="outlined"
                              />
                            </TableCell>
                            <TableCell>
                              <Chip
                                label={activity.category}
                                size="small"
                                variant="outlined"
                              />
                            </TableCell>
                            <TableCell>
                              {formatDate(activity.updated_at)}
                            </TableCell>
                          </TableRow>
                        ))}
                      </TableBody>
                    </Table>
                  </TableContainer>
                </CardContent>
              </Card>

              {/* Usage Distribution */}
              <Card>
                <CardHeader
                  title="Module Usage Distribution"
                  subheader="How modules are being used across projects"
                />
                <CardContent>
                  <TableContainer>
                    <Table>
                      <TableHead>
                        <TableRow>
                          <TableCell>Usage Level</TableCell>
                          <TableCell>Module Count</TableCell>
                        </TableRow>
                      </TableHead>
                      <TableBody>
                        {moduleOverview.usageDistribution.map(
                          (usage, index) => (
                            <TableRow key={index}>
                              <TableCell>
                                <Typography variant="subtitle2">
                                  {usage.usage_bucket}
                                </Typography>
                              </TableCell>
                              <TableCell>
                                <Chip
                                  label={usage.module_count}
                                  color="primary"
                                  size="small"
                                />
                              </TableCell>
                            </TableRow>
                          )
                        )}
                      </TableBody>
                    </Table>
                  </TableContainer>
                </CardContent>
              </Card>
            </>
          ) : (
            <Box sx={{ display: 'flex', justifyContent: 'center', p: 4 }}>
              <CircularProgress />
            </Box>
          )}
        </TabPanel>

        {/* Templates Oversight Tab */}
        <TabPanel value={tabValue} index={3}>
          <Box
            sx={{
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
              mb: 3,
            }}
          >
            <Typography variant="h6">Templates Oversight</Typography>
            <Button
              variant="contained"
              startIcon={<RefreshIcon />}
              onClick={loadDashboardData}
            >
              Refresh Data
            </Button>
          </Box>

          {templateOverview ? (
            <>
              {/* Template Summary Cards */}
              <Grid container spacing={3} sx={{ mb: 3 }}>
                <Grid item xs={12} sm={6} md={4}>
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
                          <Typography color="textSecondary" gutterBottom>
                            Total Templates
                          </Typography>
                          <Typography variant="h4">
                            {templateOverview.summary.totalTemplates}
                          </Typography>
                        </Box>
                        <TemplateIcon color="primary" sx={{ fontSize: 40 }} />
                      </Box>
                    </CardContent>
                  </Card>
                </Grid>

                <Grid item xs={12} sm={6} md={4}>
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
                          <Typography color="textSecondary" gutterBottom>
                            Active Templates
                          </Typography>
                          <Typography variant="h4">
                            {templateOverview.summary.activeTemplates}
                          </Typography>
                        </Box>
                        <CheckCircleIcon
                          color="success"
                          sx={{ fontSize: 40 }}
                        />
                      </Box>
                    </CardContent>
                  </Card>
                </Grid>

                <Grid item xs={12} sm={6} md={4}>
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
                          <Typography color="textSecondary" gutterBottom>
                            Avg Age (Days)
                          </Typography>
                          <Typography variant="h4">
                            {templateOverview.summary.avgTemplateAgeDays.toFixed(
                              1
                            )}
                          </Typography>
                        </Box>
                        <TimelineIcon color="secondary" sx={{ fontSize: 40 }} />
                      </Box>
                    </CardContent>
                  </Card>
                </Grid>
              </Grid>

              {/* Most Popular Templates */}
              <Card sx={{ mb: 3 }}>
                <CardHeader
                  title="Most Popular Templates"
                  subheader="Templates by recent activity"
                />
                <CardContent>
                  <TableContainer>
                    <Table>
                      <TableHead>
                        <TableRow>
                          <TableCell>Template Name</TableCell>
                          <TableCell>Type</TableCell>
                          <TableCell>Owner</TableCell>
                          <TableCell>Status</TableCell>
                          <TableCell>Tags</TableCell>
                          <TableCell>Last Updated</TableCell>
                          <TableCell>Actions</TableCell>
                        </TableRow>
                      </TableHead>
                      <TableBody>
                        {templateOverview.popularTemplates.map((template) => (
                          <TableRow key={template.id}>
                            <TableCell>
                              <Typography variant="subtitle2">
                                {template.name}
                              </Typography>
                            </TableCell>
                            <TableCell>
                              <Chip
                                label={template.type}
                                size="small"
                                variant="outlined"
                              />
                            </TableCell>
                            <TableCell>
                              <Typography variant="body2">
                                {template.owner_name}
                              </Typography>
                            </TableCell>
                            <TableCell>
                              <Chip
                                label={
                                  template.is_active ? 'Active' : 'Inactive'
                                }
                                color={
                                  template.is_active ? 'success' : 'default'
                                }
                                size="small"
                              />
                            </TableCell>
                            <TableCell>
                              <Box
                                sx={{
                                  display: 'flex',
                                  gap: 0.5,
                                  flexWrap: 'wrap',
                                }}
                              >
                                {template.tags && template.tags.length > 0 ? (
                                  template.tags.map((tag, index) => (
                                    <Chip
                                      key={index}
                                      label={tag}
                                      size="small"
                                      variant="outlined"
                                    />
                                  ))
                                ) : (
                                  <Typography
                                    variant="body2"
                                    color="textSecondary"
                                  >
                                    No tags
                                  </Typography>
                                )}
                              </Box>
                            </TableCell>
                            <TableCell>
                              {formatDate(template.updated_at)}
                            </TableCell>
                            <TableCell>
                              <Box sx={{ display: 'flex', gap: 1 }}>
                                <Tooltip title="View Template">
                                  <IconButton size="small">
                                    <VisibilityIcon />
                                  </IconButton>
                                </Tooltip>
                                <Tooltip title="Edit Template">
                                  <IconButton size="small">
                                    <EditIcon />
                                  </IconButton>
                                </Tooltip>
                                <Tooltip title="Use Template">
                                  <IconButton size="small">
                                    <AddIcon />
                                  </IconButton>
                                </Tooltip>
                                <Tooltip title="Delete Template">
                                  <IconButton size="small" color="error">
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
                </CardContent>
              </Card>

              {/* Recent Activity */}
              <Card sx={{ mb: 3 }}>
                <CardHeader
                  title="Recent Template Activity"
                  subheader="Templates updated in the last 30 days"
                />
                <CardContent>
                  <TableContainer>
                    <Table>
                      <TableHead>
                        <TableRow>
                          <TableCell>Template Name</TableCell>
                          <TableCell>Type</TableCell>
                          <TableCell>Updated By</TableCell>
                          <TableCell>Last Updated</TableCell>
                        </TableRow>
                      </TableHead>
                      <TableBody>
                        {templateOverview.recentActivity.map((activity) => (
                          <TableRow key={activity.id}>
                            <TableCell>
                              <Typography variant="subtitle2">
                                {activity.name}
                              </Typography>
                            </TableCell>
                            <TableCell>
                              <Chip
                                label={activity.type}
                                size="small"
                                variant="outlined"
                              />
                            </TableCell>
                            <TableCell>
                              <Typography variant="body2">
                                {activity.updated_by}
                              </Typography>
                            </TableCell>
                            <TableCell>
                              {formatDate(activity.updated_at)}
                            </TableCell>
                          </TableRow>
                        ))}
                      </TableBody>
                    </Table>
                  </TableContainer>
                </CardContent>
              </Card>

              {/* Owner Distribution */}
              <Card>
                <CardHeader
                  title="Template Owner Distribution"
                  subheader="Number of templates per user"
                />
                <CardContent>
                  <TableContainer>
                    <Table>
                      <TableHead>
                        <TableRow>
                          <TableCell>Username</TableCell>
                          <TableCell>Email</TableCell>
                          <TableCell>Template Count</TableCell>
                        </TableRow>
                      </TableHead>
                      <TableBody>
                        {templateOverview.ownerDistribution.map(
                          (owner, index) => (
                            <TableRow key={index}>
                              <TableCell>
                                <Typography variant="subtitle2">
                                  {owner.username}
                                </Typography>
                              </TableCell>
                              <TableCell>
                                <Typography variant="body2">
                                  {owner.email}
                                </Typography>
                              </TableCell>
                              <TableCell>
                                <Chip
                                  label={owner.template_count}
                                  color="primary"
                                  size="small"
                                />
                              </TableCell>
                            </TableRow>
                          )
                        )}
                      </TableBody>
                    </Table>
                  </TableContainer>
                </CardContent>
              </Card>
            </>
          ) : (
            <Box sx={{ display: 'flex', justifyContent: 'center', p: 4 }}>
              <CircularProgress />
            </Box>
          )}
        </TabPanel>

        {/* Robots Management Tab */}
        <TabPanel value={tabValue} index={4}>
          <Box
            sx={{
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
              mb: 3,
            }}
          >
            <Typography variant="h6">Robots Management</Typography>
            <Button
              variant="contained"
              startIcon={<AddIcon />}
              onClick={() => setAddRobotDialogOpen(true)}
            >
              Add Robot
            </Button>
          </Box>

          {/* Robots Management Table */}
          <TableContainer component={Paper}>
            <Table>
              <TableHead>
                <TableRow>
                  <TableCell>Robot</TableCell>
                  <TableCell>Code</TableCell>
                  <TableCell>Modules</TableCell>
                  <TableCell>Projects</TableCell>
                  <TableCell>First Used</TableCell>
                  <TableCell>Last Used</TableCell>
                  <TableCell align="right">Actions</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {robots.map((robot) => (
                  <TableRow key={robot.robot_type || robot.code}>
                    <TableCell>
                      <Typography variant="subtitle1">
                        {robot.name ||
                          (robot.robot_type
                            ? robot.robot_type
                                .replace(/_/g, ' ')
                                .replace(/\b\w/g, (l) => l.toUpperCase())
                            : robot.code || 'Unknown Robot')}
                      </Typography>
                    </TableCell>
                    <TableCell>
                      <Chip
                        label={robot.robot_type || robot.code || 'Unknown'}
                        size="small"
                        variant="outlined"
                      />
                    </TableCell>
                    <TableCell>
                      <Chip
                        label={`${robot.module_count} modules`}
                        size="small"
                        color="primary"
                      />
                    </TableCell>
                    <TableCell>
                      <Chip
                        label={`${robot.project_count} projects`}
                        size="small"
                        color="secondary"
                      />
                    </TableCell>
                    <TableCell>N/A</TableCell>
                    <TableCell>N/A</TableCell>
                    <TableCell align="right">
                      <Box display="flex" gap={1} justifyContent="flex-end">
                        <Tooltip title="Edit Robot">
                          <IconButton
                            size="small"
                            onClick={() => openEditRobotDialog(robot)}
                          >
                            <EditIcon />
                          </IconButton>
                        </Tooltip>
                        <Tooltip title="Delete Robot">
                          <IconButton
                            size="small"
                            color="error"
                            onClick={() => openDeleteRobotDialog(robot)}
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

        {/* Robots Oversight Tab */}
        <TabPanel value={tabValue} index={5}>
          <Box
            sx={{
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
              mb: 3,
            }}
          >
            <Box>
              <Typography variant="h6">Robot Management & Analytics</Typography>
              {robotOverview?.lastUpdated && (
                <Typography variant="body2" color="textSecondary">
                  Last updated: {formatDate(robotOverview.lastUpdated)}
                </Typography>
              )}
            </Box>
            <Button
              variant="outlined"
              onClick={loadDashboardData}
              disabled={loading}
              startIcon={<RefreshIcon />}
            >
              Refresh Data
            </Button>
          </Box>

          {robotOverview ? (
            <>
              {/* Summary Cards */}
              <Grid container spacing={3} sx={{ mb: 3 }}>
                <Grid item xs={12} sm={6} md={3}>
                  <Card>
                    <CardContent>
                      <Typography color="textSecondary" gutterBottom>
                        Total Robot Types
                      </Typography>
                      <Typography variant="h4">
                        {robotOverview.summary.totalRobotTypes}
                      </Typography>
                    </CardContent>
                  </Card>
                </Grid>
                <Grid item xs={12} sm={6} md={3}>
                  <Card>
                    <CardContent>
                      <Typography color="textSecondary" gutterBottom>
                        Modules with Robots
                      </Typography>
                      <Typography variant="h4">
                        {robotOverview.summary.totalModulesWithRobots}
                      </Typography>
                    </CardContent>
                  </Card>
                </Grid>
                <Grid item xs={12} sm={6} md={3}>
                  <Card>
                    <CardContent>
                      <Typography color="textSecondary" gutterBottom>
                        Projects Using Robots
                      </Typography>
                      <Typography variant="h4">
                        {robotOverview.summary.totalProjectsUsingRobots}
                      </Typography>
                    </CardContent>
                  </Card>
                </Grid>
                <Grid item xs={12} sm={6} md={3}>
                  <Card>
                    <CardContent>
                      <Typography color="textSecondary" gutterBottom>
                        Avg Modules/Robot
                      </Typography>
                      <Typography variant="h4">
                        {robotOverview.summary.avgModulesPerRobot.toFixed(1)}
                      </Typography>
                    </CardContent>
                  </Card>
                </Grid>
              </Grid>

              {/* Robot Type Distribution */}
              <Grid container spacing={3} sx={{ mb: 3 }}>
                <Grid item xs={12} md={6}>
                  <Card>
                    <CardContent>
                      <RobotTypeBarChart
                        data={robotOverview.typeDistribution}
                        height={300}
                      />
                    </CardContent>
                  </Card>
                </Grid>

                {/* Popular Robots */}
                <Grid item xs={12} md={6}>
                  <Card>
                    <CardContent>
                      <Typography variant="h6" gutterBottom>
                        Most Popular Robots
                      </Typography>
                      <TableContainer>
                        <Table size="small">
                          <TableHead>
                            <TableRow>
                              <TableCell>Robot Type</TableCell>
                              <TableCell align="right">Modules</TableCell>
                              <TableCell align="right">Projects</TableCell>
                            </TableRow>
                          </TableHead>
                          <TableBody>
                            {robotOverview.popularRobots
                              .slice(0, 5)
                              .map((robot, index) => (
                                <TableRow key={index}>
                                  <TableCell>{robot.robot_type}</TableCell>
                                  <TableCell align="right">
                                    {robot.module_count}
                                  </TableCell>
                                  <TableCell align="right">
                                    {robot.project_count}
                                  </TableCell>
                                </TableRow>
                              ))}
                          </TableBody>
                        </Table>
                      </TableContainer>
                    </CardContent>
                  </Card>
                </Grid>
              </Grid>

              {/* Recent Activity and Category Distribution */}
              <Grid container spacing={3}>
                <Grid item xs={12} md={8}>
                  <Card>
                    <CardContent>
                      <Typography variant="h6" gutterBottom>
                        Recent Robot-Related Activity
                      </Typography>
                      <TableContainer>
                        <Table size="small">
                          <TableHead>
                            <TableRow>
                              <TableCell>Module</TableCell>
                              <TableCell>Type</TableCell>
                              <TableCell>Category</TableCell>
                              <TableCell>Robots</TableCell>
                              <TableCell>Updated</TableCell>
                            </TableRow>
                          </TableHead>
                          <TableBody>
                            {robotOverview.recentActivity
                              .slice(0, 10)
                              .map((activity) => (
                                <TableRow key={activity.id}>
                                  <TableCell>{activity.name}</TableCell>
                                  <TableCell>{activity.type}</TableCell>
                                  <TableCell>{activity.category}</TableCell>
                                  <TableCell>
                                    {activity.supported_robots.join(', ')}
                                  </TableCell>
                                  <TableCell>
                                    {formatDate(activity.updated_at)}
                                  </TableCell>
                                </TableRow>
                              ))}
                          </TableBody>
                        </Table>
                      </TableContainer>
                    </CardContent>
                  </Card>
                </Grid>

                <Grid item xs={12} md={4}>
                  <Card>
                    <CardContent>
                      <RobotCategoryPieChart
                        data={robotOverview.categoryDistribution}
                        height={300}
                      />
                    </CardContent>
                  </Card>
                </Grid>
              </Grid>

              {/* Robot Usage Trends */}
              <Grid container spacing={3} sx={{ mt: 2 }}>
                <Grid item xs={12}>
                  <Card>
                    <CardContent>
                      <RobotTrendLineChart
                        data={[
                          { date: '2024-01', modules: 15, projects: 8 },
                          { date: '2024-02', modules: 22, projects: 12 },
                          { date: '2024-03', modules: 28, projects: 18 },
                          { date: '2024-04', modules: 35, projects: 25 },
                          { date: '2024-05', modules: 42, projects: 32 },
                          { date: '2024-06', modules: 48, projects: 38 },
                        ]}
                        title="Robot Usage Trends (Last 6 Months)"
                        height={300}
                      />
                    </CardContent>
                  </Card>
                </Grid>
              </Grid>
            </>
          ) : (
            <Box sx={{ display: 'flex', justifyContent: 'center', p: 4 }}>
              <CircularProgress />
            </Box>
          )}
        </TabPanel>

        {/* System Metrics Tab */}
        <TabPanel value={tabValue} index={6}>
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
                    <Box
                      sx={{
                        display: 'flex',
                        justifyContent: 'space-between',
                        mb: 1,
                      }}
                    >
                      <Typography variant="body2">CPU Usage</Typography>
                      <Typography variant="body2">
                        {systemMetrics?.cpuUsage}%
                      </Typography>
                    </Box>
                    <LinearProgress
                      variant="determinate"
                      value={systemMetrics?.cpuUsage || 0}
                      color={
                        systemMetrics && systemMetrics.cpuUsage > 80
                          ? 'error'
                          : 'primary'
                      }
                    />
                  </Box>

                  <Box sx={{ mb: 3 }}>
                    <Box
                      sx={{
                        display: 'flex',
                        justifyContent: 'space-between',
                        mb: 1,
                      }}
                    >
                      <Typography variant="body2">Memory Usage</Typography>
                      <Typography variant="body2">
                        {systemMetrics?.memoryUsage}%
                      </Typography>
                    </Box>
                    <LinearProgress
                      variant="determinate"
                      value={systemMetrics?.memoryUsage || 0}
                      color={
                        systemMetrics && systemMetrics.memoryUsage > 80
                          ? 'error'
                          : 'primary'
                      }
                    />
                  </Box>

                  <Box sx={{ mb: 3 }}>
                    <Box
                      sx={{
                        display: 'flex',
                        justifyContent: 'space-between',
                        mb: 1,
                      }}
                    >
                      <Typography variant="body2">Network Usage</Typography>
                      <Typography variant="body2">
                        {systemMetrics?.networkUsage}%
                      </Typography>
                    </Box>
                    <LinearProgress
                      variant="determinate"
                      value={systemMetrics?.networkUsage || 0}
                      color="primary"
                    />
                  </Box>

                  <Box>
                    <Box
                      sx={{
                        display: 'flex',
                        justifyContent: 'space-between',
                        mb: 1,
                      }}
                    >
                      <Typography variant="body2">Storage Usage</Typography>
                      <Typography variant="body2">
                        {systemMetrics
                          ? `${((systemMetrics.usedStorage / systemMetrics.totalStorage) * 100).toFixed(1)}%`
                          : '0%'}
                      </Typography>
                    </Box>
                    <LinearProgress
                      variant="determinate"
                      value={
                        systemMetrics
                          ? (systemMetrics.usedStorage /
                              systemMetrics.totalStorage) *
                            100
                          : 0
                      }
                      color={
                        systemMetrics &&
                        (systemMetrics.usedStorage /
                          systemMetrics.totalStorage) *
                          100 >
                          80
                          ? 'error'
                          : 'primary'
                      }
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
                        secondary={systemMetrics?.systemStatus}
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
                        secondary={
                          systemMetrics
                            ? formatBytes(systemMetrics.totalStorage)
                            : '0 Bytes'
                        }
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
                        secondary={`${systemMetrics?.activeUsers || 0} / ${systemMetrics?.totalUsers || 0}`}
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
                        secondary={`${systemMetrics?.activeProjects || 0} / ${systemMetrics?.totalProjects || 0}`}
                      />
                    </ListItem>
                  </List>
                </CardContent>
              </Card>
            </Grid>
          </Grid>
        </TabPanel>

        {/* Security & Access Tab */}
        <TabPanel value={tabValue} index={5}>
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

      {/* User Action Dialog */}
      <Dialog
        open={userActionDialog}
        onClose={closeUserActionDialog}
        maxWidth="sm"
        fullWidth
      >
        <DialogTitle>User Actions</DialogTitle>
        <DialogContent>
          {selectedUser && (
            <Box sx={{ pt: 2 }}>
              <Typography variant="body1" gutterBottom>
                Actions for user: <strong>{selectedUser.username}</strong>
              </Typography>
              <Typography variant="body2" color="textSecondary" gutterBottom>
                Email: {selectedUser.email}
              </Typography>
              <Typography variant="body2" color="textSecondary" gutterBottom>
                Current Role: {selectedUser.role}
              </Typography>
            </Box>
          )}
        </DialogContent>
        <DialogActions>
          <Button onClick={closeUserActionDialog} disabled={userActionLoading}>
            Cancel
          </Button>
          {selectedUser && selectedUser.role === 'user' && (
            <Button
              onClick={() => handleUserAction(selectedUser.id, 'promote')}
              disabled={userActionLoading}
              color="primary"
            >
              Promote to Admin
            </Button>
          )}
          {selectedUser && selectedUser.role === 'admin' && (
            <Button
              onClick={() => handleUserAction(selectedUser.id, 'demote')}
              disabled={userActionLoading}
              color="warning"
            >
              Demote to User
            </Button>
          )}
        </DialogActions>
      </Dialog>

      {/* Create User Dialog */}
      <Dialog
        open={createUserDialog}
        onClose={closeCreateUserDialog}
        maxWidth="sm"
        fullWidth
      >
        <DialogTitle>Create New User</DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 2 }}>
            <TextField
              fullWidth
              label="Username"
              value={createUserForm.username}
              onChange={(e) =>
                handleCreateUserFormChange('username', e.target.value)
              }
              margin="normal"
              required
            />
            <TextField
              fullWidth
              label="Email"
              type="email"
              value={createUserForm.email}
              onChange={(e) =>
                handleCreateUserFormChange('email', e.target.value)
              }
              margin="normal"
              required
            />
            <TextField
              fullWidth
              label="Password"
              type="password"
              value={createUserForm.password}
              onChange={(e) =>
                handleCreateUserFormChange('password', e.target.value)
              }
              margin="normal"
              required
            />
            <FormControl fullWidth margin="normal">
              <InputLabel>Role</InputLabel>
              <Select
                value={createUserForm.role}
                onChange={(e) =>
                  handleCreateUserFormChange('role', e.target.value)
                }
                label="Role"
              >
                <MenuItem value="user">User</MenuItem>
                <MenuItem value="admin">Admin</MenuItem>
              </Select>
            </FormControl>
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={closeCreateUserDialog} disabled={createUserLoading}>
            Cancel
          </Button>
          <Button
            onClick={handleCreateUser}
            disabled={
              createUserLoading ||
              !createUserForm.username ||
              !createUserForm.email ||
              !createUserForm.password
            }
            variant="contained"
          >
            {createUserLoading ? 'Creating...' : 'Create User'}
          </Button>
        </DialogActions>
      </Dialog>

      {/* Add Robot Dialog */}
      <Dialog
        open={addRobotDialogOpen}
        onClose={() => setAddRobotDialogOpen(false)}
        maxWidth="sm"
        fullWidth
      >
        <DialogTitle>Add New Robot</DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 2 }}>
            <TextField
              fullWidth
              label="Robot Code"
              value={newRobot.code}
              onChange={(e) =>
                setNewRobot({ ...newRobot, code: e.target.value })
              }
              margin="normal"
              required
              placeholder="e.g., turtlebot3"
            />
            <TextField
              fullWidth
              label="Robot Name"
              value={newRobot.name}
              onChange={(e) =>
                setNewRobot({ ...newRobot, name: e.target.value })
              }
              margin="normal"
              required
              placeholder="e.g., TurtleBot 3"
            />
            <TextField
              fullWidth
              label="Description"
              value={newRobot.description}
              onChange={(e) =>
                setNewRobot({ ...newRobot, description: e.target.value })
              }
              margin="normal"
              multiline
              rows={3}
              placeholder="Optional description of the robot"
            />
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setAddRobotDialogOpen(false)}>Cancel</Button>
          <Button
            onClick={handleAddRobot}
            disabled={!newRobot.code || !newRobot.name}
            variant="contained"
          >
            Add Robot
          </Button>
        </DialogActions>
      </Dialog>

      {/* Edit Robot Dialog */}
      <Dialog
        open={editRobotDialogOpen}
        onClose={() => setEditRobotDialogOpen(false)}
        maxWidth="sm"
        fullWidth
      >
        <DialogTitle>Edit Robot</DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 2 }}>
            <TextField
              fullWidth
              label="Robot Name"
              value={selectedRobot?.name || ''}
              onChange={(e) =>
                setSelectedRobot({ ...selectedRobot, name: e.target.value })
              }
              margin="normal"
              required
            />
            <TextField
              fullWidth
              label="Description"
              value={selectedRobot?.description || ''}
              onChange={(e) =>
                setSelectedRobot({
                  ...selectedRobot,
                  description: e.target.value,
                })
              }
              margin="normal"
              multiline
              rows={3}
            />
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setEditRobotDialogOpen(false)}>Cancel</Button>
          <Button
            onClick={handleEditRobot}
            disabled={!selectedRobot?.name}
            variant="contained"
          >
            Update Robot
          </Button>
        </DialogActions>
      </Dialog>

      {/* Delete Robot Confirmation Dialog */}
      <Dialog
        open={deleteRobotDialogOpen}
        onClose={() => setDeleteRobotDialogOpen(false)}
        maxWidth="sm"
        fullWidth
      >
        <DialogTitle>Delete Robot</DialogTitle>
        <DialogContent>
          <Typography>
            Are you sure you want to delete the robot "
            {selectedRobot?.name || selectedRobot?.robot_type || 'Unknown'}"?
          </Typography>
          <Typography variant="body2" color="textSecondary" sx={{ mt: 1 }}>
            This action will remove the robot from all modules and projects that
            support it.
          </Typography>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setDeleteRobotDialogOpen(false)}>
            Cancel
          </Button>
          <Button onClick={handleDeleteRobot} color="error" variant="contained">
            Delete Robot
          </Button>
        </DialogActions>
      </Dialog>

      {/* Snackbar for notifications */}
      <Snackbar
        open={snackbar.open}
        autoHideDuration={6000}
        onClose={() => setSnackbar({ ...snackbar, open: false })}
      >
        <Alert
          onClose={() => setSnackbar({ ...snackbar, open: false })}
          severity={snackbar.severity}
          sx={{ width: '100%' }}
        >
          {snackbar.message}
        </Alert>
      </Snackbar>
    </Box>
  );
};

export default AdminDashboard;
