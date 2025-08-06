import React, { useState, useEffect } from 'react';
import {
  Box,
  Typography,
  Card,
  CardContent,
  Grid,
  Tabs,
  Tab,
  Button,
  TextField,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Chip,
  Avatar,
  List,
  ListItem,
  ListItemText,
  ListItemAvatar,
  ListItemSecondaryAction,
  IconButton,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  Switch,
  FormControlLabel,
  Alert,
  CircularProgress,
  Tooltip,
  Divider,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Paper,
  Checkbox,
  Radio,
  RadioGroup,
  FormGroup,
  InputAdornment,
  Badge,
} from '@mui/material';
import {
  Share as ShareIcon,
  Download as DownloadIcon,
  Upload as UploadIcon,
  Link as LinkIcon,
  Email as EmailIcon,
  Group as GroupIcon,
  Person as PersonIcon,
  Public as PublicIcon,
  Lock as LockIcon,
  Visibility as VisibilityIcon,
  VisibilityOff as VisibilityOffIcon,
  Edit as EditIcon,
  Delete as DeleteIcon,
  Add as AddIcon,
  Remove as RemoveIcon,
  ExpandMore as ExpandMoreIcon,
  CheckCircle as CheckCircleIcon,
  Warning as WarningIcon,
  Info as InfoIcon,
  Settings as SettingsIcon,
  Copy as CopyIcon,
  QrCode as QrCodeIcon,
  History as HistoryIcon,
  Security as SecurityIcon,
  CloudUpload as CloudUploadIcon,
  CloudDownload as CloudDownloadIcon,
  Archive as ArchiveIcon,
  RestoreFromTrash as RestoreIcon,
  GetApp as GetAppIcon,
  Send as SendIcon,
  Block as BlockIcon,
  Unblock as UnblockIcon,
} from '@mui/icons-material';

// Types
interface ProjectShareSettings {
  isPublic: boolean;
  allowComments: boolean;
  allowForking: boolean;
  requireApproval: boolean;
  expirationDate?: string;
  maxCollaborators: number;
  allowedDomains: string[];
  passwordProtected: boolean;
  password?: string;
}

interface Collaborator {
  id: string;
  email: string;
  name: string;
  role: 'owner' | 'admin' | 'editor' | 'viewer';
  status: 'active' | 'pending' | 'blocked';
  joinedAt: string;
  lastActive: string;
  permissions: string[];
}

interface ShareLink {
  id: string;
  url: string;
  type: 'public' | 'private' | 'temporary';
  expiresAt?: string;
  accessCount: number;
  createdAt: string;
  createdBy: string;
  isActive: boolean;
}

interface ExportFormat {
  id: string;
  name: string;
  description: string;
  extension: string;
  icon: React.ReactNode;
  supported: boolean;
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
      id={`sharing-tabpanel-${index}`}
      aria-labelledby={`sharing-tab-${index}`}
      {...other}
    >
      {value === index && <Box sx={{ p: 3 }}>{children}</Box>}
    </div>
  );
}

const ProjectSharing: React.FC = () => {
  const [tabValue, setTabValue] = useState(0);
  const [loading, setLoading] = useState(false);
  const [shareSettings, setShareSettings] = useState<ProjectShareSettings>({
    isPublic: false,
    allowComments: true,
    allowForking: false,
    requireApproval: true,
    maxCollaborators: 10,
    allowedDomains: [],
    passwordProtected: false,
  });
  const [collaborators, setCollaborators] = useState<Collaborator[]>([]);
  const [shareLinks, setShareLinks] = useState<ShareLink[]>([]);
  const [showInviteDialog, setShowInviteDialog] = useState(false);
  const [showExportDialog, setShowExportDialog] = useState(false);
  const [showLinkDialog, setShowLinkDialog] = useState(false);
  const [selectedCollaborator, setSelectedCollaborator] = useState<Collaborator | null>(null);
  const [inviteEmail, setInviteEmail] = useState('');
  const [inviteRole, setInviteRole] = useState<'editor' | 'viewer'>('viewer');
  const [inviteMessage, setInviteMessage] = useState('');
  const [selectedExportFormats, setSelectedExportFormats] = useState<string[]>([]);
  const [exportProgress, setExportProgress] = useState(0);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);

  const exportFormats: ExportFormat[] = [
    {
      id: 'json',
      name: 'JSON',
      description: 'Complete project data in JSON format',
      extension: '.json',
      icon: <SettingsIcon />,
      supported: true,
    },
    {
      id: 'yaml',
      name: 'YAML',
      description: 'Project configuration in YAML format',
      extension: '.yaml',
      icon: <SettingsIcon />,
      supported: true,
    },
    {
      id: 'zip',
      name: 'ZIP Archive',
      description: 'Complete project with all files',
      extension: '.zip',
      icon: <ArchiveIcon />,
      supported: true,
    },
    {
      id: 'docker',
      name: 'Docker Image',
      description: 'Docker image with project environment',
      extension: '.tar',
      icon: <CloudUploadIcon />,
      supported: true,
    },
    {
      id: 'pdf',
      name: 'PDF Documentation',
      description: 'Project documentation and diagrams',
      extension: '.pdf',
      icon: <GetAppIcon />,
      supported: true,
    },
    {
      id: 'ros',
      name: 'ROS Package',
      description: 'ROS2 package format',
      extension: '.tar.gz',
      icon: <CloudDownloadIcon />,
      supported: true,
    },
  ];

  useEffect(() => {
    loadSharingData();
  }, []);

  const loadSharingData = async () => {
    setLoading(true);
    try {
      // Mock data - in real app this would come from API
      const mockCollaborators: Collaborator[] = [
        {
          id: '1',
          email: 'alice@robium.com',
          name: 'Alice Johnson',
          role: 'admin',
          status: 'active',
          joinedAt: '2024-01-01T00:00:00Z',
          lastActive: '2024-01-15T10:30:00Z',
          permissions: ['read', 'write', 'admin'],
        },
        {
          id: '2',
          email: 'bob@robium.com',
          name: 'Bob Smith',
          role: 'editor',
          status: 'active',
          joinedAt: '2024-01-05T00:00:00Z',
          lastActive: '2024-01-14T15:45:00Z',
          permissions: ['read', 'write'],
        },
        {
          id: '3',
          email: 'charlie@robium.com',
          name: 'Charlie Brown',
          role: 'viewer',
          status: 'pending',
          joinedAt: '2024-01-10T00:00:00Z',
          lastActive: '2024-01-10T00:00:00Z',
          permissions: ['read'],
        },
      ];

      const mockShareLinks: ShareLink[] = [
        {
          id: '1',
          url: 'https://robium.com/share/project-123',
          type: 'public',
          accessCount: 45,
          createdAt: '2024-01-01T00:00:00Z',
          createdBy: 'admin@robium.com',
          isActive: true,
        },
        {
          id: '2',
          url: 'https://robium.com/share/project-123-private',
          type: 'private',
          expiresAt: '2024-02-01T00:00:00Z',
          accessCount: 12,
          createdAt: '2024-01-05T00:00:00Z',
          createdBy: 'admin@robium.com',
          isActive: true,
        },
      ];

      setCollaborators(mockCollaborators);
      setShareLinks(mockShareLinks);
      setLoading(false);
    } catch (err) {
      setError('Failed to load sharing data');
      setLoading(false);
    }
  };

  const handleInviteCollaborator = async () => {
    try {
      // TODO: Implement API call
      console.log('Inviting collaborator:', { inviteEmail, inviteRole, inviteMessage });
      
      const newCollaborator: Collaborator = {
        id: Date.now().toString(),
        email: inviteEmail,
        name: inviteEmail.split('@')[0],
        role: inviteRole,
        status: 'pending',
        joinedAt: new Date().toISOString(),
        lastActive: new Date().toISOString(),
        permissions: inviteRole === 'editor' ? ['read', 'write'] : ['read'],
      };

      setCollaborators([...collaborators, newCollaborator]);
      setShowInviteDialog(false);
      setInviteEmail('');
      setInviteRole('viewer');
      setInviteMessage('');
      setSuccess('Invitation sent successfully');
    } catch (err) {
      setError('Failed to send invitation');
    }
  };

  const handleExportProject = async () => {
    try {
      setExportProgress(0);
      setShowExportDialog(false);
      
      // Simulate export progress
      const interval = setInterval(() => {
        setExportProgress((prev) => {
          if (prev >= 100) {
            clearInterval(interval);
            setSuccess('Project exported successfully');
            return 100;
          }
          return prev + 10;
        });
      }, 200);

      // TODO: Implement actual export logic
      console.log('Exporting project in formats:', selectedExportFormats);
    } catch (err) {
      setError('Failed to export project');
    }
  };

  const handleCreateShareLink = async () => {
    try {
      const newLink: ShareLink = {
        id: Date.now().toString(),
        url: `https://robium.com/share/project-${Date.now()}`,
        type: 'public',
        accessCount: 0,
        createdAt: new Date().toISOString(),
        createdBy: 'admin@robium.com',
        isActive: true,
      };

      setShareLinks([...shareLinks, newLink]);
      setShowLinkDialog(false);
      setSuccess('Share link created successfully');
    } catch (err) {
      setError('Failed to create share link');
    }
  };

  const handleCollaboratorAction = async (collaboratorId: string, action: 'remove' | 'block' | 'promote' | 'demote') => {
    try {
      // TODO: Implement API calls
      console.log(`Collaborator action: ${action} for collaborator ${collaboratorId}`);
      
      setCollaborators(collaborators.map(collaborator => {
        if (collaborator.id === collaboratorId) {
          switch (action) {
            case 'remove':
              return { ...collaborator, status: 'blocked' as const };
            case 'block':
              return { ...collaborator, status: 'blocked' as const };
            case 'promote':
              return { ...collaborator, role: collaborator.role === 'viewer' ? 'editor' : 'admin' as any };
            case 'demote':
              return { ...collaborator, role: collaborator.role === 'admin' ? 'editor' : 'viewer' as any };
            default:
              return collaborator;
          }
        }
        return collaborator;
      }));
    } catch (err) {
      setError(`Failed to ${action} collaborator`);
    }
  };

  const handleShareLinkAction = async (linkId: string, action: 'deactivate' | 'delete') => {
    try {
      // TODO: Implement API calls
      console.log(`Share link action: ${action} for link ${linkId}`);
      
      if (action === 'delete') {
        setShareLinks(shareLinks.filter(link => link.id !== linkId));
      } else {
        setShareLinks(shareLinks.map(link => 
          link.id === linkId ? { ...link, isActive: false } : link
        ));
      }
    } catch (err) {
      setError(`Failed to ${action} share link`);
    }
  };

  const copyToClipboard = (text: string) => {
    navigator.clipboard.writeText(text);
    setSuccess('Copied to clipboard');
  };

  const getRoleIcon = (role: string) => {
    switch (role) {
      case 'owner':
        return <PersonIcon color="error" />;
      case 'admin':
        return <SecurityIcon color="warning" />;
      case 'editor':
        return <EditIcon color="primary" />;
      case 'viewer':
        return <VisibilityIcon color="action" />;
      default:
        return <PersonIcon />;
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'active':
        return 'success';
      case 'pending':
        return 'warning';
      case 'blocked':
        return 'error';
      default:
        return 'default';
    }
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
          Project Sharing & Export
        </Typography>
        <Box sx={{ display: 'flex', gap: 2 }}>
          <Button
            variant="outlined"
            startIcon={<ShareIcon />}
            onClick={() => setShowInviteDialog(true)}
          >
            Invite Collaborator
          </Button>
          <Button
            variant="contained"
            startIcon={<DownloadIcon />}
            onClick={() => setShowExportDialog(true)}
          >
            Export Project
          </Button>
        </Box>
      </Box>

      {/* Alerts */}
      {error && (
        <Alert severity="error" sx={{ mb: 3 }} onClose={() => setError(null)}>
          {error}
        </Alert>
      )}
      {success && (
        <Alert severity="success" sx={{ mb: 3 }} onClose={() => setSuccess(null)}>
          {success}
        </Alert>
      )}

      {/* Export Progress */}
      {exportProgress > 0 && exportProgress < 100 && (
        <Card sx={{ mb: 3 }}>
          <CardContent>
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
              <CircularProgress variant="determinate" value={exportProgress} size={24} />
              <Typography>
                Exporting project... {exportProgress}%
              </Typography>
            </Box>
          </CardContent>
        </Card>
      )}

      {/* Main Tabs */}
      <Card>
        <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
          <Tabs value={tabValue} onChange={(_, newValue) => setTabValue(newValue)}>
            <Tab icon={<GroupIcon />} label="Collaborators" />
            <Tab icon={<LinkIcon />} label="Share Links" />
            <Tab icon={<SettingsIcon />} label="Sharing Settings" />
            <Tab icon={<HistoryIcon />} label="Activity" />
          </Tabs>
        </Box>

        {/* Collaborators Tab */}
        <TabPanel value={tabValue} index={0}>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
            <Typography variant="h6">
              Project Collaborators ({collaborators.length})
            </Typography>
            <Button
              variant="contained"
              startIcon={<AddIcon />}
              onClick={() => setShowInviteDialog(true)}
            >
              Invite Collaborator
            </Button>
          </Box>

          <TableContainer component={Paper}>
            <Table>
              <TableHead>
                <TableRow>
                  <TableCell>Collaborator</TableCell>
                  <TableCell>Role</TableCell>
                  <TableCell>Status</TableCell>
                  <TableCell>Joined</TableCell>
                  <TableCell>Last Active</TableCell>
                  <TableCell>Actions</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {collaborators.map((collaborator) => (
                  <TableRow key={collaborator.id}>
                    <TableCell>
                      <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
                        <Avatar>{collaborator.name.charAt(0).toUpperCase()}</Avatar>
                        <Box>
                          <Typography variant="subtitle2">{collaborator.name}</Typography>
                          <Typography variant="body2" color="textSecondary">
                            {collaborator.email}
                          </Typography>
                        </Box>
                      </Box>
                    </TableCell>
                    <TableCell>
                      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                        {getRoleIcon(collaborator.role)}
                        <Chip label={collaborator.role} size="small" />
                      </Box>
                    </TableCell>
                    <TableCell>
                      <Chip
                        label={collaborator.status}
                        color={getStatusColor(collaborator.status) as any}
                        size="small"
                      />
                    </TableCell>
                    <TableCell>{formatDate(collaborator.joinedAt)}</TableCell>
                    <TableCell>{formatDate(collaborator.lastActive)}</TableCell>
                    <TableCell>
                      <Box sx={{ display: 'flex', gap: 1 }}>
                        <Tooltip title="Edit Permissions">
                          <IconButton size="small">
                            <EditIcon />
                          </IconButton>
                        </Tooltip>
                        {collaborator.role !== 'owner' && (
                          <>
                            {collaborator.role === 'viewer' && (
                              <Tooltip title="Promote to Editor">
                                <IconButton
                                  size="small"
                                  color="primary"
                                  onClick={() => handleCollaboratorAction(collaborator.id, 'promote')}
                                >
                                  <AddIcon />
                                </IconButton>
                              </Tooltip>
                            )}
                            {collaborator.role === 'editor' && (
                              <Tooltip title="Promote to Admin">
                                <IconButton
                                  size="small"
                                  color="primary"
                                  onClick={() => handleCollaboratorAction(collaborator.id, 'promote')}
                                >
                                  <SecurityIcon />
                                </IconButton>
                              </Tooltip>
                            )}
                            {collaborator.role === 'admin' && (
                              <Tooltip title="Demote to Editor">
                                <IconButton
                                  size="small"
                                  color="warning"
                                  onClick={() => handleCollaboratorAction(collaborator.id, 'demote')}
                                >
                                  <RemoveIcon />
                                </IconButton>
                              </Tooltip>
                            )}
                            {collaborator.status === 'active' ? (
                              <Tooltip title="Block Collaborator">
                                <IconButton
                                  size="small"
                                  color="warning"
                                  onClick={() => handleCollaboratorAction(collaborator.id, 'block')}
                                >
                                  <BlockIcon />
                                </IconButton>
                              </Tooltip>
                            ) : (
                              <Tooltip title="Unblock Collaborator">
                                <IconButton
                                  size="small"
                                  color="success"
                                  onClick={() => handleCollaboratorAction(collaborator.id, 'promote')}
                                >
                                  <UnblockIcon />
                                </IconButton>
                              </Tooltip>
                            )}
                            <Tooltip title="Remove Collaborator">
                              <IconButton
                                size="small"
                                color="error"
                                onClick={() => handleCollaboratorAction(collaborator.id, 'remove')}
                              >
                                <DeleteIcon />
                              </IconButton>
                            </Tooltip>
                          </>
                        )}
                      </Box>
                    </TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          </TableContainer>
        </TabPanel>

        {/* Share Links Tab */}
        <TabPanel value={tabValue} index={1}>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
            <Typography variant="h6">
              Share Links ({shareLinks.length})
            </Typography>
            <Button
              variant="contained"
              startIcon={<AddIcon />}
              onClick={() => setShowLinkDialog(true)}
            >
              Create Share Link
            </Button>
          </Box>

          <Grid container spacing={3}>
            {shareLinks.map((link) => (
              <Grid item xs={12} md={6} key={link.id}>
                <Card>
                  <CardContent>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', mb: 2 }}>
                      <Box>
                        <Typography variant="h6" gutterBottom>
                          {link.type === 'public' ? 'Public Link' : 'Private Link'}
                        </Typography>
                        <Typography variant="body2" color="textSecondary" sx={{ mb: 1 }}>
                          Created {formatDate(link.createdAt)}
                        </Typography>
                        {link.expiresAt && (
                          <Typography variant="body2" color="textSecondary">
                            Expires {formatDate(link.expiresAt)}
                          </Typography>
                        )}
                      </Box>
                      <Chip
                        label={link.isActive ? 'Active' : 'Inactive'}
                        color={link.isActive ? 'success' : 'default'}
                        size="small"
                      />
                    </Box>

                    <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 2 }}>
                      <TextField
                        fullWidth
                        value={link.url}
                        variant="outlined"
                        size="small"
                        InputProps={{
                          readOnly: true,
                          endAdornment: (
                            <InputAdornment position="end">
                              <Tooltip title="Copy Link">
                                <IconButton
                                  size="small"
                                  onClick={() => copyToClipboard(link.url)}
                                >
                                  <CopyIcon />
                                </IconButton>
                              </Tooltip>
                            </InputAdornment>
                          ),
                        }}
                      />
                    </Box>

                    <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                      <Typography variant="body2" color="textSecondary">
                        {link.accessCount} accesses
                      </Typography>
                      <Box sx={{ display: 'flex', gap: 1 }}>
                        <Tooltip title="QR Code">
                          <IconButton size="small">
                            <QrCodeIcon />
                          </IconButton>
                        </Tooltip>
                        {link.isActive ? (
                          <Tooltip title="Deactivate Link">
                            <IconButton
                              size="small"
                              color="warning"
                              onClick={() => handleShareLinkAction(link.id, 'deactivate')}
                            >
                              <VisibilityOffIcon />
                            </IconButton>
                          </Tooltip>
                        ) : (
                          <Tooltip title="Activate Link">
                            <IconButton
                              size="small"
                              color="success"
                              onClick={() => handleShareLinkAction(link.id, 'deactivate')}
                            >
                              <VisibilityIcon />
                            </IconButton>
                          </Tooltip>
                        )}
                        <Tooltip title="Delete Link">
                          <IconButton
                            size="small"
                            color="error"
                            onClick={() => handleShareLinkAction(link.id, 'delete')}
                          >
                            <DeleteIcon />
                          </IconButton>
                        </Tooltip>
                      </Box>
                    </Box>
                  </CardContent>
                </Card>
              </Grid>
            ))}
          </Grid>
        </TabPanel>

        {/* Sharing Settings Tab */}
        <TabPanel value={tabValue} index={2}>
          <Typography variant="h6" gutterBottom>
            Project Sharing Settings
          </Typography>

          <Grid container spacing={3}>
            <Grid item xs={12} md={6}>
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    General Settings
                  </Typography>
                  
                  <FormControlLabel
                    control={
                      <Switch
                        checked={shareSettings.isPublic}
                        onChange={(e) => setShareSettings({ ...shareSettings, isPublic: e.target.checked })}
                      />
                    }
                    label="Make project public"
                  />
                  
                  <FormControlLabel
                    control={
                      <Switch
                        checked={shareSettings.allowComments}
                        onChange={(e) => setShareSettings({ ...shareSettings, allowComments: e.target.checked })}
                      />
                    }
                    label="Allow comments"
                  />
                  
                  <FormControlLabel
                    control={
                      <Switch
                        checked={shareSettings.allowForking}
                        onChange={(e) => setShareSettings({ ...shareSettings, allowForking: e.target.checked })}
                      />
                    }
                    label="Allow forking"
                  />
                  
                  <FormControlLabel
                    control={
                      <Switch
                        checked={shareSettings.requireApproval}
                        onChange={(e) => setShareSettings({ ...shareSettings, requireApproval: e.target.checked })}
                      />
                    }
                    label="Require approval for new collaborators"
                  />
                  
                  <TextField
                    fullWidth
                    label="Maximum collaborators"
                    type="number"
                    value={shareSettings.maxCollaborators}
                    onChange={(e) => setShareSettings({ ...shareSettings, maxCollaborators: parseInt(e.target.value) })}
                    margin="normal"
                  />
                </CardContent>
              </Card>
            </Grid>

            <Grid item xs={12} md={6}>
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    Security Settings
                  </Typography>
                  
                  <FormControlLabel
                    control={
                      <Switch
                        checked={shareSettings.passwordProtected}
                        onChange={(e) => setShareSettings({ ...shareSettings, passwordProtected: e.target.checked })}
                      />
                    }
                    label="Password protect project"
                  />
                  
                  {shareSettings.passwordProtected && (
                    <TextField
                      fullWidth
                      label="Project Password"
                      type="password"
                      value={shareSettings.password || ''}
                      onChange={(e) => setShareSettings({ ...shareSettings, password: e.target.value })}
                      margin="normal"
                    />
                  )}
                  
                  <TextField
                    fullWidth
                    label="Allowed domains (comma-separated)"
                    value={shareSettings.allowedDomains.join(', ')}
                    onChange={(e) => setShareSettings({ 
                      ...shareSettings, 
                      allowedDomains: e.target.value.split(',').map(domain => domain.trim()) 
                    })}
                    margin="normal"
                    helperText="Only users from these domains can access the project"
                  />
                </CardContent>
              </Card>
            </Grid>
          </Grid>
        </TabPanel>

        {/* Activity Tab */}
        <TabPanel value={tabValue} index={3}>
          <Typography variant="h6" gutterBottom>
            Recent Activity
          </Typography>
          
          <List>
            <ListItem>
              <ListItemAvatar>
                <Avatar sx={{ bgcolor: 'success.main' }}>
                  <AddIcon />
                </Avatar>
              </ListItemAvatar>
              <ListItemText
                primary="New collaborator invited"
                secondary="Alice Johnson was invited to the project"
              />
              <ListItemSecondaryAction>
                <Typography variant="body2" color="textSecondary">
                  2 hours ago
                </Typography>
              </ListItemSecondaryAction>
            </ListItem>
            
            <ListItem>
              <ListItemAvatar>
                <Avatar sx={{ bgcolor: 'info.main' }}>
                  <LinkIcon />
                </Avatar>
              </ListItemAvatar>
              <ListItemText
                primary="Share link created"
                secondary="Public share link was created"
              />
              <ListItemSecondaryAction>
                <Typography variant="body2" color="textSecondary">
                  1 day ago
                </Typography>
              </ListItemSecondaryAction>
            </ListItem>
            
            <ListItem>
              <ListItemAvatar>
                <Avatar sx={{ bgcolor: 'warning.main' }}>
                  <EditIcon />
                </Avatar>
              </ListItemAvatar>
              <ListItemText
                primary="Project settings updated"
                secondary="Sharing settings were modified"
              />
              <ListItemSecondaryAction>
                <Typography variant="body2" color="textSecondary">
                  3 days ago
                </Typography>
              </ListItemSecondaryAction>
            </ListItem>
          </List>
        </TabPanel>
      </Card>

      {/* Invite Collaborator Dialog */}
      <Dialog open={showInviteDialog} onClose={() => setShowInviteDialog(false)} maxWidth="sm" fullWidth>
        <DialogTitle>Invite Collaborator</DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 2 }}>
            <TextField
              fullWidth
              label="Email Address"
              type="email"
              value={inviteEmail}
              onChange={(e) => setInviteEmail(e.target.value)}
              margin="normal"
            />
            
            <FormControl fullWidth margin="normal">
              <InputLabel>Role</InputLabel>
              <Select value={inviteRole} label="Role" onChange={(e) => setInviteRole(e.target.value as any)}>
                <MenuItem value="viewer">Viewer (Read-only)</MenuItem>
                <MenuItem value="editor">Editor (Read & Write)</MenuItem>
              </Select>
            </FormControl>
            
            <TextField
              fullWidth
              label="Personal Message (Optional)"
              multiline
              rows={3}
              value={inviteMessage}
              onChange={(e) => setInviteMessage(e.target.value)}
              margin="normal"
              placeholder="Add a personal message to your invitation..."
            />
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowInviteDialog(false)}>Cancel</Button>
          <Button variant="contained" onClick={handleInviteCollaborator}>
            Send Invitation
          </Button>
        </DialogActions>
      </Dialog>

      {/* Export Project Dialog */}
      <Dialog open={showExportDialog} onClose={() => setShowExportDialog(false)} maxWidth="md" fullWidth>
        <DialogTitle>Export Project</DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 2 }}>
            <Typography variant="h6" gutterBottom>
              Select Export Formats
            </Typography>
            
            <Grid container spacing={2}>
              {exportFormats.map((format) => (
                <Grid item xs={12} sm={6} key={format.id}>
                  <Card variant="outlined">
                    <CardContent>
                      <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
                        <Checkbox
                          checked={selectedExportFormats.includes(format.id)}
                          onChange={(e) => {
                            if (e.target.checked) {
                              setSelectedExportFormats([...selectedExportFormats, format.id]);
                            } else {
                              setSelectedExportFormats(selectedExportFormats.filter(id => id !== format.id));
                            }
                          }}
                        />
                        <Box>
                          <Typography variant="subtitle1">
                            {format.name}
                          </Typography>
                          <Typography variant="body2" color="textSecondary">
                            {format.description}
                          </Typography>
                        </Box>
                      </Box>
                    </CardContent>
                  </Card>
                </Grid>
              ))}
            </Grid>
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowExportDialog(false)}>Cancel</Button>
          <Button 
            variant="contained" 
            onClick={handleExportProject}
            disabled={selectedExportFormats.length === 0}
          >
            Export Project
          </Button>
        </DialogActions>
      </Dialog>

      {/* Create Share Link Dialog */}
      <Dialog open={showLinkDialog} onClose={() => setShowLinkDialog(false)} maxWidth="sm" fullWidth>
        <DialogTitle>Create Share Link</DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 2 }}>
            <FormControl component="fieldset" margin="normal">
              <Typography variant="subtitle1" gutterBottom>
                Link Type
              </Typography>
              <RadioGroup
                value="public"
                onChange={() => {}}
              >
                <FormControlLabel value="public" control={<Radio />} label="Public Link (Anyone can access)" />
                <FormControlLabel value="private" control={<Radio />} label="Private Link (Password protected)" />
                <FormControlLabel value="temporary" control={<Radio />} label="Temporary Link (Expires automatically)" />
              </RadioGroup>
            </FormControl>
            
            <TextField
              fullWidth
              label="Expiration Date (Optional)"
              type="date"
              margin="normal"
              InputLabelProps={{ shrink: true }}
            />
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowLinkDialog(false)}>Cancel</Button>
          <Button variant="contained" onClick={handleCreateShareLink}>
            Create Link
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default ProjectSharing; 