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
  Chip,
  Avatar,
  List,
  ListItem,
  ListItemText,
  ListItemAvatar,
  IconButton,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  Alert,
  CircularProgress,
  Tooltip,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Paper,
  Timeline,
  TimelineItem,
  TimelineSeparator,
  TimelineConnector,
  TimelineContent,
  TimelineDot,
  TimelineOppositeContent,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Switch,
  FormControlLabel,
  Divider,
  Accordion,
  AccordionSummary,
  AccordionDetails,
} from '@mui/material';
import {
  History as HistoryIcon,
  Branch as BranchIcon,
  Merge as MergeIcon,
  Tag as TagIcon,
  Commit as CommitIcon,
  Compare as CompareIcon,
  Restore as RestoreIcon,
  Download as DownloadIcon,
  Upload as UploadIcon,
  Add as AddIcon,
  Edit as EditIcon,
  Delete as DeleteIcon,
  ExpandMore as ExpandMoreIcon,
  CheckCircle as CheckCircleIcon,
  Warning as WarningIcon,
  Error as ErrorIcon,
  Info as InfoIcon,
  Settings as SettingsIcon,
  CloudUpload as CloudUploadIcon,
  CloudDownload as CloudDownloadIcon,
  Archive as ArchiveIcon,
  RestoreFromTrash as RestoreIcon,
  GetApp as GetAppIcon,
  Send as SendIcon,
  Block as BlockIcon,
  Unblock as UnblockIcon,
  Timeline as TimelineIcon,
  Code as CodeIcon,
  BugReport as BugReportIcon,
  NewReleases as NewReleasesIcon,
  Security as SecurityIcon,
} from '@mui/icons-material';

// Types
interface Version {
  id: string;
  version: string;
  name: string;
  description: string;
  type: 'major' | 'minor' | 'patch' | 'preview' | 'beta' | 'alpha';
  status: 'draft' | 'published' | 'archived' | 'deprecated';
  createdAt: string;
  createdBy: string;
  commitHash: string;
  branch: string;
  changes: Change[];
  dependencies: Dependency[];
  releaseNotes: string;
  downloadUrl?: string;
}

interface Change {
  id: string;
  type: 'feature' | 'bugfix' | 'breaking' | 'security' | 'performance' | 'documentation';
  title: string;
  description: string;
  files: string[];
  author: string;
  timestamp: string;
}

interface Dependency {
  name: string;
  version: string;
  type: 'production' | 'development' | 'peer';
  status: 'up-to-date' | 'outdated' | 'vulnerable';
}

interface Branch {
  id: string;
  name: string;
  type: 'main' | 'develop' | 'feature' | 'hotfix' | 'release';
  status: 'active' | 'merged' | 'archived';
  lastCommit: string;
  lastCommitDate: string;
  aheadCount: number;
  behindCount: number;
  protected: boolean;
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
      id={`versioning-tabpanel-${index}`}
      aria-labelledby={`versioning-tab-${index}`}
      {...other}
    >
      {value === index && <Box sx={{ p: 3 }}>{children}</Box>}
    </div>
  );
}

const ProjectVersioning: React.FC = () => {
  const [tabValue, setTabValue] = useState(0);
  const [loading, setLoading] = useState(false);
  const [versions, setVersions] = useState<Version[]>([]);
  const [branches, setBranches] = useState<Branch[]>([]);
  const [selectedVersion, setSelectedVersion] = useState<Version | null>(null);
  const [showCreateVersionDialog, setShowCreateVersionDialog] = useState(false);
  const [showCompareDialog, setShowCompareDialog] = useState(false);
  const [showBranchDialog, setShowBranchDialog] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);

  useEffect(() => {
    loadVersioningData();
  }, []);

  const loadVersioningData = async () => {
    setLoading(true);
    try {
      // Mock data
      const mockVersions: Version[] = [
        {
          id: '1',
          version: '1.2.0',
          name: 'Feature Release',
          description: 'Major feature release with new navigation system',
          type: 'minor',
          status: 'published',
          createdAt: '2024-01-15T10:30:00Z',
          createdBy: 'admin@robium.com',
          commitHash: 'abc123def456',
          branch: 'main',
          changes: [
            {
              id: '1',
              type: 'feature',
              title: 'Add new navigation system',
              description: 'Implemented advanced navigation with pathfinding',
              files: ['src/navigation/', 'docs/navigation.md'],
              author: 'alice@robium.com',
              timestamp: '2024-01-15T10:30:00Z',
            },
          ],
          dependencies: [
            { name: 'react', version: '18.2.0', type: 'production', status: 'up-to-date' },
            { name: 'typescript', version: '5.0.0', type: 'development', status: 'up-to-date' },
          ],
          releaseNotes: 'This release introduces a new navigation system...',
          downloadUrl: 'https://robium.com/downloads/v1.2.0',
        },
        {
          id: '2',
          version: '1.1.5',
          name: 'Bug Fix Release',
          description: 'Critical bug fixes and performance improvements',
          type: 'patch',
          status: 'published',
          createdAt: '2024-01-10T14:20:00Z',
          createdBy: 'bob@robium.com',
          commitHash: 'def456ghi789',
          branch: 'main',
          changes: [
            {
              id: '2',
              type: 'bugfix',
              title: 'Fix memory leak in sensor processing',
              description: 'Resolved memory leak in sensor data processing',
              files: ['src/sensors/', 'tests/sensors.test.ts'],
              author: 'bob@robium.com',
              timestamp: '2024-01-10T14:20:00Z',
            },
          ],
          dependencies: [
            { name: 'react', version: '18.2.0', type: 'production', status: 'up-to-date' },
            { name: 'typescript', version: '5.0.0', type: 'development', status: 'up-to-date' },
          ],
          releaseNotes: 'This patch release fixes critical bugs...',
          downloadUrl: 'https://robium.com/downloads/v1.1.5',
        },
      ];

      const mockBranches: Branch[] = [
        {
          id: '1',
          name: 'main',
          type: 'main',
          status: 'active',
          lastCommit: 'abc123def456',
          lastCommitDate: '2024-01-15T10:30:00Z',
          aheadCount: 0,
          behindCount: 0,
          protected: true,
        },
        {
          id: '2',
          name: 'develop',
          type: 'develop',
          status: 'active',
          lastCommit: 'xyz789abc123',
          lastCommitDate: '2024-01-16T09:15:00Z',
          aheadCount: 3,
          behindCount: 0,
          protected: true,
        },
        {
          id: '3',
          name: 'feature/navigation-v2',
          type: 'feature',
          status: 'active',
          lastCommit: 'def456ghi789',
          lastCommitDate: '2024-01-16T11:45:00Z',
          aheadCount: 5,
          behindCount: 2,
          protected: false,
        },
      ];

      setVersions(mockVersions);
      setBranches(mockBranches);
      setLoading(false);
    } catch (err) {
      setError('Failed to load versioning data');
      setLoading(false);
    }
  };

  const handleCreateVersion = async () => {
    try {
      // TODO: Implement API call
      console.log('Creating new version');
      setShowCreateVersionDialog(false);
      setSuccess('Version created successfully');
    } catch (err) {
      setError('Failed to create version');
    }
  };

  const handleCreateBranch = async () => {
    try {
      // TODO: Implement API call
      console.log('Creating new branch');
      setShowBranchDialog(false);
      setSuccess('Branch created successfully');
    } catch (err) {
      setError('Failed to create branch');
    }
  };

  const getVersionTypeColor = (type: string) => {
    switch (type) {
      case 'major':
        return 'error';
      case 'minor':
        return 'warning';
      case 'patch':
        return 'success';
      case 'preview':
        return 'info';
      case 'beta':
        return 'secondary';
      case 'alpha':
        return 'default';
      default:
        return 'default';
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'published':
        return 'success';
      case 'draft':
        return 'warning';
      case 'archived':
        return 'default';
      case 'deprecated':
        return 'error';
      default:
        return 'default';
    }
  };

  const getChangeTypeIcon = (type: string) => {
    switch (type) {
      case 'feature':
        return <AddIcon color="success" />;
      case 'bugfix':
        return <BugReportIcon color="error" />;
      case 'breaking':
        return <WarningIcon color="warning" />;
      case 'security':
        return <SecurityIcon color="error" />;
      case 'performance':
        return <SpeedIcon color="info" />;
      case 'documentation':
        return <InfoIcon color="primary" />;
      default:
        return <InfoIcon />;
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
          Project Versioning
        </Typography>
        <Box sx={{ display: 'flex', gap: 2 }}>
          <Button
            variant="outlined"
            startIcon={<BranchIcon />}
            onClick={() => setShowBranchDialog(true)}
          >
            Create Branch
          </Button>
          <Button
            variant="contained"
            startIcon={<TagIcon />}
            onClick={() => setShowCreateVersionDialog(true)}
          >
            Create Version
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

      {/* Main Tabs */}
      <Card>
        <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
          <Tabs value={tabValue} onChange={(_, newValue) => setTabValue(newValue)}>
            <Tab icon={<HistoryIcon />} label="Version History" />
            <Tab icon={<BranchIcon />} label="Branches" />
            <Tab icon={<CompareIcon />} label="Compare" />
            <Tab icon={<SettingsIcon />} label="Settings" />
          </Tabs>
        </Box>

        {/* Version History Tab */}
        <TabPanel value={tabValue} index={0}>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
            <Typography variant="h6">
              Version History ({versions.length})
            </Typography>
            <Button
              variant="contained"
              startIcon={<TagIcon />}
              onClick={() => setShowCreateVersionDialog(true)}
            >
              Create Version
            </Button>
          </Box>

          <Grid container spacing={3}>
            {versions.map((version) => (
              <Grid item xs={12} key={version.id}>
                <Card>
                  <CardContent>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', mb: 2 }}>
                      <Box>
                        <Typography variant="h6" gutterBottom>
                          {version.name} (v{version.version})
                        </Typography>
                        <Typography variant="body2" color="textSecondary" sx={{ mb: 1 }}>
                          {version.description}
                        </Typography>
                        <Box sx={{ display: 'flex', gap: 1, mb: 2 }}>
                          <Chip
                            label={version.type}
                            color={getVersionTypeColor(version.type) as any}
                            size="small"
                          />
                          <Chip
                            label={version.status}
                            color={getStatusColor(version.status) as any}
                            size="small"
                          />
                          <Chip
                            label={`Commit: ${version.commitHash.substring(0, 8)}`}
                            size="small"
                            variant="outlined"
                          />
                        </Box>
                      </Box>
                      <Box sx={{ display: 'flex', gap: 1 }}>
                        <Tooltip title="Download Version">
                          <IconButton size="small">
                            <DownloadIcon />
                          </IconButton>
                        </Tooltip>
                        <Tooltip title="Compare with Previous">
                          <IconButton size="small">
                            <CompareIcon />
                          </IconButton>
                        </Tooltip>
                        <Tooltip title="Restore Version">
                          <IconButton size="small">
                            <RestoreIcon />
                          </IconButton>
                        </Tooltip>
                      </Box>
                    </Box>

                    <Typography variant="subtitle2" gutterBottom>
                      Changes
                    </Typography>
                    <List dense>
                      {version.changes.map((change) => (
                        <ListItem key={change.id} sx={{ pl: 0 }}>
                          <ListItemAvatar>
                            {getChangeTypeIcon(change.type)}
                          </ListItemAvatar>
                          <ListItemText
                            primary={change.title}
                            secondary={change.description}
                          />
                        </ListItem>
                      ))}
                    </List>

                    <Typography variant="subtitle2" gutterBottom>
                      Dependencies
                    </Typography>
                    <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                      {version.dependencies.map((dep, index) => (
                        <Chip
                          key={index}
                          label={`${dep.name}@${dep.version}`}
                          size="small"
                          color={dep.status === 'vulnerable' ? 'error' : dep.status === 'outdated' ? 'warning' : 'default'}
                          variant="outlined"
                        />
                      ))}
                    </Box>

                    <Typography variant="body2" color="textSecondary" sx={{ mt: 2 }}>
                      Created by {version.createdBy} on {formatDate(version.createdAt)}
                    </Typography>
                  </CardContent>
                </Card>
              </Grid>
            ))}
          </Grid>
        </TabPanel>

        {/* Branches Tab */}
        <TabPanel value={tabValue} index={1}>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
            <Typography variant="h6">
              Branches ({branches.length})
            </Typography>
            <Button
              variant="contained"
              startIcon={<BranchIcon />}
              onClick={() => setShowBranchDialog(true)}
            >
              Create Branch
            </Button>
          </Box>

          <TableContainer component={Paper}>
            <Table>
              <TableHead>
                <TableRow>
                  <TableCell>Branch</TableCell>
                  <TableCell>Type</TableCell>
                  <TableCell>Status</TableCell>
                  <TableCell>Last Commit</TableCell>
                  <TableCell>Ahead/Behind</TableCell>
                  <TableCell>Protected</TableCell>
                  <TableCell>Actions</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {branches.map((branch) => (
                  <TableRow key={branch.id}>
                    <TableCell>
                      <Typography variant="subtitle2">{branch.name}</Typography>
                    </TableCell>
                    <TableCell>
                      <Chip label={branch.type} size="small" />
                    </TableCell>
                    <TableCell>
                      <Chip
                        label={branch.status}
                        color={branch.status === 'active' ? 'success' : 'default'}
                        size="small"
                      />
                    </TableCell>
                    <TableCell>
                      <Box>
                        <Typography variant="body2">
                          {branch.lastCommit.substring(0, 8)}
                        </Typography>
                        <Typography variant="caption" color="textSecondary">
                          {formatDate(branch.lastCommitDate)}
                        </Typography>
                      </Box>
                    </TableCell>
                    <TableCell>
                      <Typography variant="body2">
                        +{branch.aheadCount} / -{branch.behindCount}
                      </Typography>
                    </TableCell>
                    <TableCell>
                      <Chip
                        label={branch.protected ? 'Yes' : 'No'}
                        color={branch.protected ? 'success' : 'default'}
                        size="small"
                      />
                    </TableCell>
                    <TableCell>
                      <Box sx={{ display: 'flex', gap: 1 }}>
                        <Tooltip title="Merge Branch">
                          <IconButton size="small">
                            <MergeIcon />
                          </IconButton>
                        </Tooltip>
                        <Tooltip title="Compare Branches">
                          <IconButton size="small">
                            <CompareIcon />
                          </IconButton>
                        </Tooltip>
                        {branch.type !== 'main' && (
                          <Tooltip title="Delete Branch">
                            <IconButton size="small" color="error">
                              <DeleteIcon />
                            </IconButton>
                          </Tooltip>
                        )}
                      </Box>
                    </TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          </TableContainer>
        </TabPanel>

        {/* Compare Tab */}
        <TabPanel value={tabValue} index={2}>
          <Typography variant="h6" gutterBottom>
            Compare Versions
          </Typography>
          
          <Grid container spacing={3}>
            <Grid item xs={12} md={6}>
              <FormControl fullWidth margin="normal">
                <InputLabel>From Version</InputLabel>
                <Select label="From Version">
                  {versions.map((version) => (
                    <MenuItem key={version.id} value={version.id}>
                      {version.version} - {version.name}
                    </MenuItem>
                  ))}
                </Select>
              </FormControl>
            </Grid>
            
            <Grid item xs={12} md={6}>
              <FormControl fullWidth margin="normal">
                <InputLabel>To Version</InputLabel>
                <Select label="To Version">
                  {versions.map((version) => (
                    <MenuItem key={version.id} value={version.id}>
                      {version.version} - {version.name}
                    </MenuItem>
                  ))}
                </Select>
              </FormControl>
            </Grid>
          </Grid>

          <Button
            variant="contained"
            startIcon={<CompareIcon />}
            sx={{ mt: 2 }}
          >
            Compare Versions
          </Button>
        </TabPanel>

        {/* Settings Tab */}
        <TabPanel value={tabValue} index={3}>
          <Typography variant="h6" gutterBottom>
            Versioning Settings
          </Typography>
          
          <Grid container spacing={3}>
            <Grid item xs={12} md={6}>
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    Version Management
                  </Typography>
                  
                  <FormControlLabel
                    control={<Switch defaultChecked />}
                    label="Auto-increment version numbers"
                  />
                  
                  <FormControlLabel
                    control={<Switch defaultChecked />}
                    label="Require release notes"
                  />
                  
                  <FormControlLabel
                    control={<Switch />}
                    label="Enable semantic versioning"
                  />
                  
                  <FormControlLabel
                    control={<Switch defaultChecked />}
                    label="Auto-create tags"
                  />
                </CardContent>
              </Card>
            </Grid>

            <Grid item xs={12} md={6}>
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    Branch Protection
                  </Typography>
                  
                  <FormControlLabel
                    control={<Switch defaultChecked />}
                    label="Protect main branch"
                  />
                  
                  <FormControlLabel
                    control={<Switch defaultChecked />}
                    label="Require pull request reviews"
                  />
                  
                  <FormControlLabel
                    control={<Switch />}
                    label="Require status checks"
                  />
                  
                  <FormControlLabel
                    control={<Switch defaultChecked />}
                    label="Prevent force pushes"
                  />
                </CardContent>
              </Card>
            </Grid>
          </Grid>
        </TabPanel>
      </Card>

      {/* Create Version Dialog */}
      <Dialog open={showCreateVersionDialog} onClose={() => setShowCreateVersionDialog(false)} maxWidth="md" fullWidth>
        <DialogTitle>Create New Version</DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 2 }}>
            <Grid container spacing={2}>
              <Grid item xs={12} md={6}>
                <TextField
                  fullWidth
                  label="Version Number"
                  placeholder="1.2.0"
                  margin="normal"
                />
              </Grid>
              <Grid item xs={12} md={6}>
                <FormControl fullWidth margin="normal">
                  <InputLabel>Version Type</InputLabel>
                  <Select label="Version Type">
                    <MenuItem value="major">Major</MenuItem>
                    <MenuItem value="minor">Minor</MenuItem>
                    <MenuItem value="patch">Patch</MenuItem>
                    <MenuItem value="preview">Preview</MenuItem>
                    <MenuItem value="beta">Beta</MenuItem>
                    <MenuItem value="alpha">Alpha</MenuItem>
                  </Select>
                </FormControl>
              </Grid>
              <Grid item xs={12}>
                <TextField
                  fullWidth
                  label="Version Name"
                  placeholder="Feature Release"
                  margin="normal"
                />
              </Grid>
              <Grid item xs={12}>
                <TextField
                  fullWidth
                  label="Description"
                  multiline
                  rows={3}
                  placeholder="Describe what this version includes..."
                  margin="normal"
                />
              </Grid>
              <Grid item xs={12}>
                <TextField
                  fullWidth
                  label="Release Notes"
                  multiline
                  rows={5}
                  placeholder="Detailed release notes..."
                  margin="normal"
                />
              </Grid>
            </Grid>
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowCreateVersionDialog(false)}>Cancel</Button>
          <Button variant="contained" onClick={handleCreateVersion}>
            Create Version
          </Button>
        </DialogActions>
      </Dialog>

      {/* Create Branch Dialog */}
      <Dialog open={showBranchDialog} onClose={() => setShowBranchDialog(false)} maxWidth="sm" fullWidth>
        <DialogTitle>Create New Branch</DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 2 }}>
            <TextField
              fullWidth
              label="Branch Name"
              placeholder="feature/new-feature"
              margin="normal"
            />
            
            <FormControl fullWidth margin="normal">
              <InputLabel>Branch Type</InputLabel>
              <Select label="Branch Type">
                <MenuItem value="feature">Feature</MenuItem>
                <MenuItem value="hotfix">Hotfix</MenuItem>
                <MenuItem value="release">Release</MenuItem>
              </Select>
            </FormControl>
            
            <FormControl fullWidth margin="normal">
              <InputLabel>Base Branch</InputLabel>
              <Select label="Base Branch">
                {branches.map((branch) => (
                  <MenuItem key={branch.id} value={branch.name}>
                    {branch.name}
                  </MenuItem>
                ))}
              </Select>
            </FormControl>
            
            <FormControlLabel
              control={<Switch />}
              label="Protect this branch"
            />
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowBranchDialog(false)}>Cancel</Button>
          <Button variant="contained" onClick={handleCreateBranch}>
            Create Branch
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default ProjectVersioning; 