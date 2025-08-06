import React, { useState, useEffect } from 'react';
import {
  Box,
  Typography,
  Card,
  CardContent,
  Tabs,
  Tab,
  TextField,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Switch,
  FormControlLabel,
  Button,
  Chip,
  Alert,
  CircularProgress,
  Grid,
  Divider,
  List,
  ListItem,
  ListItemText,
  ListItemSecondaryAction,
  IconButton,
  Slider,
  Radio,
  RadioGroup,
  FormLabel,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Paper,
} from '@mui/material';
import {
  Settings as SettingsIcon,
  Environment as EnvironmentIcon,
  Code as CodeIcon,
  Storage as StorageIcon,
  Security as SecurityIcon,
  Save as SaveIcon,
  Cancel as CancelIcon,
  Add as AddIcon,
  Remove as RemoveIcon,
  Edit as EditIcon,
  Delete as DeleteIcon,
  ExpandMore as ExpandMoreIcon,
  Warning as WarningIcon,
  CheckCircle as CheckCircleIcon,
  Error as ErrorIcon,
  Info as InfoIcon,
} from '@mui/icons-material';
import { useParams, useNavigate } from 'react-router-dom';

// Types
interface ProjectSettings {
  name: string;
  description: string;
  category: string;
  tags: string[];
  isPublic: boolean;
  autoSave: boolean;
  enableDebugging: boolean;
  enableLogging: boolean;
  maxMemory: number;
  cpuLimit: number;
  enableGPU: boolean;
  backupFrequency: 'daily' | 'weekly' | 'monthly' | 'never';
}

interface EnvironmentConfig {
  baseImage: string;
  pythonVersion: string;
  nodeVersion: string;
  systemDependencies: string[];
  pythonDependencies: string[];
  nodeDependencies: string[];
  environmentVariables: Record<string, string>;
  ports: number[];
  volumes: string[];
}

interface Dependency {
  name: string;
  version: string;
  type: 'system' | 'python' | 'node' | 'ros';
  required: boolean;
  description: string;
}

interface SimulationConfig {
  enableSimulation: boolean;
  simulationEngine: 'gazebo' | 'rviz' | 'custom';
  physicsEngine: 'ode' | 'bullet' | 'simbody';
  timeStep: number;
  maxSimulationTime: number;
  enableVisualization: boolean;
  enableRecording: boolean;
  recordingFormat: 'bag' | 'video' | 'data';
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
      id={`settings-tabpanel-${index}`}
      aria-labelledby={`settings-tab-${index}`}
      {...other}
    >
      {value === index && <Box sx={{ p: 3 }}>{children}</Box>}
    </div>
  );
}

const ProjectSettings: React.FC = () => {
  const { projectId } = useParams<{ projectId: string }>();
  const navigate = useNavigate();
  const [tabValue, setTabValue] = useState(0);
  const [loading, setLoading] = useState(false);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);
  const [showDeleteDialog, setShowDeleteDialog] = useState(false);

  // Project Settings State
  const [projectSettings, setProjectSettings] = useState<ProjectSettings>({
    name: 'Autonomous Navigation Project',
    description: 'A project for navigating complex environments autonomously using advanced AI algorithms and sensor fusion.',
    category: 'Navigation',
    tags: ['autonomous', 'navigation', 'ai'],
    isPublic: false,
    autoSave: true,
    enableDebugging: false,
    enableLogging: true,
    maxMemory: 4096,
    cpuLimit: 4,
    enableGPU: false,
    backupFrequency: 'weekly',
  });

  // Environment Configuration State
  const [environmentConfig, setEnvironmentConfig] = useState<EnvironmentConfig>({
    baseImage: 'python:3.9-slim',
    pythonVersion: '3.9',
    nodeVersion: '18',
    systemDependencies: ['build-essential', 'git', 'curl'],
    pythonDependencies: ['numpy', 'opencv-python', 'ros2'],
    nodeDependencies: ['express', 'socket.io'],
    environmentVariables: {
      ROS_DOMAIN_ID: '0',
      PYTHONPATH: '/opt/ros/humble/lib/python3.9/site-packages',
    },
    ports: [8080, 11311],
    volumes: ['/tmp/ros2', '/var/log/ros2'],
  });

  // Dependencies State
  const [dependencies, setDependencies] = useState<Dependency[]>([
    {
      name: 'numpy',
      version: '1.21.0',
      type: 'python',
      required: true,
      description: 'Numerical computing library',
    },
    {
      name: 'opencv-python',
      version: '4.5.0',
      type: 'python',
      required: true,
      description: 'Computer vision library',
    },
    {
      name: 'ros2',
      version: 'humble',
      type: 'ros',
      required: true,
      description: 'Robot Operating System 2',
    },
  ]);

  // Simulation Configuration State
  const [simulationConfig, setSimulationConfig] = useState<SimulationConfig>({
    enableSimulation: true,
    simulationEngine: 'gazebo',
    physicsEngine: 'ode',
    timeStep: 0.01,
    maxSimulationTime: 3600,
    enableVisualization: true,
    enableRecording: false,
    recordingFormat: 'bag',
  });

  // New dependency form state
  const [newDependency, setNewDependency] = useState<Partial<Dependency>>({
    name: '',
    version: '',
    type: 'python',
    required: true,
    description: '',
  });

  // New environment variable form state
  const [newEnvVar, setNewEnvVar] = useState({ key: '', value: '' });

  useEffect(() => {
    // Load project settings from API
    loadProjectSettings();
  }, [projectId]);

  const loadProjectSettings = async () => {
    setLoading(true);
    try {
      // TODO: Replace with actual API call
      // const response = await ApiService.getProjectSettings(projectId);
      // setProjectSettings(response.settings);
      // setEnvironmentConfig(response.environment);
      // setDependencies(response.dependencies);
      // setSimulationConfig(response.simulation);
      
      // Simulate API delay
      await new Promise(resolve => setTimeout(resolve, 1000));
      setLoading(false);
    } catch (err) {
      setError('Failed to load project settings');
      setLoading(false);
    }
  };

  const handleSaveSettings = async () => {
    setSaving(true);
    setError(null);
    setSuccess(null);
    
    try {
      // TODO: Replace with actual API call
      // await ApiService.updateProjectSettings(projectId, {
      //   settings: projectSettings,
      //   environment: environmentConfig,
      //   dependencies,
      //   simulation: simulationConfig,
      // });
      
      // Simulate API delay
      await new Promise(resolve => setTimeout(resolve, 1000));
      setSuccess('Project settings saved successfully');
      setSaving(false);
    } catch (err) {
      setError('Failed to save project settings');
      setSaving(false);
    }
  };

  const handleDeleteProject = async () => {
    try {
      // TODO: Replace with actual API call
      // await ApiService.deleteProject(projectId);
      navigate('/projects');
    } catch (err) {
      setError('Failed to delete project');
    }
  };

  const addDependency = () => {
    if (newDependency.name && newDependency.version) {
      setDependencies([
        ...dependencies,
        {
          name: newDependency.name,
          version: newDependency.version,
          type: newDependency.type || 'python',
          required: newDependency.required || true,
          description: newDependency.description || '',
        } as Dependency,
      ]);
      setNewDependency({ name: '', version: '', type: 'python', required: true, description: '' });
    }
  };

  const removeDependency = (index: number) => {
    setDependencies(dependencies.filter((_, i) => i !== index));
  };

  const addEnvironmentVariable = () => {
    if (newEnvVar.key && newEnvVar.value) {
      setEnvironmentConfig({
        ...environmentConfig,
        environmentVariables: {
          ...environmentConfig.environmentVariables,
          [newEnvVar.key]: newEnvVar.value,
        },
      });
      setNewEnvVar({ key: '', value: '' });
    }
  };

  const removeEnvironmentVariable = (key: string) => {
    const newEnvVars = { ...environmentConfig.environmentVariables };
    delete newEnvVars[key];
    setEnvironmentConfig({
      ...environmentConfig,
      environmentVariables: newEnvVars,
    });
  };

  const handleTabChange = (event: React.SyntheticEvent, newValue: number) => {
    setTabValue(newValue);
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
          Project Settings
        </Typography>
        <Box sx={{ display: 'flex', gap: 2 }}>
          <Button
            variant="outlined"
            color="error"
            onClick={() => setShowDeleteDialog(true)}
          >
            Delete Project
          </Button>
          <Button
            variant="contained"
            startIcon={<SaveIcon />}
            onClick={handleSaveSettings}
            disabled={saving}
          >
            {saving ? <CircularProgress size={20} /> : 'Save Settings'}
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

      {/* Settings Tabs */}
      <Card>
        <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
          <Tabs value={tabValue} onChange={handleTabChange} aria-label="project settings tabs">
            <Tab icon={<SettingsIcon />} label="General" />
            <Tab icon={<EnvironmentIcon />} label="Environment" />
            <Tab icon={<CodeIcon />} label="Dependencies" />
            <Tab icon={<StorageIcon />} label="Simulation" />
            <Tab icon={<SecurityIcon />} label="Advanced" />
          </Tabs>
        </Box>

        {/* General Settings Tab */}
        <TabPanel value={tabValue} index={0}>
          <Grid container spacing={3}>
            <Grid item xs={12} md={6}>
              <TextField
                fullWidth
                label="Project Name"
                value={projectSettings.name}
                onChange={(e) => setProjectSettings({ ...projectSettings, name: e.target.value })}
                margin="normal"
              />
            </Grid>
            <Grid item xs={12} md={6}>
              <FormControl fullWidth margin="normal">
                <InputLabel>Category</InputLabel>
                <Select
                  value={projectSettings.category}
                  onChange={(e) => setProjectSettings({ ...projectSettings, category: e.target.value })}
                  label="Category"
                >
                  <MenuItem value="Navigation">Navigation</MenuItem>
                  <MenuItem value="Manipulation">Manipulation</MenuItem>
                  <MenuItem value="Perception">Perception</MenuItem>
                  <MenuItem value="Logistics">Logistics</MenuItem>
                  <MenuItem value="Healthcare">Healthcare</MenuItem>
                </Select>
              </FormControl>
            </Grid>
            <Grid item xs={12}>
              <TextField
                fullWidth
                label="Description"
                value={projectSettings.description}
                onChange={(e) => setProjectSettings({ ...projectSettings, description: e.target.value })}
                multiline
                rows={3}
                margin="normal"
              />
            </Grid>
            <Grid item xs={12}>
              <FormControlLabel
                control={
                  <Switch
                    checked={projectSettings.isPublic}
                    onChange={(e) => setProjectSettings({ ...projectSettings, isPublic: e.target.checked })}
                  />
                }
                label="Public Project"
              />
            </Grid>
            <Grid item xs={12}>
              <Typography variant="h6" gutterBottom>
                Application Settings
              </Typography>
              <Grid container spacing={2}>
                <Grid item xs={12} md={6}>
                  <FormControlLabel
                    control={
                      <Switch
                        checked={projectSettings.autoSave}
                        onChange={(e) => setProjectSettings({ ...projectSettings, autoSave: e.target.checked })}
                      />
                    }
                    label="Auto-save changes"
                  />
                </Grid>
                <Grid item xs={12} md={6}>
                  <FormControlLabel
                    control={
                      <Switch
                        checked={projectSettings.enableDebugging}
                        onChange={(e) => setProjectSettings({ ...projectSettings, enableDebugging: e.target.checked })}
                      />
                    }
                    label="Enable debugging"
                  />
                </Grid>
                <Grid item xs={12} md={6}>
                  <FormControlLabel
                    control={
                      <Switch
                        checked={projectSettings.enableLogging}
                        onChange={(e) => setProjectSettings({ ...projectSettings, enableLogging: e.target.checked })}
                      />
                    }
                    label="Enable logging"
                  />
                </Grid>
                <Grid item xs={12} md={6}>
                  <FormControlLabel
                    control={
                      <Switch
                        checked={projectSettings.enableGPU}
                        onChange={(e) => setProjectSettings({ ...projectSettings, enableGPU: e.target.checked })}
                      />
                    }
                    label="Enable GPU acceleration"
                  />
                </Grid>
              </Grid>
            </Grid>
            <Grid item xs={12} md={6}>
              <Typography gutterBottom>Memory Limit (MB)</Typography>
              <Slider
                value={projectSettings.maxMemory}
                onChange={(_, value) => setProjectSettings({ ...projectSettings, maxMemory: value as number })}
                min={512}
                max={8192}
                step={512}
                marks
                valueLabelDisplay="auto"
              />
            </Grid>
            <Grid item xs={12} md={6}>
              <Typography gutterBottom>CPU Limit (cores)</Typography>
              <Slider
                value={projectSettings.cpuLimit}
                onChange={(_, value) => setProjectSettings({ ...projectSettings, cpuLimit: value as number })}
                min={1}
                max={8}
                step={1}
                marks
                valueLabelDisplay="auto"
              />
            </Grid>
            <Grid item xs={12}>
              <FormControl component="fieldset">
                <FormLabel component="legend">Backup Frequency</FormLabel>
                <RadioGroup
                  value={projectSettings.backupFrequency}
                  onChange={(e) => setProjectSettings({ ...projectSettings, backupFrequency: e.target.value as any })}
                >
                  <FormControlLabel value="daily" control={<Radio />} label="Daily" />
                  <FormControlLabel value="weekly" control={<Radio />} label="Weekly" />
                  <FormControlLabel value="monthly" control={<Radio />} label="Monthly" />
                  <FormControlLabel value="never" control={<Radio />} label="Never" />
                </RadioGroup>
              </FormControl>
            </Grid>
          </Grid>
        </TabPanel>

        {/* Environment Settings Tab */}
        <TabPanel value={tabValue} index={1}>
          <Grid container spacing={3}>
            <Grid item xs={12} md={6}>
              <TextField
                fullWidth
                label="Base Image"
                value={environmentConfig.baseImage}
                onChange={(e) => setEnvironmentConfig({ ...environmentConfig, baseImage: e.target.value })}
                margin="normal"
                helperText="Docker base image for the project"
              />
            </Grid>
            <Grid item xs={12} md={3}>
              <TextField
                fullWidth
                label="Python Version"
                value={environmentConfig.pythonVersion}
                onChange={(e) => setEnvironmentConfig({ ...environmentConfig, pythonVersion: e.target.value })}
                margin="normal"
              />
            </Grid>
            <Grid item xs={12} md={3}>
              <TextField
                fullWidth
                label="Node Version"
                value={environmentConfig.nodeVersion}
                onChange={(e) => setEnvironmentConfig({ ...environmentConfig, nodeVersion: e.target.value })}
                margin="normal"
              />
            </Grid>
            
            {/* Environment Variables */}
            <Grid item xs={12}>
              <Typography variant="h6" gutterBottom>
                Environment Variables
              </Typography>
              <TableContainer component={Paper} variant="outlined">
                <Table>
                  <TableHead>
                    <TableRow>
                      <TableCell>Key</TableCell>
                      <TableCell>Value</TableCell>
                      <TableCell align="right">Actions</TableCell>
                    </TableRow>
                  </TableHead>
                  <TableBody>
                    {Object.entries(environmentConfig.environmentVariables).map(([key, value]) => (
                      <TableRow key={key}>
                        <TableCell>{key}</TableCell>
                        <TableCell>{value}</TableCell>
                        <TableCell align="right">
                          <IconButton
                            size="small"
                            onClick={() => removeEnvironmentVariable(key)}
                            color="error"
                          >
                            <DeleteIcon />
                          </IconButton>
                        </TableCell>
                      </TableRow>
                    ))}
                  </TableBody>
                </Table>
              </TableContainer>
              
              <Box sx={{ mt: 2, display: 'flex', gap: 2, alignItems: 'center' }}>
                <TextField
                  label="Key"
                  value={newEnvVar.key}
                  onChange={(e) => setNewEnvVar({ ...newEnvVar, key: e.target.value })}
                  size="small"
                />
                <TextField
                  label="Value"
                  value={newEnvVar.value}
                  onChange={(e) => setNewEnvVar({ ...newEnvVar, value: e.target.value })}
                  size="small"
                />
                <Button
                  variant="outlined"
                  startIcon={<AddIcon />}
                  onClick={addEnvironmentVariable}
                  disabled={!newEnvVar.key || !newEnvVar.value}
                >
                  Add
                </Button>
              </Box>
            </Grid>

            {/* Ports */}
            <Grid item xs={12} md={6}>
              <Typography variant="h6" gutterBottom>
                Ports
              </Typography>
              <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
                {environmentConfig.ports.map((port, index) => (
                  <Chip
                    key={index}
                    label={port}
                    onDelete={() => {
                      const newPorts = environmentConfig.ports.filter((_, i) => i !== index);
                      setEnvironmentConfig({ ...environmentConfig, ports: newPorts });
                    }}
                  />
                ))}
              </Box>
            </Grid>

            {/* Volumes */}
            <Grid item xs={12} md={6}>
              <Typography variant="h6" gutterBottom>
                Volumes
              </Typography>
              <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
                {environmentConfig.volumes.map((volume, index) => (
                  <Chip
                    key={index}
                    label={volume}
                    onDelete={() => {
                      const newVolumes = environmentConfig.volumes.filter((_, i) => i !== index);
                      setEnvironmentConfig({ ...environmentConfig, volumes: newVolumes });
                    }}
                  />
                ))}
              </Box>
            </Grid>
          </Grid>
        </TabPanel>

        {/* Dependencies Tab */}
        <TabPanel value={tabValue} index={2}>
          <Grid container spacing={3}>
            <Grid item xs={12}>
              <Typography variant="h6" gutterBottom>
                Project Dependencies
              </Typography>
              
              <TableContainer component={Paper} variant="outlined">
                <Table>
                  <TableHead>
                    <TableRow>
                      <TableCell>Name</TableCell>
                      <TableCell>Version</TableCell>
                      <TableCell>Type</TableCell>
                      <TableCell>Required</TableCell>
                      <TableCell>Description</TableCell>
                      <TableCell align="right">Actions</TableCell>
                    </TableRow>
                  </TableHead>
                  <TableBody>
                    {dependencies.map((dep, index) => (
                      <TableRow key={index}>
                        <TableCell>{dep.name}</TableCell>
                        <TableCell>{dep.version}</TableCell>
                        <TableCell>
                          <Chip
                            label={dep.type}
                            size="small"
                            color={dep.type === 'ros' ? 'primary' : 'default'}
                          />
                        </TableCell>
                        <TableCell>
                          <Chip
                            label={dep.required ? 'Required' : 'Optional'}
                            size="small"
                            color={dep.required ? 'error' : 'default'}
                          />
                        </TableCell>
                        <TableCell>{dep.description}</TableCell>
                        <TableCell align="right">
                          <IconButton
                            size="small"
                            onClick={() => removeDependency(index)}
                            color="error"
                          >
                            <DeleteIcon />
                          </IconButton>
                        </TableCell>
                      </TableRow>
                    ))}
                  </TableBody>
                </Table>
              </TableContainer>
            </Grid>

            {/* Add New Dependency */}
            <Grid item xs={12}>
              <Card variant="outlined">
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    Add New Dependency
                  </Typography>
                  <Grid container spacing={2}>
                    <Grid item xs={12} md={3}>
                      <TextField
                        fullWidth
                        label="Name"
                        value={newDependency.name}
                        onChange={(e) => setNewDependency({ ...newDependency, name: e.target.value })}
                        size="small"
                      />
                    </Grid>
                    <Grid item xs={12} md={2}>
                      <TextField
                        fullWidth
                        label="Version"
                        value={newDependency.version}
                        onChange={(e) => setNewDependency({ ...newDependency, version: e.target.value })}
                        size="small"
                      />
                    </Grid>
                    <Grid item xs={12} md={2}>
                      <FormControl fullWidth size="small">
                        <InputLabel>Type</InputLabel>
                        <Select
                          value={newDependency.type}
                          onChange={(e) => setNewDependency({ ...newDependency, type: e.target.value as any })}
                          label="Type"
                        >
                          <MenuItem value="python">Python</MenuItem>
                          <MenuItem value="node">Node.js</MenuItem>
                          <MenuItem value="system">System</MenuItem>
                          <MenuItem value="ros">ROS</MenuItem>
                        </Select>
                      </FormControl>
                    </Grid>
                    <Grid item xs={12} md={2}>
                      <FormControlLabel
                        control={
                          <Switch
                            checked={newDependency.required}
                            onChange={(e) => setNewDependency({ ...newDependency, required: e.target.checked })}
                            size="small"
                          />
                        }
                        label="Required"
                      />
                    </Grid>
                    <Grid item xs={12} md={3}>
                      <TextField
                        fullWidth
                        label="Description"
                        value={newDependency.description}
                        onChange={(e) => setNewDependency({ ...newDependency, description: e.target.value })}
                        size="small"
                      />
                    </Grid>
                    <Grid item xs={12}>
                      <Button
                        variant="contained"
                        startIcon={<AddIcon />}
                        onClick={addDependency}
                        disabled={!newDependency.name || !newDependency.version}
                      >
                        Add Dependency
                      </Button>
                    </Grid>
                  </Grid>
                </CardContent>
              </Card>
            </Grid>
          </Grid>
        </TabPanel>

        {/* Simulation Settings Tab */}
        <TabPanel value={tabValue} index={3}>
          <Grid container spacing={3}>
            <Grid item xs={12}>
              <FormControlLabel
                control={
                  <Switch
                    checked={simulationConfig.enableSimulation}
                    onChange={(e) => setSimulationConfig({ ...simulationConfig, enableSimulation: e.target.checked })}
                  />
                }
                label="Enable Simulation"
              />
            </Grid>
            
            {simulationConfig.enableSimulation && (
              <>
                <Grid item xs={12} md={6}>
                  <FormControl fullWidth>
                    <InputLabel>Simulation Engine</InputLabel>
                    <Select
                      value={simulationConfig.simulationEngine}
                      onChange={(e) => setSimulationConfig({ ...simulationConfig, simulationEngine: e.target.value as any })}
                      label="Simulation Engine"
                    >
                      <MenuItem value="gazebo">Gazebo</MenuItem>
                      <MenuItem value="rviz">RViz</MenuItem>
                      <MenuItem value="custom">Custom</MenuItem>
                    </Select>
                  </FormControl>
                </Grid>
                
                <Grid item xs={12} md={6}>
                  <FormControl fullWidth>
                    <InputLabel>Physics Engine</InputLabel>
                    <Select
                      value={simulationConfig.physicsEngine}
                      onChange={(e) => setSimulationConfig({ ...simulationConfig, physicsEngine: e.target.value as any })}
                      label="Physics Engine"
                    >
                      <MenuItem value="ode">ODE</MenuItem>
                      <MenuItem value="bullet">Bullet</MenuItem>
                      <MenuItem value="simbody">Simbody</MenuItem>
                    </Select>
                  </FormControl>
                </Grid>
                
                <Grid item xs={12} md={6}>
                  <Typography gutterBottom>Time Step (seconds)</Typography>
                  <Slider
                    value={simulationConfig.timeStep}
                    onChange={(_, value) => setSimulationConfig({ ...simulationConfig, timeStep: value as number })}
                    min={0.001}
                    max={0.1}
                    step={0.001}
                    marks
                    valueLabelDisplay="auto"
                  />
                </Grid>
                
                <Grid item xs={12} md={6}>
                  <Typography gutterBottom>Max Simulation Time (seconds)</Typography>
                  <Slider
                    value={simulationConfig.maxSimulationTime}
                    onChange={(_, value) => setSimulationConfig({ ...simulationConfig, maxSimulationTime: value as number })}
                    min={60}
                    max={7200}
                    step={60}
                    marks
                    valueLabelDisplay="auto"
                  />
                </Grid>
                
                <Grid item xs={12}>
                  <Typography variant="h6" gutterBottom>
                    Visualization & Recording
                  </Typography>
                  <Grid container spacing={2}>
                    <Grid item xs={12} md={4}>
                      <FormControlLabel
                        control={
                          <Switch
                            checked={simulationConfig.enableVisualization}
                            onChange={(e) => setSimulationConfig({ ...simulationConfig, enableVisualization: e.target.checked })}
                          />
                        }
                        label="Enable Visualization"
                      />
                    </Grid>
                    <Grid item xs={12} md={4}>
                      <FormControlLabel
                        control={
                          <Switch
                            checked={simulationConfig.enableRecording}
                            onChange={(e) => setSimulationConfig({ ...simulationConfig, enableRecording: e.target.checked })}
                          />
                        }
                        label="Enable Recording"
                      />
                    </Grid>
                    <Grid item xs={12} md={4}>
                      <FormControl fullWidth>
                        <InputLabel>Recording Format</InputLabel>
                        <Select
                          value={simulationConfig.recordingFormat}
                          onChange={(e) => setSimulationConfig({ ...simulationConfig, recordingFormat: e.target.value as any })}
                          label="Recording Format"
                        >
                          <MenuItem value="bag">ROS Bag</MenuItem>
                          <MenuItem value="video">Video</MenuItem>
                          <MenuItem value="data">Raw Data</MenuItem>
                        </Select>
                      </FormControl>
                    </Grid>
                  </Grid>
                </Grid>
              </>
            )}
          </Grid>
        </TabPanel>

        {/* Advanced Settings Tab */}
        <TabPanel value={tabValue} index={4}>
          <Grid container spacing={3}>
            <Grid item xs={12}>
              <Typography variant="h6" gutterBottom>
                Advanced Configuration
              </Typography>
              <Alert severity="info" sx={{ mb: 3 }}>
                These settings are for advanced users. Incorrect configuration may affect project functionality.
              </Alert>
            </Grid>
            
            <Grid item xs={12}>
              <Accordion>
                <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                  <Typography variant="h6">System Dependencies</Typography>
                </AccordionSummary>
                <AccordionDetails>
                  <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
                    {environmentConfig.systemDependencies.map((dep, index) => (
                      <Chip
                        key={index}
                        label={dep}
                        onDelete={() => {
                          const newDeps = environmentConfig.systemDependencies.filter((_, i) => i !== index);
                          setEnvironmentConfig({ ...environmentConfig, systemDependencies: newDeps });
                        }}
                      />
                    ))}
                  </Box>
                </AccordionDetails>
              </Accordion>
            </Grid>
            
            <Grid item xs={12}>
              <Accordion>
                <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                  <Typography variant="h6">Python Dependencies</Typography>
                </AccordionSummary>
                <AccordionDetails>
                  <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
                    {environmentConfig.pythonDependencies.map((dep, index) => (
                      <Chip
                        key={index}
                        label={dep}
                        onDelete={() => {
                          const newDeps = environmentConfig.pythonDependencies.filter((_, i) => i !== index);
                          setEnvironmentConfig({ ...environmentConfig, pythonDependencies: newDeps });
                        }}
                      />
                    ))}
                  </Box>
                </AccordionDetails>
              </Accordion>
            </Grid>
            
            <Grid item xs={12}>
              <Accordion>
                <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                  <Typography variant="h6">Node.js Dependencies</Typography>
                </AccordionSummary>
                <AccordionDetails>
                  <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
                    {environmentConfig.nodeDependencies.map((dep, index) => (
                      <Chip
                        key={index}
                        label={dep}
                        onDelete={() => {
                          const newDeps = environmentConfig.nodeDependencies.filter((_, i) => i !== index);
                          setEnvironmentConfig({ ...environmentConfig, nodeDependencies: newDeps });
                        }}
                      />
                    ))}
                  </Box>
                </AccordionDetails>
              </Accordion>
            </Grid>
          </Grid>
        </TabPanel>
      </Card>

      {/* Delete Project Dialog */}
      <Dialog open={showDeleteDialog} onClose={() => setShowDeleteDialog(false)}>
        <DialogTitle>Delete Project</DialogTitle>
        <DialogContent>
          <Typography>
            Are you sure you want to delete "{projectSettings.name}"? This action cannot be undone.
          </Typography>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowDeleteDialog(false)}>Cancel</Button>
          <Button onClick={handleDeleteProject} color="error" variant="contained">
            Delete Project
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default ProjectSettings; 