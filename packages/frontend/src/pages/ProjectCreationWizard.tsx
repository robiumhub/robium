import React, { useState, useEffect, useMemo, useCallback } from 'react';
import { useFormValidation } from '../hooks/useValidation';
import { ValidationFeedback } from '../components/ValidationFeedback';
import { useSearchParams } from 'react-router-dom';
import AIProjectGenerationService, {
  AIProjectSuggestion,
} from '../services/AIProjectGenerationService';
import { ApiService } from '../services/api';
import {
  Box,
  Typography,
  Stepper,
  Step,
  StepLabel,
  StepContent,
  Button,
  Paper,
  Card,
  CardContent,
  TextField,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Chip,
  Checkbox,
  FormControlLabel,
  Alert,
  CircularProgress,
  List,
  ListItem,
  ListItemText,
  ListItemIcon,
  Radio,
  RadioGroup,
  FormLabel,
  Slider,
  Switch,
  IconButton,
} from '@mui/material';
import { Grid } from '@mui/material';
import {
  Check as CheckIcon,
  Add as AddIcon,
  Remove as RemoveIcon,
  Save as SaveIcon,
  ArrowBack as ArrowBackIcon,
  ArrowForward as ArrowForwardIcon,
  Folder as FolderIcon,
  Code as CodeIcon,
  Settings as SettingsIcon,
  Preview as PreviewIcon,
  SmartToy as RobotIcon,
} from '@mui/icons-material';
import { useNavigate } from 'react-router-dom';

// Types
interface ProjectData {
  name: string;
  description: string;
  tags: string[];
  algorithms: string[];
  environment: EnvironmentConfig;
  settings: ProjectSettings;
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

interface ProjectSettings {
  autoSave: boolean;
  enableDebugging: boolean;
  enableLogging: boolean;
  maxMemory: number;
  cpuLimit: number;
  enableGPU: boolean;
  backupFrequency: 'daily' | 'weekly' | 'monthly' | 'never';
}

interface WizardStep {
  label: string;
  description: string;
  icon: React.ReactNode;
  completed: boolean;
  valid: boolean;
}

const ProjectCreationWizard: React.FC = () => {
  const navigate = useNavigate();
  const [searchParams] = useSearchParams();
  const [activeStep, setActiveStep] = useState(0);
  const [loading, setLoading] = useState(false);
  const [aiLoading, setAiLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isAIMode, setIsAIMode] = useState(false);
  const [userInput, setUserInput] = useState('');
  const [projectData, setProjectData] = useState<ProjectData>({
    name: '',
    description: '',
    tags: [],
    algorithms: [],
    environment: {
      baseImage: 'python:3.9-slim',
      pythonVersion: '3.9',
      nodeVersion: '16',
      systemDependencies: [],
      pythonDependencies: [],
      nodeDependencies: [],
      environmentVariables: {},
      ports: [8080],
      volumes: [],
    },
    settings: {
      autoSave: true,
      enableDebugging: false,
      enableLogging: true,
      maxMemory: 2048,
      cpuLimit: 2,
      enableGPU: false,
      backupFrequency: 'weekly',
    },
  });

  // Real-time form validation
  const fieldValidations = {
    name: [
      { type: 'required' as const, message: 'Project name is required' },
      {
        type: 'minLength' as const,
        value: 3,
        message: 'Project name must be at least 3 characters',
      },
      {
        type: 'maxLength' as const,
        value: 50,
        message: 'Project name must be no more than 50 characters',
      },
      {
        type: 'pattern' as const,
        value: /^[a-zA-Z0-9_-]+$/,
        message:
          'Project name can only contain letters, numbers, underscores, and hyphens',
      },
    ],
    description: [
      { type: 'required' as const, message: 'Project description is required' },
      {
        type: 'minLength' as const,
        value: 10,
        message: 'Project description must be at least 10 characters',
      },
      {
        type: 'maxLength' as const,
        value: 500,
        message: 'Project description must be no more than 500 characters',
      },
    ],
    algorithms: [
      {
        type: 'required' as const,
        message: 'At least one module must be selected',
      },
    ],
    'environment.baseImage': [
      { type: 'required' as const, message: 'Base image is required' },
    ],
  };

  const validation = useFormValidation(projectData, fieldValidations, {
    debounceDelay: 500,
    validateOnChange: true,
  });

  const baseImages = [
    { value: 'python:3.9-slim', label: 'Python 3.9 (Slim)' },
    { value: 'python:3.10-slim', label: 'Python 3.10 (Slim)' },
    { value: 'python:3.11-slim', label: 'Python 3.11 (Slim)' },
    { value: 'node:16-alpine', label: 'Node.js 16 (Alpine)' },
    { value: 'node:18-alpine', label: 'Node.js 18 (Alpine)' },
    { value: 'ubuntu:20.04', label: 'Ubuntu 20.04' },
    { value: 'ubuntu:22.04', label: 'Ubuntu 22.04' },
  ];

  // Modules fetched from backend
  const [modules, setModules] = useState<
    Array<{ id: string; name: string; category?: string; description?: string }>
  >([]);
  const [modulesLoading, setModulesLoading] = useState(false);
  const [modulesError, setModulesError] = useState<string | null>(null);

  useEffect(() => {
    const fetchModules = async () => {
      try {
        setModulesLoading(true);
        setModulesError(null);
        const data = await ApiService.get<any[]>('/modules');
        // Ensure shape
        const normalized = (Array.isArray(data) ? data : []).map((m: any) => ({
          id: m.id,
          name: m.name,
          category: m.category,
          description: m.description,
        }));
        setModules(normalized);
      } catch (err) {
        setModulesError(
          err instanceof Error ? err.message : 'Failed to load modules'
        );
      } finally {
        setModulesLoading(false);
      }
    };
    fetchModules();
  }, []);

  // Wizard steps - memoized to prevent re-creation on every render
  const steps: WizardStep[] = useMemo(
    () => [
      {
        label: 'Project Details',
        description: 'Basic project information',
        icon: <FolderIcon />,
        completed: false,
        valid: false,
      },
      {
        label: 'Algorithm Selection',
        description: 'Choose ROS algorithms and tools',
        icon: <CodeIcon />,
        completed: false,
        valid: false,
      },
      {
        label: 'Environment Configuration',
        description: 'Configure development environment',
        icon: <SettingsIcon />,
        completed: false,
        valid: false,
      },
      {
        label: 'Project Settings',
        description: 'Advanced project settings',
        icon: <SettingsIcon />,
        completed: false,
        valid: false,
      },
      {
        label: 'Review & Create',
        description: 'Review and create project',
        icon: <PreviewIcon />,
        completed: false,
        valid: false,
      },
    ],
    []
  );

  // Validation functions - memoized to prevent re-creation
  const validateStep = useCallback(
    (step: number): boolean => {
      switch (step) {
        case 0:
          return (
            projectData.name.trim().length > 0 &&
            projectData.description.trim().length > 0
          );
        case 1:
          return projectData.algorithms.length > 0;
        case 2:
          return projectData.environment.baseImage.length > 0;
        case 3:
          return true; // Settings are optional
        case 4:
          return true; // Review step
        default:
          return false;
      }
    },
    [projectData]
  );

  // Step validation state (separate from steps array to prevent mutations)
  const [stepValidations, setStepValidations] = useState<boolean[]>(
    new Array(6).fill(false)
  );

  // Check if we're in AI mode
  useEffect(() => {
    const mode = searchParams.get('mode');
    const input = searchParams.get('input');
    if (mode === 'ai' && input) {
      setIsAIMode(true);
      setUserInput(input);
      handleAIGeneration(input);
    }
  }, [searchParams]);

  // Update step validation without mutating steps array
  useEffect(() => {
    const isValid = validateStep(activeStep);
    setStepValidations((prev) => {
      const newValidations = [...prev];
      newValidations[activeStep] = isValid;
      return newValidations;
    });
  }, [projectData, activeStep, validateStep]);

  // Handle next step
  const handleNext = () => {
    if (activeStep < steps.length - 1) {
      setActiveStep(activeStep + 1);
    }
  };

  // Handle previous step
  const handleBack = () => {
    if (activeStep > 0) {
      setActiveStep(activeStep - 1);
    }
  };

  // Handle step completion
  const handleStepComplete = (step: number) => {
    steps[step].completed = true;
  };

  // Handle project creation
  const handleCreateProject = async () => {
    try {
      setLoading(true);
      setError(null);

      // Create project payload with full configuration
      const projectPayload = {
        name: projectData.name,
        description: projectData.description,
        tags: projectData.tags,
        algorithms: projectData.algorithms,
        environment: projectData.environment,
        settings: projectData.settings,
      };

      // Call the API using ApiService
      const result = await ApiService.post('/projects', projectPayload);
      console.log('Project created successfully:', result);

      // Navigate to the new project
      navigate('/projects');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to create project');
    } finally {
      setLoading(false);
    }
  };

  // Handle tag input
  const handleTagInput = (event: React.KeyboardEvent) => {
    if (event.key !== 'Enter') return;
    event.preventDefault();

    const inputElement =
      (event.target as HTMLInputElement | HTMLTextAreaElement) || null;
    const value = (inputElement?.value || '').trim();
    if (!value) return;

    if (!projectData.tags.includes(value)) {
      setProjectData((prev) => ({
        ...prev,
        tags: [...prev.tags, value],
      }));
    }

    if (inputElement) {
      inputElement.value = '';
    }
  };

  // Remove tag
  const removeTag = (tagToRemove: string) => {
    setProjectData((prev) => ({
      ...prev,
      tags: prev.tags.filter((tag) => tag !== tagToRemove),
    }));
  };

  // Handle AI generation
  const handleAIGeneration = async (input: string) => {
    try {
      setAiLoading(true);
      setError(null);

      const suggestion =
        await AIProjectGenerationService.generateProjectSuggestion(input);

      // Apply AI suggestions to the form
      setProjectData({
        name: suggestion.name,
        description: suggestion.description,
        tags: suggestion.tags,
        algorithms: suggestion.algorithms,
        environment: {
          baseImage: suggestion.environment.baseImage,
          pythonVersion: suggestion.environment.pythonVersion || '3.11',
          nodeVersion: suggestion.environment.nodeVersion || '16',
          systemDependencies: suggestion.environment.systemDependencies,
          pythonDependencies: suggestion.environment.pythonDependencies,
          nodeDependencies: suggestion.environment.nodeDependencies,
          environmentVariables: suggestion.environment.environmentVariables,
          ports: suggestion.environment.ports,
          volumes: [],
        },
        settings: {
          ...suggestion.settings,
          backupFrequency: suggestion.settings.backupFrequency as
            | 'daily'
            | 'weekly'
            | 'monthly'
            | 'never',
        },
      });
    } catch (err) {
      setError(
        err instanceof Error ? err.message : 'Failed to generate AI suggestions'
      );
    } finally {
      setAiLoading(false);
    }
  };

  // Toggle algorithm selection
  const toggleAlgorithm = (algorithmId: string) => {
    setProjectData((prev) => ({
      ...prev,
      algorithms: prev.algorithms.includes(algorithmId)
        ? prev.algorithms.filter((id) => id !== algorithmId)
        : [...prev.algorithms, algorithmId],
    }));
  };

  // Add environment variable
  const addEnvironmentVariable = () => {
    const key = `VAR_${Object.keys(projectData.environment.environmentVariables).length + 1}`;
    setProjectData((prev) => ({
      ...prev,
      environment: {
        ...prev.environment,
        environmentVariables: {
          ...prev.environment.environmentVariables,
          [key]: '',
        },
      },
    }));
  };

  // Remove environment variable
  const removeEnvironmentVariable = (key: string) => {
    const newEnvVars = { ...projectData.environment.environmentVariables };
    delete newEnvVars[key];
    setProjectData((prev) => ({
      ...prev,
      environment: {
        ...prev.environment,
        environmentVariables: newEnvVars,
      },
    }));
  };

  // Update environment variable
  const updateEnvironmentVariable = (key: string, value: string) => {
    setProjectData((prev) => ({
      ...prev,
      environment: {
        ...prev.environment,
        environmentVariables: {
          ...prev.environment.environmentVariables,
          [key]: value,
        },
      },
    }));
  };

  return (
    <Box sx={{ maxWidth: 1200, mx: 'auto' }}>
      <Box sx={{ display: 'flex', alignItems: 'center', gap: 2, mb: 2 }}>
        <Typography variant="h4" component="h1">
          {isAIMode ? '🤖 AI-Generated Project' : 'Create New Project'}
        </Typography>
        {isAIMode && (
          <Chip
            label="AI Mode"
            color="primary"
            variant="outlined"
            icon={<RobotIcon />}
          />
        )}
      </Box>
      <Typography variant="body1" color="text.secondary" sx={{ mb: 4 }}>
        {isAIMode
          ? `AI has analyzed your request and pre-filled the form. Review and customize as needed.`
          : 'Follow the steps below to create your new robotics project'}
      </Typography>

      {/* Error Alert */}
      {error && (
        <Alert severity="error" sx={{ mb: 3 }}>
          {error}
        </Alert>
      )}

      <Paper sx={{ p: 3 }}>
        <Stepper activeStep={activeStep} orientation="vertical">
          {steps.map((step, index) => (
            <Step key={step.label}>
              <StepLabel
                icon={step.icon}
                optional={
                  <Typography variant="caption" color="text.secondary">
                    {step.description}
                  </Typography>
                }
              >
                {step.label}
              </StepLabel>
              <StepContent>
                <Box sx={{ mb: 2 }}>
                  {/* Step 1: Project Details */}
                  {index === 0 && (
                    <Grid container spacing={3}>
                      <Grid item xs={12}>
                        <TextField
                          fullWidth
                          label="Project Name"
                          value={projectData.name}
                          onChange={(e) =>
                            setProjectData((prev) => ({
                              ...prev,
                              name: e.target.value,
                            }))
                          }
                          required
                          helperText="Enter a descriptive name for your project"
                        />
                      </Grid>
                      <Grid item xs={12}>
                        <TextField
                          fullWidth
                          label="Description"
                          value={projectData.description}
                          onChange={(e) =>
                            setProjectData((prev) => ({
                              ...prev,
                              description: e.target.value,
                            }))
                          }
                          multiline
                          rows={3}
                          required
                          helperText="Describe what your project does"
                        />
                      </Grid>

                      <Grid item xs={12}>
                        <TextField
                          fullWidth
                          label="Tags"
                          placeholder="Press Enter to add tags"
                          onKeyDown={handleTagInput}
                          helperText="Add tags to help categorize your project"
                        />
                        <Box
                          sx={{
                            mt: 1,
                            display: 'flex',
                            flexWrap: 'wrap',
                            gap: 1,
                          }}
                        >
                          {projectData.tags.map((tag) => (
                            <Chip
                              key={tag}
                              label={tag}
                              onDelete={() => removeTag(tag)}
                              size="small"
                            />
                          ))}
                        </Box>
                      </Grid>
                    </Grid>
                  )}

                  {/* Step 2: Module Selection */}
                  {index === 1 && (
                    <Box>
                      <Typography variant="h6" gutterBottom>
                        Select Modules
                      </Typography>
                      <Typography
                        variant="body2"
                        color="text.secondary"
                        sx={{ mb: 3 }}
                      >
                        Choose the modules you'll need for your project
                      </Typography>
                      {modulesError && (
                        <Alert severity="error" sx={{ mb: 2 }}>
                          {modulesError}
                        </Alert>
                      )}
                      <Grid container spacing={2}>
                        {(modulesLoading ? [] : modules).map((mod) => (
                          <Grid item xs={12} sm={6} md={4} key={mod.id}>
                            <Card
                              sx={{
                                cursor: 'pointer',
                                border: projectData.algorithms.includes(mod.id)
                                  ? 2
                                  : 1,
                                borderColor: projectData.algorithms.includes(
                                  mod.id
                                )
                                  ? 'primary.main'
                                  : 'divider',
                                '&:hover': {
                                  borderColor: 'primary.main',
                                },
                              }}
                              onClick={() => toggleAlgorithm(mod.id)}
                            >
                              <CardContent>
                                <Box
                                  sx={{
                                    display: 'flex',
                                    alignItems: 'center',
                                    mb: 1,
                                  }}
                                >
                                  <Checkbox
                                    checked={projectData.algorithms.includes(
                                      mod.id
                                    )}
                                    onChange={() => toggleAlgorithm(mod.id)}
                                    size="small"
                                  />
                                  <Typography variant="h6" component="h3">
                                    {mod.name}
                                  </Typography>
                                </Box>
                                <Chip
                                  label={mod.category}
                                  size="small"
                                  variant="outlined"
                                  sx={{ mb: 1 }}
                                />
                                <Typography
                                  variant="body2"
                                  color="text.secondary"
                                >
                                  {mod.description}
                                </Typography>
                              </CardContent>
                            </Card>
                          </Grid>
                        ))}
                      </Grid>
                    </Box>
                  )}

                  {/* Step 3: Environment Configuration */}
                  {index === 2 && (
                    <Box>
                      <Typography variant="h6" gutterBottom>
                        Configure Development Environment
                      </Typography>
                      <Grid container spacing={3}>
                        <Grid item xs={12} md={6}>
                          <FormControl fullWidth>
                            <InputLabel>Base Image</InputLabel>
                            <Select
                              value={projectData.environment.baseImage}
                              label="Base Image"
                              onChange={(e) =>
                                setProjectData((prev) => ({
                                  ...prev,
                                  environment: {
                                    ...prev.environment,
                                    baseImage: e.target.value,
                                  },
                                }))
                              }
                            >
                              {baseImages.map((image) => (
                                <MenuItem key={image.value} value={image.value}>
                                  {image.label}
                                </MenuItem>
                              ))}
                            </Select>
                          </FormControl>
                        </Grid>
                        <Grid item xs={12} md={6}>
                          <TextField
                            fullWidth
                            label="Python Version"
                            value={projectData.environment.pythonVersion}
                            onChange={(e) =>
                              setProjectData((prev) => ({
                                ...prev,
                                environment: {
                                  ...prev.environment,
                                  pythonVersion: e.target.value,
                                },
                              }))
                            }
                          />
                        </Grid>
                        <Grid item xs={12}>
                          <Typography variant="subtitle1" gutterBottom>
                            Environment Variables
                          </Typography>
                          <Box sx={{ mb: 2 }}>
                            {Object.entries(
                              projectData.environment.environmentVariables
                            ).map(([key, value]) => (
                              <Box
                                key={key}
                                sx={{ display: 'flex', gap: 1, mb: 1 }}
                              >
                                <TextField
                                  size="small"
                                  label="Key"
                                  value={key}
                                  onChange={(e) => {
                                    const newEnvVars = {
                                      ...projectData.environment
                                        .environmentVariables,
                                    };
                                    delete newEnvVars[key];
                                    newEnvVars[e.target.value] = value;
                                    setProjectData((prev) => ({
                                      ...prev,
                                      environment: {
                                        ...prev.environment,
                                        environmentVariables: newEnvVars,
                                      },
                                    }));
                                  }}
                                />
                                <TextField
                                  size="small"
                                  label="Value"
                                  value={value}
                                  onChange={(e) =>
                                    updateEnvironmentVariable(
                                      key,
                                      e.target.value
                                    )
                                  }
                                />
                                <IconButton
                                  size="small"
                                  onClick={() => removeEnvironmentVariable(key)}
                                  color="error"
                                >
                                  <RemoveIcon />
                                </IconButton>
                              </Box>
                            ))}
                            <Button
                              startIcon={<AddIcon />}
                              onClick={addEnvironmentVariable}
                              size="small"
                            >
                              Add Environment Variable
                            </Button>
                          </Box>
                        </Grid>
                      </Grid>
                    </Box>
                  )}

                  {/* Step 4: Project Settings */}
                  {index === 3 && (
                    <Box>
                      <Typography variant="h6" gutterBottom>
                        Advanced Project Settings
                      </Typography>
                      <Grid container spacing={3}>
                        <Grid item xs={12} md={6}>
                          <FormControlLabel
                            control={
                              <Switch
                                checked={projectData.settings.autoSave}
                                onChange={(e) =>
                                  setProjectData((prev) => ({
                                    ...prev,
                                    settings: {
                                      ...prev.settings,
                                      autoSave: e.target.checked,
                                    },
                                  }))
                                }
                              />
                            }
                            label="Auto-save changes"
                          />
                        </Grid>
                        <Grid item xs={12} md={6}>
                          <FormControlLabel
                            control={
                              <Switch
                                checked={projectData.settings.enableDebugging}
                                onChange={(e) =>
                                  setProjectData((prev) => ({
                                    ...prev,
                                    settings: {
                                      ...prev.settings,
                                      enableDebugging: e.target.checked,
                                    },
                                  }))
                                }
                              />
                            }
                            label="Enable debugging"
                          />
                        </Grid>
                        <Grid item xs={12} md={6}>
                          <Typography gutterBottom>
                            Memory Limit (MB)
                          </Typography>
                          <Slider
                            value={projectData.settings.maxMemory}
                            onChange={(_, value) =>
                              setProjectData((prev) => ({
                                ...prev,
                                settings: {
                                  ...prev.settings,
                                  maxMemory: value as number,
                                },
                              }))
                            }
                            min={512}
                            max={8192}
                            step={512}
                            marks
                            valueLabelDisplay="auto"
                          />
                        </Grid>
                        <Grid item xs={12} md={6}>
                          <Typography gutterBottom>
                            CPU Limit (cores)
                          </Typography>
                          <Slider
                            value={projectData.settings.cpuLimit}
                            onChange={(_, value) =>
                              setProjectData((prev) => ({
                                ...prev,
                                settings: {
                                  ...prev.settings,
                                  cpuLimit: value as number,
                                },
                              }))
                            }
                            min={1}
                            max={8}
                            step={1}
                            marks
                            valueLabelDisplay="auto"
                          />
                        </Grid>
                        <Grid item xs={12}>
                          <FormControl component="fieldset">
                            <FormLabel component="legend">
                              Backup Frequency
                            </FormLabel>
                            <RadioGroup
                              value={projectData.settings.backupFrequency}
                              onChange={(e) =>
                                setProjectData((prev) => ({
                                  ...prev,
                                  settings: {
                                    ...prev.settings,
                                    backupFrequency: e.target.value as any,
                                  },
                                }))
                              }
                            >
                              <FormControlLabel
                                value="daily"
                                control={<Radio />}
                                label="Daily"
                              />
                              <FormControlLabel
                                value="weekly"
                                control={<Radio />}
                                label="Weekly"
                              />
                              <FormControlLabel
                                value="monthly"
                                control={<Radio />}
                                label="Monthly"
                              />
                              <FormControlLabel
                                value="never"
                                control={<Radio />}
                                label="Never"
                              />
                            </RadioGroup>
                          </FormControl>
                        </Grid>
                      </Grid>
                    </Box>
                  )}

                  {/* Step 5: Review */}
                  {index === 4 && (
                    <Box>
                      <Typography variant="h6" gutterBottom>
                        Review Project Configuration
                      </Typography>
                      <Grid container spacing={3}>
                        <Grid item xs={12} md={6}>
                          <Card>
                            <CardContent>
                              <Typography variant="h6" gutterBottom>
                                Project Details
                              </Typography>
                              <List dense>
                                <ListItem>
                                  <ListItemText
                                    primary="Name"
                                    secondary={projectData.name}
                                  />
                                </ListItem>

                                <ListItem>
                                  <ListItemText
                                    primary="Tags"
                                    secondary={
                                      projectData.tags.join(', ') || 'None'
                                    }
                                  />
                                </ListItem>
                              </List>
                            </CardContent>
                          </Card>
                        </Grid>
                        <Grid item xs={12} md={6}>
                          <Card>
                            <CardContent>
                              <Typography variant="h6" gutterBottom>
                                Selected Modules
                              </Typography>
                              <List dense>
                                {projectData.algorithms.map((moduleId) => {
                                  const mod = modules.find((m) => m.id === moduleId);
                                  return (
                                    <ListItem key={moduleId}>
                                      <ListItemIcon>
                                        <CheckIcon color="primary" />
                                      </ListItemIcon>
                                      <ListItemText
                                        primary={mod?.name || moduleId}
                                        secondary={mod?.category}
                                      />
                                    </ListItem>
                                  );
                                })}
                              </List>
                            </CardContent>
                          </Card>
                        </Grid>
                        <Grid item xs={12}>
                          <Card>
                            <CardContent>
                              <Typography variant="h6" gutterBottom>
                                Environment Configuration
                              </Typography>
                              <Grid container spacing={2}>
                                <Grid item xs={12} md={4}>
                                  <Typography variant="subtitle2">
                                    Base Image
                                  </Typography>
                                  <Typography variant="body2">
                                    {projectData.environment.baseImage}
                                  </Typography>
                                </Grid>
                                <Grid item xs={12} md={4}>
                                  <Typography variant="subtitle2">
                                    Python Version
                                  </Typography>
                                  <Typography variant="body2">
                                    {projectData.environment.pythonVersion}
                                  </Typography>
                                </Grid>
                                <Grid item xs={12} md={4}>
                                  <Typography variant="subtitle2">
                                    Environment Variables
                                  </Typography>
                                  <Typography variant="body2">
                                    {
                                      Object.keys(
                                        projectData.environment
                                          .environmentVariables
                                      ).length
                                    }{' '}
                                    variables
                                  </Typography>
                                </Grid>
                              </Grid>
                            </CardContent>
                          </Card>
                        </Grid>
                      </Grid>
                    </Box>
                  )}
                </Box>

                {/* Step Actions */}
                <Box
                  sx={{
                    mt: 3,
                    display: 'flex',
                    justifyContent: 'space-between',
                  }}
                >
                  <Button
                    disabled={activeStep === 0}
                    onClick={handleBack}
                    startIcon={<ArrowBackIcon />}
                  >
                    Back
                  </Button>
                  <Box>
                    {activeStep === steps.length - 1 ? (
                      <Button
                        variant="contained"
                        onClick={handleCreateProject}
                        disabled={loading || !validateStep(activeStep)}
                        startIcon={
                          loading ? (
                            <CircularProgress size={20} />
                          ) : (
                            <SaveIcon />
                          )
                        }
                      >
                        {loading ? 'Creating Project...' : 'Create Project'}
                      </Button>
                    ) : (
                      <Button
                        variant="contained"
                        onClick={handleNext}
                        disabled={!validateStep(activeStep)}
                        endIcon={<ArrowForwardIcon />}
                      >
                        Next
                      </Button>
                    )}
                  </Box>
                </Box>
              </StepContent>
            </Step>
          ))}
        </Stepper>
      </Paper>
    </Box>
  );
};

export default ProjectCreationWizard;
