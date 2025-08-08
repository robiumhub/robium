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
  Alert,
  CircularProgress,
  List,
  ListItem,
  ListItemText,
  ListItemIcon,
  FormControlLabel,
} from '@mui/material';
import { Grid } from '@mui/material';
import {
  Check as CheckIcon,
  Save as SaveIcon,
  ArrowBack as ArrowBackIcon,
  ArrowForward as ArrowForwardIcon,
  Folder as FolderIcon,
  Code as CodeIcon,
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
  robot: string;
  hardware: string;
  is_template?: boolean;
}

// Robot and hardware options
const robotOptions = [
  { value: 'turtlebot3', label: 'TurtleBot 3' },
  { value: 'turtlebot4', label: 'TurtleBot 4' },
];
const hardwareOptions = [
  { value: 'raspberrypi', label: 'Raspberry Pi' },
  { value: 'nvidia-orin', label: 'NVIDIA Orin' },
];

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
    robot: '',
    hardware: '',
    is_template: false,
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
  };

  const validation = useFormValidation(projectData, fieldValidations, {
    debounceDelay: 500,
    validateOnChange: true,
  });

  // No environment config for now

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
        label: 'Module Selection',
        description: 'Choose modules for your project',
        icon: <CodeIcon />,
        completed: false,
        valid: false,
      },
      {
        label: 'Robot Configuration',
        description: 'Select robot and hardware',
        icon: <RobotIcon />,
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
          return (
            projectData.robot.length > 0 && projectData.hardware.length > 0
          );
        case 3:
          return true; // Review step

        default:
          return false;
      }
    },
    [projectData]
  );

  // Step validation state (separate from steps array to prevent mutations)
  const [stepValidations, setStepValidations] = useState<boolean[]>(() =>
    new Array(steps.length).fill(false)
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
        robot: projectData.robot,
        hardware: projectData.hardware,
        is_template: projectData.is_template === true,
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
      setProjectData((prev) => ({
        ...prev,
        name: suggestion.name,
        description: suggestion.description,
        tags: suggestion.tags,
        algorithms: suggestion.algorithms,
      }));
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

  // Environment helpers removed

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

                      <Grid item xs={12}>
                        <FormControlLabel
                          control={
                            <Checkbox
                              checked={Boolean(projectData.is_template)}
                              onChange={(e) =>
                                setProjectData((prev) => ({
                                  ...prev,
                                  is_template: e.target.checked,
                                }))
                              }
                            />
                          }
                          label="Save this project as a template"
                        />
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

                  {/* Step 3: Robot Configuration */}
                  {index === 2 && (
                    <Box>
                      <Typography variant="h6" gutterBottom>
                        Robot Configuration
                      </Typography>
                      <Grid container spacing={3}>
                        <Grid item xs={12} md={6}>
                          <FormControl fullWidth>
                            <InputLabel>Robot</InputLabel>
                            <Select
                              value={projectData.robot}
                              label="Robot"
                              onChange={(e) =>
                                setProjectData((prev) => ({
                                  ...prev,
                                  robot: e.target.value,
                                }))
                              }
                            >
                              {robotOptions.map((r) => (
                                <MenuItem key={r.value} value={r.value}>
                                  {r.label}
                                </MenuItem>
                              ))}
                            </Select>
                          </FormControl>
                        </Grid>
                        <Grid item xs={12} md={6}>
                          <FormControl fullWidth>
                            <InputLabel>Hardware</InputLabel>
                            <Select
                              value={projectData.hardware}
                              label="Hardware"
                              onChange={(e) =>
                                setProjectData((prev) => ({
                                  ...prev,
                                  hardware: e.target.value,
                                }))
                              }
                            >
                              {hardwareOptions.map((h) => (
                                <MenuItem key={h.value} value={h.value}>
                                  {h.label}
                                </MenuItem>
                              ))}
                            </Select>
                          </FormControl>
                        </Grid>
                      </Grid>
                    </Box>
                  )}

                  {/* Step 4 removed (Project Settings) */}

                  {/* Step 4: Review */}
                  {index === 3 && (
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
                                <ListItem>
                                  <ListItemText
                                    primary="Template"
                                    secondary={
                                      projectData.is_template ? 'Yes' : 'No'
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
                                  const mod = modules.find(
                                    (m) => m.id === moduleId
                                  );
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
                        {/* Environment review removed */}
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
