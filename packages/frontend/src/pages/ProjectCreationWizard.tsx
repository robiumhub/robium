import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
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
  Checkbox,
  FormControlLabel,
  Alert,
  CircularProgress,
  Chip,
} from '@mui/material';
import { Grid } from '@mui/material';
import {
  ArrowBack as ArrowBackIcon,
  ArrowForward as ArrowForwardIcon,
  Folder as FolderIcon,
  Settings as SettingsIcon,
  Preview as PreviewIcon,
} from '@mui/icons-material';
import { ApiService } from '../services/api';
import {
  NewProjectForm,
  defaultNewProjectForm,
  validateConstraints,
} from '../types/project';

interface WizardStep {
  label: string;
  description: string;
  icon: React.ReactNode;
  completed: boolean;
  valid: boolean;
}

const ProjectCreationWizard: React.FC = () => {
  const navigate = useNavigate();
  const [activeStep, setActiveStep] = useState(0);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [showErrors, setShowErrors] = useState(false);
  const [form, setForm] = useState<NewProjectForm>(defaultNewProjectForm);

  const validationErrors = validateConstraints(form);
  const hasValidationErrors = validationErrors.length > 0;

  const steps: WizardStep[] = [
    {
      label: 'Project Info',
      description: 'Basic project information',
      icon: <FolderIcon />,
      completed: !!form.name.trim(),
      valid: !!form.name.trim(),
    },
    {
      label: 'Configuration',
      description: 'Robot and environment configuration',
      icon: <SettingsIcon />,
      completed: !hasValidationErrors && activeStep > 1,
      valid: !hasValidationErrors,
    },
    {
      label: 'Summary',
      description: 'Review and create project',
      icon: <PreviewIcon />,
      completed: false,
      valid: !hasValidationErrors && !!form.name.trim(),
    },
  ];

  const handleNext = () => {
    if (activeStep === steps.length - 1) {
      handleCreateProject();
    } else {
      setActiveStep((prevActiveStep) => prevActiveStep + 1);
    }
  };

  const handleBack = () => {
    setActiveStep((prevActiveStep) => prevActiveStep - 1);
  };

  const handleCreateProject = async () => {
    if (hasValidationErrors) {
      setShowErrors(true);
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const payload = {
        name: form.name,
        description: form.description,
        config: {
          name: form.name,
          robotTarget: form.robotTarget,
          simulation: form.simulation,
          rmw: form.rmw,
          execution: form.execution,
          deployment: form.deployment,
          baseImage: form.baseImage,
          rosVersion: form.rosVersion,
          foxglove: form.foxglove,
          teleopKeyboard: form.teleopKeyboard,
          teleopJoystick: form.teleopJoystick,
        },
      };

      const project = await ApiService.createProject(payload);

      // Generate Dockerfile
      await ApiService.generateDockerfile(project.id);

      navigate(`/projects/${project.id}`);
    } catch (err: any) {
      setError(err.message || 'Failed to create project');
    } finally {
      setLoading(false);
    }
  };

  const updateForm = (field: keyof NewProjectForm, value: any) => {
    setForm((prev) => ({ ...prev, [field]: value }));
  };

  const getBaseImageLabel = (image: string) => {
    switch (image) {
      case 'ros_humble':
        return 'ROS Humble';
      case 'cuda_ubuntu2204':
        return 'CUDA Ubuntu 22.04';
      case 'jetson_l4t_ros':
        return 'Jetson L4T ROS';
      default:
        return image;
    }
  };

  const getRuntimeFlags = () => {
    const flags = [];
    if (form.simulation === 'isaac' || form.deployment === 'cloud_gpu') {
      flags.push('--gpus all');
    }
    return flags;
  };

  const getExposedPorts = () => {
    const ports = [];
    if (form.foxglove) {
      ports.push('9090 (Foxglove)');
    }
    return ports;
  };

  return (
    <Box sx={{ maxWidth: 1200, mx: 'auto', p: 3 }}>
      <Typography variant="h4" gutterBottom>
        Create New Project
      </Typography>

      {error && (
        <Alert severity="error" sx={{ mb: 2 }}>
          {error}
        </Alert>
      )}

      <Stepper activeStep={activeStep} orientation="vertical">
        {steps.map((step, index) => (
          <Step key={step.label}>
            <StepLabel
              icon={step.icon}
              optional={
                index === 2 ? (
                  <Typography variant="caption">Final step</Typography>
                ) : null
              }
            >
              {step.label}
            </StepLabel>
            <StepContent>
              <Box sx={{ mb: 2 }}>
                <Typography>{step.description}</Typography>

                {index === 0 && (
                  <ProjectInfoSection form={form} updateForm={updateForm} />
                )}

                {index === 1 && (
                  <ConfigurationSection
                    form={form}
                    updateForm={updateForm}
                    showErrors={showErrors}
                    validationErrors={validationErrors}
                  />
                )}

                {index === 2 && <SummarySection form={form} />}

                <Box sx={{ mt: 2 }}>
                  <Button
                    variant="contained"
                    onClick={handleNext}
                    sx={{ mr: 1 }}
                    disabled={loading || !step.valid}
                  >
                    {loading ? (
                      <CircularProgress size={20} sx={{ mr: 1 }} />
                    ) : (
                      <ArrowForwardIcon sx={{ mr: 1 }} />
                    )}
                    {index === steps.length - 1 ? 'Create Project' : 'Continue'}
                  </Button>
                  <Button
                    disabled={index === 0}
                    onClick={handleBack}
                    sx={{ mr: 1 }}
                  >
                    <ArrowBackIcon sx={{ mr: 1 }} />
                    Back
                  </Button>
                </Box>
              </Box>
            </StepContent>
          </Step>
        ))}
      </Stepper>
    </Box>
  );
};

// Project Info Section
const ProjectInfoSection: React.FC<{
  form: NewProjectForm;
  updateForm: (field: keyof NewProjectForm, value: any) => void;
}> = ({ form, updateForm }) => (
  <Box sx={{ mt: 2 }}>
    <TextField
      fullWidth
      label="Project Name"
      value={form.name}
      onChange={(e) => updateForm('name', e.target.value)}
      required
      sx={{ mb: 2 }}
    />
    <TextField
      fullWidth
      label="Description"
      value={form.description}
      onChange={(e) => updateForm('description', e.target.value)}
      multiline
      rows={3}
      sx={{ mb: 2 }}
    />
  </Box>
);

// Configuration Section
const ConfigurationSection: React.FC<{
  form: NewProjectForm;
  updateForm: (field: keyof NewProjectForm, value: any) => void;
  showErrors: boolean;
  validationErrors: string[];
}> = ({ form, updateForm, showErrors, validationErrors }) => (
  <Box sx={{ mt: 2 }}>
    {showErrors && validationErrors.length > 0 && (
      <Alert severity="error" sx={{ mb: 2 }}>
        <Typography variant="subtitle2" gutterBottom>
          Configuration Issues:
        </Typography>
        <ul style={{ margin: 0, paddingLeft: 20 }}>
          {validationErrors.map((error, index) => (
            <li key={index}>{error}</li>
          ))}
        </ul>
      </Alert>
    )}

    <Grid container spacing={3}>
      <Grid item xs={12} md={6}>
        <FormControl fullWidth sx={{ mb: 2 }}>
          <InputLabel>Robot Target</InputLabel>
          <Select
            value={form.robotTarget}
            onChange={(e) => updateForm('robotTarget', e.target.value)}
            label="Robot Target"
          >
            <MenuItem value="none">None</MenuItem>
            <MenuItem value="turtlebot4">TurtleBot 4</MenuItem>
            <MenuItem value="ur5">UR5</MenuItem>
          </Select>
        </FormControl>

        <FormControl fullWidth sx={{ mb: 2 }}>
          <InputLabel>Simulation</InputLabel>
          <Select
            value={form.simulation}
            onChange={(e) => updateForm('simulation', e.target.value)}
            label="Simulation"
          >
            <MenuItem value="none">None</MenuItem>
            <MenuItem value="gazebo">Gazebo</MenuItem>
            <MenuItem value="isaac">Isaac</MenuItem>
          </Select>
        </FormControl>

        <FormControl fullWidth sx={{ mb: 2 }}>
          <InputLabel>RMW</InputLabel>
          <Select
            value={form.rmw}
            onChange={(e) => updateForm('rmw', e.target.value)}
            label="RMW"
          >
            <MenuItem value="cyclonedds">CycloneDDS</MenuItem>
            <MenuItem value="fastrtps">FastRTPS</MenuItem>
          </Select>
        </FormControl>

        <FormControl fullWidth sx={{ mb: 2 }}>
          <InputLabel>Execution Mode</InputLabel>
          <Select
            value={form.execution}
            onChange={(e) => updateForm('execution', e.target.value)}
            label="Execution Mode"
          >
            <MenuItem value="simulator">Simulator</MenuItem>
            <MenuItem value="real">Real</MenuItem>
            <MenuItem value="hybrid">Hybrid</MenuItem>
          </Select>
        </FormControl>
      </Grid>

      <Grid item xs={12} md={6}>
        <FormControl fullWidth sx={{ mb: 2 }}>
          <InputLabel>Deployment</InputLabel>
          <Select
            value={form.deployment}
            onChange={(e) => updateForm('deployment', e.target.value)}
            label="Deployment"
          >
            <MenuItem value="local">Local</MenuItem>
            <MenuItem value="cloud_gpu">Cloud GPU</MenuItem>
            <MenuItem value="edge">Edge</MenuItem>
          </Select>
        </FormControl>

        <FormControl fullWidth sx={{ mb: 2 }}>
          <InputLabel>Base Image</InputLabel>
          <Select
            value={form.baseImage}
            onChange={(e) => updateForm('baseImage', e.target.value)}
            label="Base Image"
          >
            <MenuItem value="ros_humble">ROS Humble</MenuItem>
            <MenuItem value="cuda_ubuntu2204">CUDA Ubuntu 22.04</MenuItem>
            <MenuItem value="jetson_l4t_ros">Jetson L4T ROS</MenuItem>
          </Select>
        </FormControl>

        <FormControl fullWidth sx={{ mb: 2 }}>
          <InputLabel>ROS Version</InputLabel>
          <Select
            value={form.rosVersion}
            onChange={(e) => updateForm('rosVersion', e.target.value)}
            label="ROS Version"
          >
            <MenuItem value="humble">Humble</MenuItem>
            <MenuItem value="iron">Iron</MenuItem>
            <MenuItem value="jazzy">Jazzy</MenuItem>
          </Select>
        </FormControl>

        <Box sx={{ mt: 2 }}>
          <FormControlLabel
            control={
              <Checkbox
                checked={form.foxglove}
                onChange={(e) => updateForm('foxglove', e.target.checked)}
              />
            }
            label="Foxglove Studio"
          />
          <FormControlLabel
            control={
              <Checkbox
                checked={form.teleopKeyboard}
                onChange={(e) => updateForm('teleopKeyboard', e.target.checked)}
              />
            }
            label="Keyboard Teleop"
          />
          <FormControlLabel
            control={
              <Checkbox
                checked={form.teleopJoystick}
                onChange={(e) => updateForm('teleopJoystick', e.target.checked)}
              />
            }
            label="Joystick Teleop"
          />
        </Box>
      </Grid>
    </Grid>
  </Box>
);

// Summary Section
const SummarySection: React.FC<{ form: NewProjectForm }> = ({ form }) => {
  const runtimeFlags =
    form.simulation === 'isaac' || form.deployment === 'cloud_gpu'
      ? ['--gpus all']
      : [];
  const exposedPorts = form.foxglove ? ['9090 (Foxglove)'] : [];

  const getBaseImageLabel = (image: string) => {
    switch (image) {
      case 'ros_humble':
        return 'ROS Humble';
      case 'cuda_ubuntu2204':
        return 'CUDA Ubuntu 22.04';
      case 'jetson_l4t_ros':
        return 'Jetson L4T ROS';
      default:
        return image;
    }
  };

  return (
    <Box sx={{ mt: 2 }}>
      <Card>
        <CardContent>
          <Typography variant="h6" gutterBottom>
            Project Summary
          </Typography>

          <Grid container spacing={2}>
            <Grid item xs={12} md={6}>
              <Typography variant="subtitle2" color="text.secondary">
                Project Details
              </Typography>
              <Typography variant="body2" sx={{ mb: 1 }}>
                <strong>Name:</strong> {form.name}
              </Typography>
              {form.description && (
                <Typography variant="body2" sx={{ mb: 1 }}>
                  <strong>Description:</strong> {form.description}
                </Typography>
              )}

              <Typography
                variant="subtitle2"
                color="text.secondary"
                sx={{ mt: 2 }}
              >
                Configuration
              </Typography>
              <Typography variant="body2" sx={{ mb: 1 }}>
                <strong>Robot:</strong> {form.robotTarget}
              </Typography>
              <Typography variant="body2" sx={{ mb: 1 }}>
                <strong>Simulation:</strong> {form.simulation}
              </Typography>
              <Typography variant="body2" sx={{ mb: 1 }}>
                <strong>RMW:</strong> {form.rmw}
              </Typography>
              <Typography variant="body2" sx={{ mb: 1 }}>
                <strong>Execution:</strong> {form.execution}
              </Typography>
              <Typography variant="body2" sx={{ mb: 1 }}>
                <strong>Deployment:</strong> {form.deployment}
              </Typography>
            </Grid>

            <Grid item xs={12} md={6}>
              <Typography variant="subtitle2" color="text.secondary">
                Technical Details
              </Typography>
              <Typography variant="body2" sx={{ mb: 1 }}>
                <strong>Base Image:</strong> {getBaseImageLabel(form.baseImage)}
              </Typography>
              <Typography variant="body2" sx={{ mb: 1 }}>
                <strong>ROS Version:</strong> {form.rosVersion}
              </Typography>

              {runtimeFlags.length > 0 && (
                <>
                  <Typography
                    variant="subtitle2"
                    color="text.secondary"
                    sx={{ mt: 2 }}
                  >
                    Runtime Flags
                  </Typography>
                  {runtimeFlags.map((flag, index) => (
                    <Chip
                      key={index}
                      label={flag}
                      size="small"
                      sx={{ mr: 1, mb: 1 }}
                    />
                  ))}
                </>
              )}

              {exposedPorts.length > 0 && (
                <>
                  <Typography
                    variant="subtitle2"
                    color="text.secondary"
                    sx={{ mt: 2 }}
                  >
                    Exposed Ports
                  </Typography>
                  {exposedPorts.map((port, index) => (
                    <Chip
                      key={index}
                      label={port}
                      size="small"
                      sx={{ mr: 1, mb: 1 }}
                    />
                  ))}
                </>
              )}

              {(form.foxglove ||
                form.teleopKeyboard ||
                form.teleopJoystick) && (
                <>
                  <Typography
                    variant="subtitle2"
                    color="text.secondary"
                    sx={{ mt: 2 }}
                  >
                    Features
                  </Typography>
                  {form.foxglove && (
                    <Chip label="Foxglove" size="small" sx={{ mr: 1, mb: 1 }} />
                  )}
                  {form.teleopKeyboard && (
                    <Chip
                      label="Keyboard Teleop"
                      size="small"
                      sx={{ mr: 1, mb: 1 }}
                    />
                  )}
                  {form.teleopJoystick && (
                    <Chip
                      label="Joystick Teleop"
                      size="small"
                      sx={{ mr: 1, mb: 1 }}
                    />
                  )}
                </>
              )}
            </Grid>
          </Grid>
        </CardContent>
      </Card>
    </Box>
  );
};

export default ProjectCreationWizard;
