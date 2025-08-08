import React, { useState, useEffect } from 'react';
import {
  Box,
  Typography,
  Grid,
  Card,
  CardContent,
  CardActions,
  Button,
  Chip,
  Avatar,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  TextField,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Alert,
  CircularProgress,
  Tooltip,
  IconButton,
} from '@mui/material';
import {
  ContentCopy as CloneIcon,
  Visibility as ViewIcon,
  Delete as DeleteIcon,
  Code as CodeIcon,
  SmartToy as RobotIcon,
  Navigation as NavigationIcon,
  CameraAlt as VisionIcon,
  Build as ManipulationIcon,
  Psychology as AIIcon,
} from '@mui/icons-material';
import { useNavigate } from 'react-router-dom';
import { ApiService } from '../services/api';

interface TemplateProject {
  id: string;
  name: string;
  description: string;
  tags: string[];
  module_count: number;
  package_count: number;
  created_at: string;
}

const Templates: React.FC = () => {
  const [templates, setTemplates] = useState<TemplateProject[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [selectedTemplate, setSelectedTemplate] =
    useState<TemplateProject | null>(null);
  const [cloneDialogOpen, setCloneDialogOpen] = useState(false);
  const [newProjectName, setNewProjectName] = useState('');
  const [cloning, setCloning] = useState(false);
  const navigate = useNavigate();

  // Load templates from backend

  useEffect(() => {
    const loadTemplates = async () => {
      try {
        setLoading(true);
        const data = await ApiService.get<any[]>('/projects/templates');
        setTemplates(Array.isArray(data) ? data : []);
      } catch (err) {
        setError('Failed to load templates');
        console.error('Error loading templates:', err);
      } finally {
        setLoading(false);
      }
    };

    loadTemplates();
  }, []);

  const handleCloneTemplate = (template: TemplateProject) => {
    setSelectedTemplate(template);
    setNewProjectName(`${template.name} - Copy`);
    setCloneDialogOpen(true);
  };

  const handleCloneConfirm = async () => {
    if (!selectedTemplate || !newProjectName.trim()) return;

    try {
      setCloning(true);
      // In real app: await ApiService.post('/projects/clone', { templateId: selectedTemplate.id, name: newProjectName });

      // Simulate API call
      await new Promise((resolve) => setTimeout(resolve, 2000));

      setCloneDialogOpen(false);
      setSelectedTemplate(null);
      setNewProjectName('');

      // Navigate to the new project
      // In real app, you'd get the new project ID from the API response
      navigate('/projects');
    } catch (err) {
      setError('Failed to clone template');
      console.error('Error cloning template:', err);
    } finally {
      setCloning(false);
    }
  };

  const getCategoryIcon = (category: string) => {
    switch (category.toLowerCase()) {
      case 'navigation':
        return <NavigationIcon />;
      case 'vision':
        return <VisionIcon />;
      case 'manipulation':
        return <ManipulationIcon />;
      case 'ai':
        return <AIIcon />;
      default:
        return <RobotIcon />;
    }
  };

  const getDifficultyColor = (difficulty: string) => {
    switch (difficulty) {
      case 'beginner':
        return 'success';
      case 'intermediate':
        return 'warning';
      case 'advanced':
        return 'error';
      default:
        return 'default';
    }
  };

  if (loading) {
    return (
      <Box
        display="flex"
        justifyContent="center"
        alignItems="center"
        minHeight="400px"
      >
        <CircularProgress />
      </Box>
    );
  }

  if (error) {
    return (
      <Box
        display="flex"
        justifyContent="center"
        alignItems="center"
        minHeight="400px"
      >
        <Alert severity="error">{error}</Alert>
      </Box>
    );
  }

  return (
    <Box>
      <Typography variant="h4" gutterBottom>
        Project Templates
      </Typography>
      <Typography variant="body1" color="text.secondary" sx={{ mb: 3 }}>
        Templates are projects created by Robium developers. Clone to start.
      </Typography>

      <Grid container spacing={3}>
        {templates.map((template) => (
          <Grid item xs={12} sm={6} md={4} key={template.id}>
            <Card
              sx={{
                height: '100%',
                display: 'flex',
                flexDirection: 'column',
              }}
            >
              <CardContent sx={{ flexGrow: 1 }}>
                <Box display="flex" alignItems="center" mb={2}>
                  <Avatar sx={{ mr: 1, bgcolor: 'primary.main' }}>
                    {template.name.charAt(0)}
                  </Avatar>
                  <Typography variant="h6" component="h2">
                    {template.name}
                  </Typography>
                </Box>

                <Typography
                  variant="body2"
                  color="text.secondary"
                  sx={{ mb: 2 }}
                >
                  {template.description}
                </Typography>

                <Box sx={{ mb: 2 }}>
                  {template.tags?.map((tag) => (
                    <Chip
                      key={tag}
                      label={tag}
                      size="small"
                      variant="outlined"
                      sx={{ mr: 0.5, mb: 0.5 }}
                    />
                  ))}
                </Box>

                <Typography variant="caption" color="text.secondary">
                  {new Date(template.created_at).toLocaleDateString()} •{' '}
                  {template.module_count} modules
                </Typography>
              </CardContent>

              <CardActions sx={{ justifyContent: 'space-between', p: 2 }}>
                <Tooltip title="Delete template">
                  <IconButton
                    size="small"
                    color="error"
                    onClick={async () => {
                      try {
                        await ApiService.delete(`/projects/${template.id}`);
                        setTemplates((prev) => prev.filter((t) => t.id !== template.id));
                      } catch (e) {
                        console.error('Failed to delete template', e);
                      }
                    }}
                  >
                    <DeleteIcon />
                  </IconButton>
                </Tooltip>
                <Button
                  variant="contained"
                  startIcon={<CloneIcon />}
                  onClick={() => handleCloneTemplate(template)}
                  size="small"
                >
                  Clone Template
                </Button>
              </CardActions>
            </Card>
          </Grid>
        ))}
      </Grid>

      {/* Clone Dialog */}
      <Dialog
        open={cloneDialogOpen}
        onClose={() => setCloneDialogOpen(false)}
        maxWidth="sm"
        fullWidth
      >
        <DialogTitle>Clone Template</DialogTitle>
        <DialogContent>
          {selectedTemplate && (
            <Box>
              <Typography variant="body1" sx={{ mb: 2 }}>
                You're about to clone the{' '}
                <strong>{selectedTemplate.name}</strong> template.
              </Typography>
              <TextField
                fullWidth
                label="Project Name"
                value={newProjectName}
                onChange={(e) => setNewProjectName(e.target.value)}
                sx={{ mb: 2 }}
              />
              <Typography variant="body2" color="text.secondary">
                This will create a new project with all the template files and
                configurations.
              </Typography>
            </Box>
          )}
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setCloneDialogOpen(false)} disabled={cloning}>
            Cancel
          </Button>
          <Button
            onClick={handleCloneConfirm}
            variant="contained"
            disabled={!newProjectName.trim() || cloning}
            startIcon={cloning ? <CircularProgress size={16} /> : <CloneIcon />}
          >
            {cloning ? 'Cloning...' : 'Clone Project'}
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default Templates;
