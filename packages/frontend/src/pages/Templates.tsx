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
  Code as CodeIcon,
  SmartToy as RobotIcon,
  Navigation as NavigationIcon,
  CameraAlt as VisionIcon,
  Build as ManipulationIcon,
  Psychology as AIIcon,
} from '@mui/icons-material';
import { useNavigate } from 'react-router-dom';
import { ApiService } from '../services/api';

interface Template {
  id: string;
  name: string;
  description: string;
  category: string;
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  tags: string[];
  imageUrl?: string;
  features: string[];
  estimatedTime: string;
  requirements: string[];
}

const Templates: React.FC = () => {
  const [templates, setTemplates] = useState<Template[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [selectedTemplate, setSelectedTemplate] = useState<Template | null>(
    null
  );
  const [cloneDialogOpen, setCloneDialogOpen] = useState(false);
  const [newProjectName, setNewProjectName] = useState('');
  const [cloning, setCloning] = useState(false);
  const navigate = useNavigate();

  // Mock templates data - in real app, this would come from API
  const mockTemplates: Template[] = [
    {
      id: '1',
      name: 'Autonomous Navigation Robot',
      description:
        'A complete ROS2-based autonomous navigation system with SLAM, path planning, and obstacle avoidance.',
      category: 'Navigation',
      difficulty: 'intermediate',
      tags: ['ROS2', 'SLAM', 'Navigation', 'Lidar'],
      features: [
        'SLAM mapping',
        'Path planning',
        'Obstacle avoidance',
        'Multi-sensor fusion',
      ],
      estimatedTime: '2-3 weeks',
      requirements: ['ROS2 Humble', 'Lidar sensor', 'IMU sensor', 'Camera'],
    },
    {
      id: '2',
      name: 'Computer Vision Robot',
      description:
        'Robot with advanced computer vision capabilities for object detection, recognition, and tracking.',
      category: 'Vision',
      difficulty: 'advanced',
      tags: ['Computer Vision', 'OpenCV', 'YOLO', 'TensorFlow'],
      features: [
        'Object detection',
        'Face recognition',
        'Gesture control',
        'Real-time tracking',
      ],
      estimatedTime: '3-4 weeks',
      requirements: [
        'High-resolution camera',
        'GPU support',
        'OpenCV',
        'Deep learning framework',
      ],
    },
    {
      id: '3',
      name: 'Manipulation Robot',
      description:
        'Robotic arm with pick-and-place capabilities, trajectory planning, and force control.',
      category: 'Manipulation',
      difficulty: 'advanced',
      tags: ['Robotic Arm', 'MoveIt', 'Trajectory Planning', 'Force Control'],
      features: [
        'Pick and place',
        'Trajectory optimization',
        'Force feedback',
        'Gripper control',
      ],
      estimatedTime: '4-5 weeks',
      requirements: [
        'Robotic arm',
        'MoveIt framework',
        'Force sensors',
        'Gripper',
      ],
    },
    {
      id: '4',
      name: 'AI Assistant Robot',
      description:
        'Intelligent robot assistant with natural language processing and conversational AI.',
      category: 'AI',
      difficulty: 'intermediate',
      tags: ['NLP', 'ChatGPT', 'Speech Recognition', 'Conversational AI'],
      features: [
        'Voice commands',
        'Natural conversations',
        'Task automation',
        'Learning capabilities',
      ],
      estimatedTime: '2-3 weeks',
      requirements: [
        'Microphone array',
        'Speaker',
        'OpenAI API',
        'Speech recognition',
      ],
    },
    {
      id: '5',
      name: 'Basic Mobile Robot',
      description:
        'Simple mobile robot with basic movement control and sensor integration.',
      category: 'Basic',
      difficulty: 'beginner',
      tags: ['Basic Movement', 'Sensors', 'Arduino', 'Simple Control'],
      features: [
        'Forward/backward movement',
        'Turning',
        'Basic obstacle detection',
        'Remote control',
      ],
      estimatedTime: '1-2 weeks',
      requirements: [
        'Arduino board',
        'Motor drivers',
        'Ultrasonic sensors',
        'Battery pack',
      ],
    },
    {
      id: '6',
      name: 'Multi-Modal Robot',
      description:
        'Advanced robot combining navigation, vision, and manipulation capabilities.',
      category: 'Advanced',
      difficulty: 'advanced',
      tags: ['Multi-Modal', 'Integration', 'Advanced Control', 'System Design'],
      features: [
        'Integrated navigation',
        'Vision-based manipulation',
        'Multi-task coordination',
        'Advanced planning',
      ],
      estimatedTime: '6-8 weeks',
      requirements: [
        'Complete robot platform',
        'Multiple sensors',
        'High-end computing',
        'Advanced software stack',
      ],
    },
  ];

  useEffect(() => {
    // Simulate API call
    const loadTemplates = async () => {
      try {
        setLoading(true);
        // In real app: const response = await ApiService.get('/templates');
        // Simulate API delay
        await new Promise((resolve) => setTimeout(resolve, 1000));
        setTemplates(mockTemplates);
      } catch (err) {
        setError('Failed to load templates');
        console.error('Error loading templates:', err);
      } finally {
        setLoading(false);
      }
    };

    loadTemplates();
  }, []);

  const handleCloneTemplate = (template: Template) => {
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
        Choose from our collection of pre-built project templates to jumpstart
        your robotics development.
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
                    {getCategoryIcon(template.category)}
                  </Avatar>
                  <Box>
                    <Typography variant="h6" component="h2">
                      {template.name}
                    </Typography>
                    <Chip
                      label={template.difficulty}
                      color={getDifficultyColor(template.difficulty) as any}
                      size="small"
                    />
                  </Box>
                </Box>

                <Typography
                  variant="body2"
                  color="text.secondary"
                  sx={{ mb: 2 }}
                >
                  {template.description}
                </Typography>

                <Box sx={{ mb: 2 }}>
                  {template.tags.map((tag) => (
                    <Chip
                      key={tag}
                      label={tag}
                      size="small"
                      variant="outlined"
                      sx={{ mr: 0.5, mb: 0.5 }}
                    />
                  ))}
                </Box>

                <Typography variant="body2" sx={{ mb: 1 }}>
                  <strong>Features:</strong>
                </Typography>
                <Box component="ul" sx={{ pl: 2, mb: 2 }}>
                  {template.features.slice(0, 3).map((feature, index) => (
                    <Typography key={index} variant="body2" component="li">
                      {feature}
                    </Typography>
                  ))}
                  {template.features.length > 3 && (
                    <Typography
                      variant="body2"
                      component="li"
                      color="text.secondary"
                    >
                      +{template.features.length - 3} more...
                    </Typography>
                  )}
                </Box>

                <Typography variant="body2" color="text.secondary">
                  <strong>Estimated time:</strong> {template.estimatedTime}
                </Typography>
              </CardContent>

              <CardActions sx={{ justifyContent: 'space-between', p: 2 }}>
                <Tooltip title="View template details">
                  <IconButton size="small">
                    <ViewIcon />
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
