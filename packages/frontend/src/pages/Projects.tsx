import React from 'react';
import {
  Box,
  Typography,
  Card,
  CardContent,
  CardActions,
  Button,
  Chip,
  Avatar,
} from '@mui/material';
import Grid from '@mui/material/Grid';
import { Add as AddIcon, Folder as FolderIcon } from '@mui/icons-material';
import { Link as RouterLink } from 'react-router-dom';

const Projects: React.FC = () => {
  // Mock data - in real app this would come from API
  const projects = [
    {
      id: '1',
      name: 'Autonomous Navigation Robot',
      description:
        'A robot capable of navigating complex environments autonomously',
      status: 'active',
      robotCount: 3,
      lastUpdated: '2024-01-15',
    },
    {
      id: '2',
      name: 'Industrial Assembly Bot',
      description: 'Robotic arm for precision assembly tasks',
      status: 'completed',
      robotCount: 1,
      lastUpdated: '2024-01-10',
    },
    {
      id: '3',
      name: 'Surveillance Drone',
      description: 'Aerial surveillance and monitoring system',
      status: 'planning',
      robotCount: 0,
      lastUpdated: '2024-01-20',
    },
  ];

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'active':
        return 'success';
      case 'completed':
        return 'default';
      case 'planning':
        return 'warning';
      default:
        return 'default';
    }
  };

  return (
    <Box>
      <Box
        sx={{
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
          mb: 3,
        }}
      >
        <Typography variant="h4" component="h1" gutterBottom>
          Projects
        </Typography>
        <Button
          variant="contained"
          startIcon={<AddIcon />}
          component={RouterLink}
          to="/projects/new"
        >
          New Project
        </Button>
      </Box>

      <Grid container spacing={3}>
        {projects.map((project) => (
          <Grid item xs={12} sm={6} md={4} key={project.id}>
            <Card
              sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}
            >
              <CardContent sx={{ flexGrow: 1 }}>
                <Box sx={{ display: 'flex', alignItems: 'center', mb: 2 }}>
                  <Avatar sx={{ mr: 2, bgcolor: 'primary.main' }}>
                    <FolderIcon />
                  </Avatar>
                  <Box>
                    <Typography variant="h6" component="h2" gutterBottom>
                      {project.name}
                    </Typography>
                    <Chip
                      label={project.status}
                      color={getStatusColor(project.status) as any}
                      size="small"
                    />
                  </Box>
                </Box>
                <Typography
                  variant="body2"
                  color="text.secondary"
                  sx={{ mb: 2 }}
                >
                  {project.description}
                </Typography>
                <Box
                  sx={{
                    display: 'flex',
                    justifyContent: 'space-between',
                    alignItems: 'center',
                  }}
                >
                  <Typography variant="caption" color="text.secondary">
                    {project.robotCount} robot
                    {project.robotCount !== 1 ? 's' : ''}
                  </Typography>
                  <Typography variant="caption" color="text.secondary">
                    Updated: {project.lastUpdated}
                  </Typography>
                </Box>
              </CardContent>
              <CardActions>
                <Button
                  size="small"
                  component={RouterLink}
                  to={`/projects/${project.id}`}
                >
                  View Details
                </Button>
                <Button size="small">Edit</Button>
              </CardActions>
            </Card>
          </Grid>
        ))}
      </Grid>
    </Box>
  );
};

export default Projects;
