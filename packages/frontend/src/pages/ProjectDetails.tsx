import React from 'react';
import {
  Box,
  Typography,
  Card,
  CardContent,
  Grid,
  Button,
  Chip,
  Avatar,
  Divider,
  List,
  ListItem,
  ListItemText,
  ListItemAvatar,
} from '@mui/material';
import { useParams, Link as RouterLink } from 'react-router-dom';
import {
  ArrowBack as ArrowBackIcon,
  Edit as EditIcon,
} from '@mui/icons-material';

const ProjectDetails: React.FC = () => {
  const { projectId } = useParams<{ projectId: string }>();

  // Mock data - in real app this would come from API
  const project = {
    id: projectId,
    name: 'Autonomous Navigation Robot',
    description:
      'A robot capable of navigating complex environments autonomously using advanced AI algorithms and sensor fusion.',
    status: 'active',
    robotCount: 3,
    lastUpdated: '2024-01-15',
    createdDate: '2023-12-01',
    robots: [
      { id: '1', name: 'NavBot-001', status: 'online', type: 'Navigation' },
      { id: '2', name: 'NavBot-002', status: 'offline', type: 'Navigation' },
      {
        id: '3',
        name: 'NavBot-003',
        status: 'maintenance',
        type: 'Navigation',
      },
    ],
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'online':
        return 'success';
      case 'offline':
        return 'error';
      case 'maintenance':
        return 'warning';
      default:
        return 'default';
    }
  };

  return (
    <Box>
      <Box sx={{ display: 'flex', alignItems: 'center', mb: 3 }}>
        <Button
          component={RouterLink}
          to="/projects"
          startIcon={<ArrowBackIcon />}
          sx={{ mr: 2 }}
        >
          Back to Projects
        </Button>
        <Typography variant="h4" component="h1" sx={{ flexGrow: 1 }}>
          {project.name}
        </Typography>
        <Button variant="outlined" startIcon={<EditIcon />} sx={{ ml: 2 }}>
          Edit Project
        </Button>
      </Box>

      <Grid container spacing={3}>
        <Grid item xs={12} md={8}>
          <Card sx={{ mb: 3 }}>
            <CardContent>
              <Typography variant="h6" gutterBottom>
                Project Overview
              </Typography>
              <Typography variant="body1" color="text.secondary" paragraph>
                {project.description}
              </Typography>
              <Box sx={{ display: 'flex', gap: 2, flexWrap: 'wrap' }}>
                <Chip label={`Status: ${project.status}`} color="primary" />
                <Chip label={`${project.robotCount} Robots`} />
                <Chip label={`Created: ${project.createdDate}`} />
                <Chip label={`Updated: ${project.lastUpdated}`} />
              </Box>
            </CardContent>
          </Card>

          <Card>
            <CardContent>
              <Typography variant="h6" gutterBottom>
                Robots in Project
              </Typography>
              <List>
                {project.robots.map((robot) => (
                  <React.Fragment key={robot.id}>
                    <ListItem>
                      <ListItemAvatar>
                        <Avatar>{robot.name.charAt(0)}</Avatar>
                      </ListItemAvatar>
                      <ListItemText
                        primary={
                          <Box
                            sx={{
                              display: 'flex',
                              alignItems: 'center',
                              gap: 1,
                            }}
                          >
                            <Typography variant="subtitle1">
                              {robot.name}
                            </Typography>
                            <Chip
                              label={robot.status}
                              color={getStatusColor(robot.status) as any}
                              size="small"
                            />
                          </Box>
                        }
                        secondary={`Type: ${robot.type}`}
                      />
                      <Button
                        component={RouterLink}
                        to={`/robots/${robot.id}`}
                        size="small"
                      >
                        View Details
                      </Button>
                    </ListItem>
                    <Divider />
                  </React.Fragment>
                ))}
              </List>
            </CardContent>
          </Card>
        </Grid>

        <Grid item xs={12} md={4}>
          <Card>
            <CardContent>
              <Typography variant="h6" gutterBottom>
                Quick Actions
              </Typography>
              <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
                <Button
                  variant="contained"
                  fullWidth
                  component={RouterLink}
                  to={`/projects/${projectId}/edit`}
                >
                  Edit Project
                </Button>
                <Button
                  variant="outlined"
                  fullWidth
                  component={RouterLink}
                  to={`/projects/${projectId}/robots/add`}
                >
                  Add Robot
                </Button>
                <Button
                  variant="outlined"
                  fullWidth
                  component={RouterLink}
                  to={`/projects/${projectId}/analytics`}
                >
                  View Analytics
                </Button>
              </Box>
            </CardContent>
          </Card>
        </Grid>
      </Grid>
    </Box>
  );
};

export default ProjectDetails;
