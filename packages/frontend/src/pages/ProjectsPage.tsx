import React, { useState, useEffect } from 'react';
import {
  Typography,
  Box,
  Card,
  CardContent,
  CardActions,
  Button,
  Grid,
  Chip,
  CircularProgress,
  Alert,
} from '@mui/material';
import { apiService } from '../services/api';
import { Project } from '@robium/shared';

const ProjectsPage: React.FC = () => {
  const [projects, setProjects] = useState<Project[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const fetchProjects = async () => {
      try {
        setLoading(true);
        setError(null);
        const response = await apiService.getProjects();
        if (response.success && response.data && response.data.projects) {
          setProjects(response.data.projects);
        } else {
          setError('Failed to fetch projects');
        }
      } catch (err) {
        console.error('Error fetching projects:', err);
        setError('Failed to load projects');
      } finally {
        setLoading(false);
      }
    };

    fetchProjects();
  }, []);

  if (loading) {
    return (
      <Box display="flex" justifyContent="center" alignItems="center" minHeight="200px">
        <CircularProgress />
      </Box>
    );
  }

  if (error) {
    return (
      <Box>
        <Alert severity="error" sx={{ mb: 2 }}>
          {error}
        </Alert>
      </Box>
    );
  }

  return (
    <Box>
      <Typography variant="h4" gutterBottom>
        Projects
      </Typography>

      {projects.length === 0 ? (
        <Typography variant="body1" color="text.secondary">
          No projects found. Create your first robotics project to get started.
        </Typography>
      ) : (
        <Grid container spacing={3}>
          {projects.map((project) => (
            <Grid item xs={12} sm={6} md={4} key={project.id}>
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    {project.name}
                  </Typography>
                  <Typography variant="body2" color="text.secondary" paragraph>
                    {project.description}
                  </Typography>
                  <Box mb={2}>
                    {project.tags.map((tag) => (
                      <Chip key={tag} label={tag} size="small" sx={{ mr: 1, mb: 1 }} />
                    ))}
                  </Box>
                  {project.metadata?.useCases && (
                    <Typography variant="caption" color="text.secondary">
                      Use Cases: {project.metadata.useCases.join(', ')}
                    </Typography>
                  )}
                </CardContent>
                <CardActions>
                  <Button size="small" color="primary">
                    Open
                  </Button>
                  <Button size="small" color="secondary">
                    Settings
                  </Button>
                </CardActions>
              </Card>
            </Grid>
          ))}
        </Grid>
      )}
    </Box>
  );
};

export default ProjectsPage;
