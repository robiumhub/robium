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

const TemplatesPage: React.FC = () => {
  const [templates, setTemplates] = useState<Project[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const fetchTemplates = async () => {
      try {
        setLoading(true);
        setError(null);
        const response = await apiService.getTemplates();
        if (response.success && response.data && response.data.projects) {
          setTemplates(response.data.projects);
        } else {
          setError('Failed to fetch templates');
        }
      } catch (err) {
        console.error('Error fetching templates:', err);
        setError('Failed to load templates');
      } finally {
        setLoading(false);
      }
    };

    fetchTemplates();
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
        Templates
      </Typography>

      {templates.length === 0 ? (
        <Typography variant="body1" color="text.secondary">
          No templates available. Check back later for new project templates.
        </Typography>
      ) : (
        <Grid container spacing={3}>
          {templates.map((template) => (
            <Grid item xs={12} sm={6} md={4} key={template.id}>
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    {template.name}
                  </Typography>
                  <Typography variant="body2" color="text.secondary" paragraph>
                    {template.description}
                  </Typography>
                  <Box mb={2}>
                    {template.tags.map((tag) => (
                      <Chip key={tag} label={tag} size="small" sx={{ mr: 1, mb: 1 }} />
                    ))}
                  </Box>
                  {template.metadata?.useCases && (
                    <Typography variant="caption" color="text.secondary">
                      Use Cases: {template.metadata.useCases.join(', ')}
                    </Typography>
                  )}
                  {template.templateVersion && (
                    <Typography variant="caption" color="text.secondary" display="block">
                      Version: {template.templateVersion}
                    </Typography>
                  )}
                </CardContent>
                <CardActions>
                  <Button size="small" color="primary">
                    Use Template
                  </Button>
                  <Button size="small" color="secondary">
                    View Details
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

export default TemplatesPage;
