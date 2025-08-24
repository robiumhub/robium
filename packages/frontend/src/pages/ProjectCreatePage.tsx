import React, { useState } from 'react';
import {
  Box,
  Typography,
  TextField,
  Button,
  Paper,
  Chip,
  Stack,
  Alert,
  CircularProgress,
  Divider,
  Card,
  CardContent,
} from '@mui/material';
import { useNavigate } from 'react-router-dom';
import { ApiService } from '../services/api';

const ProjectCreatePage: React.FC = () => {
  const navigate = useNavigate();
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);
  const [formData, setFormData] = useState({
    name: '',
    description: '',
    tags: [] as string[],
    isTemplate: false, // Always create as project, not template
  });
  const [tagInput, setTagInput] = useState('');


  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);
    setSuccess(null);



    try {
      const response = await ApiService.createProject({
        name: formData.name,
        description: formData.description,
        tags: formData.tags,
        isTemplate: formData.isTemplate,
        config: {},
        metadata: {},
        github: {
          createRepo: true,
          visibility: 'private',
          repoName: formData.name.toLowerCase().replace(/[^a-z0-9-_]/g, '-'),
        },
      });

      if (response.success && response.data && response.data.project) {
        // Show success message with GitHub repo link
        if (response.data.githubRepo) {
          setSuccess(
            `Project created successfully! GitHub repository created at: ${response.data.githubRepo.html_url}. Click the link to view your repository.`
          );
          // Navigate after a short delay to show the success message
          setTimeout(() => {
            navigate(`/projects/${response.data!.project.id}`);
          }, 3000);
        } else {
          navigate(`/projects/${response.data!.project.id}`);
        }
      } else {
        setError(response.error || 'Failed to create project');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred while creating the project');
    } finally {
      setLoading(false);
    }
  };

  const handleAddTag = () => {
    if (tagInput.trim() && !formData.tags.includes(tagInput.trim())) {
      setFormData((prev) => ({
        ...prev,
        tags: [...prev.tags, tagInput.trim()],
      }));
      setTagInput('');
    }
  };

  const handleRemoveTag = (tagToRemove: string) => {
    setFormData((prev) => ({
      ...prev,
      tags: prev.tags.filter((tag) => tag !== tagToRemove),
    }));
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter') {
      e.preventDefault();
      handleAddTag();
    }
  };

  return (
    <Box sx={{ maxWidth: 800, mx: 'auto', p: 3 }}>
      <Typography variant="h4" gutterBottom>
        Create New Project
      </Typography>

      <Paper sx={{ p: 4, mt: 3 }}>
        <form onSubmit={handleSubmit}>
          <Stack spacing={3}>
            {error && (
              <Alert severity="error" onClose={() => setError(null)}>
                {error}
              </Alert>
            )}

            {success && (
              <Alert
                severity="success"
                onClose={() => setSuccess(null)}
                action={
                  success.includes('GitHub repository') ? (
                    <Button
                      color="inherit"
                      size="small"
                      onClick={() => {
                        const url = success.match(/https:\/\/github\.com\/[^\s]+/)?.[0];
                        if (url) window.open(url, '_blank');
                      }}
                    >
                      View Repository
                    </Button>
                  ) : null
                }
              >
                {success}
              </Alert>
            )}

            <TextField
              label="Project Name"
              value={formData.name}
              onChange={(e) => setFormData((prev) => ({ ...prev, name: e.target.value }))}
              required
              fullWidth
              placeholder="Enter project name"
            />

            <TextField
              label="Description"
              value={formData.description}
              onChange={(e) => setFormData((prev) => ({ ...prev, description: e.target.value }))}
              multiline
              rows={4}
              fullWidth
              placeholder="Describe your project"
            />



            <Box>
              <Typography variant="subtitle1" gutterBottom>
                Tags
              </Typography>
              <Box sx={{ display: 'flex', gap: 1, mb: 2 }}>
                <TextField
                  value={tagInput}
                  onChange={(e) => setTagInput(e.target.value)}
                  onKeyPress={handleKeyPress}
                  placeholder="Add a tag"
                  size="small"
                  sx={{ flexGrow: 1 }}
                />
                <Button variant="outlined" onClick={handleAddTag} disabled={!tagInput.trim()}>
                  Add
                </Button>
              </Box>
              <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                {formData.tags.map((tag) => (
                  <Chip
                    key={tag}
                    label={tag}
                    onDelete={() => handleRemoveTag(tag)}
                    color="primary"
                    variant="outlined"
                  />
                ))}
              </Box>
            </Box>

            <Divider sx={{ my: 2 }} />

            {/* GitHub Repository Info */}
            <Card variant="outlined" sx={{ bgcolor: 'primary.50', borderColor: 'primary.200' }}>
              <CardContent>
                <Typography variant="h6" gutterBottom sx={{ color: 'primary.main' }}>
                  GitHub Repository
                </Typography>
                <Typography variant="body2" color="text.secondary">
                  A private GitHub repository will be automatically created for this project with the project name.
                </Typography>
              </CardContent>
            </Card>

            <Box sx={{ display: 'flex', gap: 2, justifyContent: 'flex-end' }}>
              <Button variant="outlined" onClick={() => navigate('/projects')} disabled={loading}>
                Cancel
              </Button>
              <Button
                type="submit"
                variant="contained"
                disabled={loading || !formData.name.trim()}
                startIcon={loading ? <CircularProgress size={20} /> : null}
              >
                {loading ? 'Creating...' : 'Create Project'}
              </Button>
            </Box>
          </Stack>
        </form>
      </Paper>
    </Box>
  );
};

export default ProjectCreatePage;
