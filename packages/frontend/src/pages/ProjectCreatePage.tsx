import React, { useState, useEffect } from 'react';
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
  FormControlLabel,
  Divider,
  Card,
  CardContent,
  Checkbox,
} from '@mui/material';
import { useNavigate } from 'react-router-dom';
import { ApiService } from '../services/api';
import { FilterCategory, FilterValue } from '@robium/shared';

const ProjectCreatePage: React.FC = () => {
  const navigate = useNavigate();
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);
  const [formData, setFormData] = useState({
    name: '',
    description: '',
    tags: [] as string[],
    isTemplate: false,
  });
  const [tagInput, setTagInput] = useState('');
  // Always create public GitHub repo - no options needed

  // Filter categories and values state
  const [categories, setCategories] = useState<FilterCategory[]>([]);
  const [categoryValues, setCategoryValues] = useState<Record<string, FilterValue[]>>({});
  const [selectedFilters, setSelectedFilters] = useState<Record<string, string[]>>({});
  const [filtersLoading, setFiltersLoading] = useState(false);

  // Load filter categories and values on component mount
  useEffect(() => {
    const loadFilters = async () => {
      setFiltersLoading(true);
      try {
        // Load categories
        const categoriesResponse = await ApiService.getFilterCategories();
        console.log('Categories response:', categoriesResponse);
        if (categoriesResponse.success && categoriesResponse.data) {
          const activeCategories = categoriesResponse.data.categories.filter(
            (cat: FilterCategory) => cat.isActive
          );
          console.log('Active categories:', activeCategories);
          setCategories(activeCategories);

          // Load values for each category
          const valuesPromises = activeCategories.map(async (category: FilterCategory) => {
            try {
              const valuesResponse = await ApiService.getCategoryValues(category.id);
              if (valuesResponse.success && valuesResponse.data) {
                const activeValues = valuesResponse.data.values.filter(
                  (val: FilterValue) => val.isActive
                );
                return { categoryId: category.id, values: activeValues };
              }
              return { categoryId: category.id, values: [] };
            } catch (err) {
              console.error(`Error loading values for category ${category.id}:`, err);
              return { categoryId: category.id, values: [] };
            }
          });

          const valuesResults = await Promise.all(valuesPromises);
          const valuesMap: Record<string, FilterValue[]> = {};
          valuesResults.forEach(({ categoryId, values }) => {
            valuesMap[categoryId] = values;
          });
          setCategoryValues(valuesMap);

          // Initialize selected filters with empty arrays for each category
          const initialFilters: Record<string, string[]> = {};
          activeCategories.forEach((category: FilterCategory) => {
            initialFilters[category.id] = [];
          });
          setSelectedFilters(initialFilters);
        }
      } catch (err) {
        console.error('Error loading filter categories:', err);
        setError('Failed to load filter categories');
      } finally {
        setFiltersLoading(false);
      }
    };

    loadFilters();
  }, []);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);
    setSuccess(null);

    try {
      // Build metadata from selected filters
      const metadata: Record<string, any> = {};

      // Convert selected filters to metadata structure
      categories.forEach((category) => {
        const selectedValues = selectedFilters[category.id] || [];
        if (selectedValues.length > 0) {
          // Map value IDs to actual values for display
          const categoryValuesList = categoryValues[category.id] || [];
          const selectedValueNames = selectedValues.map((valueId) => {
            const value = categoryValuesList.find((v) => v.id === valueId);
            return value ? value.value : valueId;
          });
          metadata[category.name] = selectedValueNames;
        }
      });

      const response = await ApiService.createProject({
        name: formData.name,
        description: formData.description,
        tags: formData.tags,
        isTemplate: formData.isTemplate,
        config: {},
        metadata,
        github: {
          createRepo: true,
          visibility: 'public',
          repoName: formData.name.toLowerCase().replace(/[^a-z0-9-_]/g, '-'),
        },
      });

      if (response.success && response.data && response.data.project) {
        const projectId = response.data.project.id;
        // Show success message with GitHub repo link
        if (response.data.githubRepo) {
          setSuccess(
            `Project created successfully! GitHub repository created at: ${response.data.githubRepo.html_url}. Click the link to view your repository.`
          );
          // Navigate after a short delay to show the success message
          setTimeout(() => {
            navigate(`/projects/${projectId}`);
          }, 3000);
        } else {
          navigate(`/projects/${projectId}`);
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

  const handleFilterChange = (categoryId: string, valueIds: string[]) => {
    setSelectedFilters((prev) => ({
      ...prev,
      [categoryId]: valueIds,
    }));
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

            {/* Filter Categories Section */}
            {categories.length > 0 && (
              <>
                <Divider sx={{ my: 2 }} />
                <Card variant="outlined">
                  <CardContent>
                    <Typography variant="h6" gutterBottom>
                      Project Categories
                    </Typography>
                    <Typography variant="body2" color="text.secondary" sx={{ mb: 3 }}>
                      Select categories that describe your project to help others find it
                    </Typography>

                    {filtersLoading ? (
                      <Box sx={{ display: 'flex', justifyContent: 'center', p: 3 }}>
                        <CircularProgress size={24} />
                      </Box>
                    ) : (
                      <Stack spacing={3}>
                        {categories.map((category) => {
                          const categoryValuesList = categoryValues[category.id] || [];
                          const selectedValues = selectedFilters[category.id] || [];

                          // Show category even if it has no values, but with a message

                          return (
                            <Box key={category.id}>
                              <Typography variant="subtitle1" gutterBottom>
                                {category.displayName}
                              </Typography>
                              <Typography variant="body2" color="text.secondary" sx={{ mb: 1 }}>
                                {category.description ||
                                  `Select ${category.displayName.toLowerCase()} for your project`}
                              </Typography>
                              {categoryValuesList.length > 0 ? (
                                <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
                                  {categoryValuesList.map((value) => (
                                    <FormControlLabel
                                      key={value.id}
                                      control={
                                        <Checkbox
                                          checked={selectedValues.includes(value.id)}
                                          onChange={(e) => {
                                            const newSelectedValues = e.target.checked
                                              ? [...selectedValues, value.id]
                                              : selectedValues.filter((id) => id !== value.id);
                                            handleFilterChange(category.id, newSelectedValues);
                                          }}
                                        />
                                      }
                                      label={value.displayName}
                                    />
                                  ))}
                                </Box>
                              ) : (
                                <Typography
                                  variant="body2"
                                  color="text.secondary"
                                  sx={{ fontStyle: 'italic' }}
                                >
                                  No values available for this category yet.
                                </Typography>
                              )}
                            </Box>
                          );
                        })}
                      </Stack>
                    )}
                  </CardContent>
                </Card>
              </>
            )}

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
