import React, { useState, useEffect, Fragment, useMemo } from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
  TextField,
  Button,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  Avatar,
  Chip,
  IconButton,
  InputAdornment,
  Paper,
  Divider,
  ToggleButtonGroup,
  ToggleButton,
} from '@mui/material';
import Grid from '@mui/material/Grid';
import {
  Send as SendIcon,
  Add as AddIcon,
  Folder as FolderIcon,
  Extension as ModuleIcon,
  Storage as StorageIcon,
  SmartToy as RobotIcon,
  AccessTime as TimeIcon,
  OpenInNew as OpenInNewIcon,
  Create as CreateIcon,
  Home as HomeIcon,
  Article as TemplateIcon,
  Search as SearchIcon,
  Clear as ClearIcon,
  ViewModule as ViewModuleIcon,
  ViewList as ViewListIcon,
} from '@mui/icons-material';
import { useNavigate } from 'react-router-dom';
import { useAuth } from '../contexts/AuthContext';
import { ApiService } from '../services/api';
import DataFetchWrapper from '../components/DataFetchWrapper';

interface Project {
  id: string;
  name: string;
  description: string;
  status: string;
  created_at: string;
  updated_at: string;
}

interface Stats {
  projects: number;
  modules: number;
  templates: number;
  datasets: number;
}

const Home: React.FC = () => {
  const [chatMessage, setChatMessage] = useState('');
  const [recentProjects, setRecentProjects] = useState<Project[]>([]);
  const [stats, setStats] = useState<Stats>({
    projects: 0,
    modules: 0,
    templates: 0,
    datasets: 0,
  });
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [projectSearchQuery, setProjectSearchQuery] = useState('');
  const [projectViewMode, setProjectViewMode] = useState<'grid' | 'list'>(
    'list'
  );
  const { user } = useAuth();
  const navigate = useNavigate();

  useEffect(() => {
    loadHomeData();
  }, []);

  const loadHomeData = async () => {
    try {
      setLoading(true);

      // Load recent projects
      const projectsResult = await ApiService.get<Project[]>('/projects');
      const projects = projectsResult || [];
      setRecentProjects(projects.slice(0, 5)); // Show last 5 projects

      // Load real stats
      const overview = await ApiService.get<{
        projects: number;
        modules: number;
        templates: number;
        datasets: number;
      }>('/dashboard/stats');
      setStats({
        projects: overview.projects,
        modules: overview.modules,
        templates: overview.templates,
        datasets: overview.datasets,
      });

      setError(null);
    } catch (err) {
      setError('Failed to load home data');
      console.error('Error loading home data:', err);
    } finally {
      setLoading(false);
    }
  };

  // Filter projects based on search query
  const filteredProjects = useMemo(() => {
    return recentProjects.filter(
      (project) =>
        project.name.toLowerCase().includes(projectSearchQuery.toLowerCase()) ||
        project.description
          .toLowerCase()
          .includes(projectSearchQuery.toLowerCase())
    );
  }, [recentProjects, projectSearchQuery]);

  const handleClearProjectSearch = () => {
    setProjectSearchQuery('');
  };

  const handleGenerateProject = () => {
    if (chatMessage.trim()) {
      // Navigate to AI project generation with user input
      navigate(
        `/projects/new?mode=ai&input=${encodeURIComponent(chatMessage.trim())}`
      );
    } else {
      // If no input, just go to AI mode
      navigate('/projects/new?mode=ai');
    }
  };

  const handleCreateProject = () => {
    navigate('/projects/new');
  };

  const handleOpenProject = (projectId: string) => {
    navigate(`/workspace/${projectId}`);
  };

  const handleRetry = () => {
    loadHomeData();
  };

  const statsCards = [
    {
      title: 'Projects',
      value: stats.projects,
      icon: <FolderIcon />,
      color: 'primary.main',
    },
    {
      title: 'Modules',
      value: stats.modules,
      icon: <ModuleIcon />,
      color: 'secondary.main',
    },
    {
      title: 'Templates',
      value: stats.templates,
      icon: <TemplateIcon />,
      color: 'success.main',
    },
    {
      title: 'Datasets',
      value: stats.datasets,
      icon: <StorageIcon />,
      color: 'warning.main',
    },
  ];

  return (
    <DataFetchWrapper
      loading={loading}
      error={error}
      errorTitle="Home Error"
      errorDetails="Unable to load home data. Please check your connection and try again."
      onRetry={handleRetry}
      loadingMessage="Loading home data..."
      showSkeleton={true}
      skeletonType="dashboard"
    >
      <Box
        sx={{
          minHeight: '100vh',
          display: 'flex',
          flexDirection: 'column',
          width: '100%',
          background: 'linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%)',
        }}
      >
        {/* Main Content Container */}
        <Box sx={{ flex: 1, display: 'flex', flexDirection: 'column' }}>
          {/* Content Layout - Side by Side */}
          <Box
            sx={{
              display: 'flex',
              flexDirection: { xs: 'column', lg: 'row' },
              gap: 4,
              height: '100%',
            }}
          >
            {/* Left Side - Platform Overview and Recent Projects */}
            <Box
              sx={{
                flex: 1,
                display: 'flex',
                flexDirection: 'column',
                gap: 4,
              }}
            >
              {/* Platform Overview */}
              <Box>
                <Typography
                  variant="h4"
                  component="h2"
                  gutterBottom
                  sx={{
                    fontWeight: 700,
                    mb: 4,
                    color: '#2c3e50',
                  }}
                >
                  Platform Overview
                </Typography>
                <Box
                  sx={{
                    display: 'grid',
                    gridTemplateColumns: {
                      xs: '1fr',
                      sm: 'repeat(2, 1fr)',
                      md: 'repeat(2, 1fr)',
                      lg: 'repeat(2, 1fr)',
                    },
                    gap: 3,
                  }}
                >
                  {statsCards.map((stat) => (
                    <Card
                      key={stat.title}
                      sx={{
                        height: '100%',
                        display: 'flex',
                        alignItems: 'center',
                        boxShadow: '0 8px 25px rgba(0,0,0,0.1)',
                        borderRadius: 3,
                        transition: 'all 0.3s ease-in-out',
                        '&:hover': {
                          transform: 'translateY(-4px)',
                          boxShadow: '0 15px 35px rgba(0,0,0,0.15)',
                        },
                      }}
                    >
                      <CardContent sx={{ width: '100%', p: 3 }}>
                        <Box
                          display="flex"
                          alignItems="center"
                          justifyContent="space-between"
                        >
                          <Box>
                            <Typography
                              color="textSecondary"
                              gutterBottom
                              sx={{
                                fontSize: '0.9rem',
                                fontWeight: 500,
                                textTransform: 'uppercase',
                                letterSpacing: '0.5px',
                              }}
                            >
                              {stat.title}
                            </Typography>
                            <Typography
                              variant="h3"
                              component="div"
                              sx={{
                                fontWeight: 700,
                                color: '#2c3e50',
                              }}
                            >
                              {stat.value}
                            </Typography>
                          </Box>
                          <Avatar
                            sx={{
                              bgcolor: stat.color,
                              width: 56,
                              height: 56,
                              display: 'flex',
                              alignItems: 'center',
                              justifyContent: 'center',
                            }}
                          >
                            {stat.icon}
                          </Avatar>
                        </Box>
                      </CardContent>
                    </Card>
                  ))}
                </Box>
              </Box>

              {/* Recent Projects */}
              <Box sx={{ flex: 1 }}>
                <Box
                  sx={{
                    display: 'flex',
                    justifyContent: 'space-between',
                    alignItems: 'center',
                    mb: 3,
                    flexWrap: 'wrap',
                    gap: 2,
                  }}
                >
                  <Typography
                    variant="h4"
                    component="h3"
                    sx={{
                      fontWeight: 700,
                      color: '#2c3e50',
                    }}
                  >
                    Recent Projects
                  </Typography>

                  <Box sx={{ display: 'flex', gap: 1, alignItems: 'center' }}>
                    <ToggleButtonGroup
                      value={projectViewMode}
                      exclusive
                      onChange={(_, newMode) =>
                        newMode && setProjectViewMode(newMode)
                      }
                      size="small"
                    >
                      <ToggleButton value="grid">
                        <ViewModuleIcon />
                      </ToggleButton>
                      <ToggleButton value="list">
                        <ViewListIcon />
                      </ToggleButton>
                    </ToggleButtonGroup>
                  </Box>
                </Box>

                {/* Search Controls */}
                <Box
                  sx={{
                    display: 'flex',
                    gap: 2,
                    mb: 3,
                    flexWrap: 'wrap',
                    alignItems: 'center',
                  }}
                >
                  <TextField
                    placeholder="Search projects..."
                    value={projectSearchQuery}
                    onChange={(e) => setProjectSearchQuery(e.target.value)}
                    InputProps={{
                      startAdornment: (
                        <InputAdornment position="start">
                          <SearchIcon />
                        </InputAdornment>
                      ),
                      endAdornment: projectSearchQuery && (
                        <InputAdornment position="end">
                          <IconButton
                            size="small"
                            onClick={handleClearProjectSearch}
                            edge="end"
                          >
                            <ClearIcon />
                          </IconButton>
                        </InputAdornment>
                      ),
                    }}
                    size="small"
                    sx={{ minWidth: 200 }}
                  />

                  <Typography variant="body2" color="text.secondary">
                    {filteredProjects.length} of {recentProjects.length}{' '}
                    projects
                  </Typography>
                </Box>

                {filteredProjects.length > 0 ? (
                  projectViewMode === 'list' ? (
                    <List sx={{ p: 0 }}>
                      {filteredProjects.map((project, index) => (
                        <React.Fragment key={project.id}>
                          <Card
                            sx={{
                              mb: 2,
                              borderRadius: 3,
                              boxShadow: '0 4px 12px rgba(0,0,0,0.08)',
                              transition: 'all 0.3s ease-in-out',
                              cursor: 'pointer',
                              '&:hover': {
                                transform: 'translateY(-2px)',
                                boxShadow: '0 8px 20px rgba(0,0,0,0.12)',
                              },
                            }}
                            onClick={() => handleOpenProject(project.id)}
                          >
                            <CardContent sx={{ p: 3 }}>
                              <Box
                                display="flex"
                                alignItems="center"
                                justifyContent="space-between"
                              >
                                <Box display="flex" alignItems="center" gap={2}>
                                  <FolderIcon
                                    color="primary"
                                    sx={{ fontSize: 32 }}
                                  />
                                  <Box>
                                    <Typography
                                      variant="h6"
                                      sx={{ fontWeight: 600, mb: 1 }}
                                    >
                                      {project.name}
                                    </Typography>
                                    <Box
                                      display="flex"
                                      alignItems="center"
                                      gap={2}
                                    >
                                      <Box
                                        display="flex"
                                        alignItems="center"
                                        gap={1}
                                      >
                                        <TimeIcon
                                          fontSize="small"
                                          color="action"
                                        />
                                        <Typography
                                          variant="body2"
                                          color="text.secondary"
                                        >
                                          {new Date(
                                            project.updated_at
                                          ).toLocaleDateString()}
                                        </Typography>
                                      </Box>
                                      <Chip
                                        label={project.status}
                                        size="small"
                                        color="success"
                                        sx={{ fontWeight: 500 }}
                                      />
                                    </Box>
                                  </Box>
                                </Box>
                                <IconButton
                                  size="medium"
                                  sx={{
                                    '&:hover': {
                                      backgroundColor: 'primary.main',
                                      color: 'white',
                                    },
                                  }}
                                >
                                  <OpenInNewIcon />
                                </IconButton>
                              </Box>
                            </CardContent>
                          </Card>
                        </React.Fragment>
                      ))}
                    </List>
                  ) : (
                    <Box
                      sx={{
                        display: 'grid',
                        gridTemplateColumns: {
                          xs: '1fr',
                          sm: 'repeat(2, 1fr)',
                          md: 'repeat(3, 1fr)',
                        },
                        gap: 3,
                      }}
                    >
                      {filteredProjects.map((project) => (
                        <Card
                          key={project.id}
                          sx={{
                            height: '100%',
                            borderRadius: 3,
                            boxShadow: '0 4px 12px rgba(0,0,0,0.08)',
                            transition: 'all 0.3s ease-in-out',
                            cursor: 'pointer',
                            '&:hover': {
                              transform: 'translateY(-2px)',
                              boxShadow: '0 8px 20px rgba(0,0,0,0.12)',
                            },
                          }}
                          onClick={() => handleOpenProject(project.id)}
                        >
                          <CardContent sx={{ p: 3 }}>
                            <Box
                              display="flex"
                              alignItems="center"
                              gap={2}
                              mb={2}
                            >
                              <FolderIcon
                                color="primary"
                                sx={{ fontSize: 24 }}
                              />
                              <Typography variant="h6" sx={{ fontWeight: 600 }}>
                                {project.name}
                              </Typography>
                            </Box>
                            <Typography
                              variant="body2"
                              color="text.secondary"
                              sx={{ mb: 2 }}
                            >
                              {project.description}
                            </Typography>
                            <Box
                              display="flex"
                              alignItems="center"
                              justifyContent="space-between"
                            >
                              <Box display="flex" alignItems="center" gap={1}>
                                <TimeIcon fontSize="small" color="action" />
                                <Typography
                                  variant="body2"
                                  color="text.secondary"
                                >
                                  {new Date(
                                    project.updated_at
                                  ).toLocaleDateString()}
                                </Typography>
                              </Box>
                              <Chip
                                label={project.status}
                                size="small"
                                color="success"
                              />
                            </Box>
                          </CardContent>
                        </Card>
                      ))}
                    </Box>
                  )
                ) : (
                  <Card
                    sx={{
                      textAlign: 'center',
                      py: 6,
                      boxShadow: '0 4px 12px rgba(0,0,0,0.08)',
                      borderRadius: 3,
                    }}
                  >
                    <CardContent>
                      <Typography
                        variant="h6"
                        color="text.secondary"
                        sx={{ mb: 3, fontWeight: 500 }}
                      >
                        {projectSearchQuery
                          ? 'No projects found matching your search'
                          : 'No projects yet. Create your first project to get started!'}
                      </Typography>
                      {!projectSearchQuery && (
                        <Button
                          variant="contained"
                          size="large"
                          startIcon={<AddIcon />}
                          onClick={handleCreateProject}
                          sx={{
                            borderRadius: 3,
                            px: 4,
                            py: 1.5,
                            fontSize: '1.1rem',
                            fontWeight: 600,
                          }}
                        >
                          Create Your First Project
                        </Button>
                      )}
                    </CardContent>
                  </Card>
                )}
              </Box>
            </Box>

            {/* Right Side - AI Assistant Section */}
            <Box
              sx={{
                width: { xs: '100%', lg: '400px' },
                flexShrink: 0,
                display: 'flex',
                flexDirection: 'column',
              }}
            >
              {/* Section Title */}
              <Typography
                variant="h6"
                component="h2"
                sx={{
                  fontWeight: 700,
                  mb: 2,
                  fontSize: { xs: '1.1rem', md: '1.25rem' },
                  color: '#2c3e50',
                }}
              >
                Create New Project
              </Typography>

              <Card
                sx={{
                  background:
                    'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
                  color: 'white',
                  borderRadius: 3,
                  boxShadow: '0 12px 24px rgba(0,0,0,0.1)',
                  overflow: 'hidden',
                  position: 'relative',
                  '&::before': {
                    content: '""',
                    position: 'absolute',
                    top: 0,
                    left: 0,
                    right: 0,
                    bottom: 0,
                    background: 'rgba(255,255,255,0.1)',
                    backdropFilter: 'blur(10px)',
                  },
                }}
              >
                <CardContent
                  sx={{
                    position: 'relative',
                    zIndex: 1,
                    p: { xs: 3, md: 4 },
                    display: 'flex',
                    flexDirection: 'column',
                    alignItems: 'center',
                    textAlign: 'center',
                  }}
                >
                  {/* Header Section */}
                  <Box sx={{ mb: 3 }}>
                    <Typography
                      variant="body2"
                      sx={{
                        opacity: 0.9,
                        fontSize: { xs: '0.9rem', md: '1rem' },
                        maxWidth: 350,
                        fontWeight: 500,
                      }}
                    >
                      Describe your robotics project and I'll create it for you.
                      We'll configure the project and add all necessary robotics
                      libraries automatically.
                    </Typography>
                  </Box>

                  {/* Input Section */}
                  <Box sx={{ width: '100%' }}>
                    <TextField
                      fullWidth
                      multiline
                      rows={4}
                      placeholder="I want to build an autonomous mobile robot that can navigate my office"
                      value={chatMessage}
                      onChange={(e) => setChatMessage(e.target.value)}
                      onKeyPress={(e) => {
                        if (e.key === 'Enter' && chatMessage.trim()) {
                          handleGenerateProject();
                        }
                      }}
                      InputProps={{
                        endAdornment: (
                          <InputAdornment position="end">
                            <IconButton
                              onClick={handleGenerateProject}
                              disabled={!chatMessage.trim()}
                              sx={{
                                color: 'rgba(255,255,255,0.8)',
                                '&:hover': {
                                  backgroundColor: 'rgba(255,255,255,0.1)',
                                  color: 'white',
                                },
                                '&:disabled': {
                                  color: 'rgba(255,255,255,0.3)',
                                },
                              }}
                            >
                              <SendIcon />
                            </IconButton>
                          </InputAdornment>
                        ),
                      }}
                      sx={{
                        '& .MuiOutlinedInput-root': {
                          backgroundColor: 'rgba(255,255,255,0.1)',
                          backdropFilter: 'blur(10px)',
                          border: '1px solid rgba(255,255,255,0.3)',
                          color: 'white',
                          borderRadius: 2,
                          height: '100%',
                          '&:hover': {
                            borderColor: 'rgba(255,255,255,0.5)',
                          },
                          '&.Mui-focused': {
                            borderColor: 'white',
                          },
                          '& textarea': {
                            color: 'white',
                            fontSize: '1rem',
                            '&::placeholder': {
                              color: 'rgba(255,255,255,0.7)',
                              opacity: 1,
                            },
                          },
                          '& fieldset': {
                            border: 'none',
                          },
                        },
                      }}
                    />
                  </Box>
                </CardContent>
              </Card>

              {/* Manual Project Creation Link - Outside the AI Card */}
              <Box sx={{ mt: 3, textAlign: 'center' }}>
                <Typography
                  variant="body2"
                  sx={{ color: 'text.secondary', mb: 2 }}
                >
                  or
                </Typography>
                <Button
                  variant="outlined"
                  onClick={handleCreateProject}
                  sx={{
                    color: 'primary.main',
                    borderColor: 'primary.main',
                    textTransform: 'none',
                    fontSize: '1rem',
                    fontWeight: 600,
                    px: 3,
                    py: 1.5,
                    borderRadius: 2,
                    '&:hover': {
                      backgroundColor: 'primary.main',
                      color: 'white',
                      borderColor: 'primary.main',
                    },
                  }}
                >
                  Create Project Manually
                </Button>
              </Box>
            </Box>
          </Box>
        </Box>
      </Box>
    </DataFetchWrapper>
  );
};

export default Home;
