import React, { useState, useEffect } from 'react';
import { 
  Box, 
  Typography, 
  Card, 
  CardContent, 
  Paper, 
  TextField, 
  Button, 
  Grid, 
  List, 
  ListItem, 
  ListItemText, 
  ListItemIcon,
  Chip,
  Avatar,
  Divider,
  IconButton,
  InputAdornment,
} from '@mui/material';
import {
  Dashboard as DashboardIcon,
  Settings as SettingsIcon,
  Person as PersonIcon,
  Add as AddIcon,
  Folder as FolderIcon,
  Send as SendIcon,
  SmartToy as RobotIcon,
  AccessTime as TimeIcon,
} from '@mui/icons-material';
import { useAuth } from '../contexts/AuthContext';
import { useToast } from '../components/Toast';
import { useError } from '../contexts/ErrorContext';
import DataFetchWrapper from '../components/DataFetchWrapper';
import { useNavigate } from 'react-router-dom';

const Dashboard: React.FC = () => {
  const { user } = useAuth();
  const { showSuccess, showError } = useToast();
  const { addError } = useError();
  const navigate = useNavigate();

  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [chatMessage, setChatMessage] = useState('');
  const [recentProjects, setRecentProjects] = useState([
    {
      id: '1',
      name: 'gfdgdfg',
      description: 'Test project',
      lastModified: '2025-08-01T03:48:40Z',
      status: 'active',
    },
  ]);
  const [stats, setStats] = useState([
    {
      title: 'Active Projects',
      value: '0',
      icon: <DashboardIcon />,
      color: 'primary.main',
    },
    {
      title: 'Available Modules',
      value: '0',
      icon: <SettingsIcon />,
      color: 'secondary.main',
    },
    {
      title: 'User Role',
      value: user?.role || 'USER',
      icon: <PersonIcon />,
      color: 'success.main',
    },
  ]);

  // Simulate data loading
  useEffect(() => {
    const loadDashboardData = async () => {
      try {
        setLoading(true);
        setError(null);

        // Simulate API call
        await new Promise((resolve) => setTimeout(resolve, 2000));

        // Update stats with mock data
        setStats([
          {
            title: 'Active Projects',
            value: '3',
            icon: <DashboardIcon />,
            color: 'primary.main',
          },
          {
            title: 'Available Modules',
            value: '11',
            icon: <SettingsIcon />,
            color: 'secondary.main',
          },
          {
            title: 'User Role',
            value: user?.role || 'USER',
            icon: <PersonIcon />,
            color: 'success.main',
          },
        ]);

        showSuccess('Dashboard data loaded successfully');
      } catch (err) {
        const errorMessage = 'Failed to load dashboard data';
        setError(errorMessage);
        addError(errorMessage, 'error', err, 'Dashboard');
        showError(errorMessage);
      } finally {
        setLoading(false);
      }
    };

    loadDashboardData();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [user?.role]); // Only depend on user role, not the functions

  const handleRetry = () => {
    setLoading(true);
    setError(null);
    // Trigger reload
    window.location.reload();
  };

  const handleChatSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (chatMessage.trim()) {
      // In real app, this would send to LLM API
      console.log('Chat message:', chatMessage);
      setChatMessage('');
      showSuccess('Message sent to AI assistant!');
    }
  };

  const handleCreateProject = () => {
    navigate('/projects/new');
  };

  const handleOpenProject = (projectId: string) => {
    navigate(`/workspace/${projectId}`);
  };

  return (
    <DataFetchWrapper
      loading={loading}
      error={error}
      errorTitle="Dashboard Error"
      errorDetails="Unable to load dashboard data. Please check your connection and try again."
      onRetry={handleRetry}
      loadingMessage="Loading dashboard data..."
      showSkeleton={true}
      skeletonType="dashboard"
    >
      <Box 
        sx={{ 
          height: '100vh', 
          display: 'flex', 
          flexDirection: 'column',
          width: '100%'
        }}
      >
        {/* Stats Cards */}
        <Box sx={{ flex: 1 }}>
          <Typography variant="h4" gutterBottom sx={{ fontWeight: 'bold', mb: 3 }}>
            Platform Overview
          </Typography>
          <Grid container spacing={4} sx={{ height: '100%' }}>
            {stats.map((stat) => (
              <Grid item xs={12} sm={6} md={4} key={stat.title}>
                <Card sx={{ height: '100%', display: 'flex', alignItems: 'center', boxShadow: 3 }}>
                  <CardContent sx={{ width: '100%' }}>
                    <Box
                      display="flex"
                      alignItems="center"
                      justifyContent="space-between"
                    >
                      <Box>
                        <Typography color="textSecondary" gutterBottom sx={{ fontSize: '0.9rem' }}>
                          {stat.title}
                        </Typography>
                        <Typography variant="h2" component="div" sx={{ fontWeight: 'bold', lineHeight: 1 }}>
                          {stat.value}
                        </Typography>
                      </Box>
                      <Box
                        sx={{
                          color: stat.color,
                          display: 'flex',
                          alignItems: 'center',
                          fontSize: '2.5rem',
                        }}
                      >
                        {stat.icon}
                      </Box>
                    </Box>
                  </CardContent>
                </Card>
              </Grid>
            ))}
          </Grid>
        </Box>
      </Box>
    </DataFetchWrapper>
  );
};

export default Dashboard;
