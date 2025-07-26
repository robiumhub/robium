import React, { useState, useEffect } from 'react';
import { Box, Typography, Card, CardContent, Paper } from '@mui/material';
import {
  Dashboard as DashboardIcon,
  Settings as SettingsIcon,
  Person as PersonIcon,
} from '@mui/icons-material';
import { useAuth } from '../contexts/AuthContext';
import { useToast } from '../components/Toast';
import { useError } from '../contexts/ErrorContext';
import DataFetchWrapper from '../components/DataFetchWrapper';

const Dashboard: React.FC = () => {
  const { user } = useAuth();
  const { showSuccess, showError } = useToast();
  const { addError } = useError();

  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [stats, setStats] = useState([
    {
      title: 'Active Projects',
      value: '0',
      icon: <DashboardIcon />,
      color: 'primary.main',
    },
    {
      title: 'Total Robots',
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
            title: 'Total Robots',
            value: '12',
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
  }, [user?.role]); // Only depend on user role, not the functions

  const handleRetry = () => {
    setLoading(true);
    setError(null);
    // Trigger reload
    window.location.reload();
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
      <Box>
        <Typography variant="h4" component="h1" gutterBottom>
          Welcome back, {user?.username}!
        </Typography>
        <Typography variant="body1" color="text.secondary" sx={{ mb: 4 }}>
          Manage your robotics projects and monitor your robots from this
          dashboard.
        </Typography>

        <Box
          sx={{
            display: 'grid',
            gridTemplateColumns: {
              xs: '1fr',
              sm: 'repeat(2, 1fr)',
              md: 'repeat(3, 1fr)',
            },
            gap: 3,
            mb: 4,
          }}
        >
          {stats.map((stat) => (
            <Card key={stat.title}>
              <CardContent>
                <Box
                  display="flex"
                  alignItems="center"
                  justifyContent="space-between"
                >
                  <Box>
                    <Typography color="textSecondary" gutterBottom>
                      {stat.title}
                    </Typography>
                    <Typography variant="h4" component="div">
                      {stat.value}
                    </Typography>
                  </Box>
                  <Box
                    sx={{
                      color: stat.color,
                      display: 'flex',
                      alignItems: 'center',
                    }}
                  >
                    {stat.icon}
                  </Box>
                </Box>
              </CardContent>
            </Card>
          ))}
        </Box>

        <Box
          sx={{
            display: 'grid',
            gridTemplateColumns: { xs: '1fr', md: '2fr 1fr' },
            gap: 3,
          }}
        >
          <Paper sx={{ p: 3 }}>
            <Typography variant="h6" gutterBottom>
              Recent Activity
            </Typography>
            <Typography variant="body2" color="text.secondary">
              No recent activity to display. Start by creating your first
              robotics project!
            </Typography>
          </Paper>
          <Paper sx={{ p: 3 }}>
            <Typography variant="h6" gutterBottom>
              Quick Actions
            </Typography>
            <Typography variant="body2" color="text.secondary">
              Quick action buttons will be available here once you have projects
              set up.
            </Typography>
          </Paper>
        </Box>
      </Box>
    </DataFetchWrapper>
  );
};

export default Dashboard;
