import React from 'react';
import {
  Box,
  Typography,
  Card,
  CardContent,
  Button,
  Chip,
  Avatar,
} from '@mui/material';
import { Add as AddIcon, SmartToy as RobotIcon } from '@mui/icons-material';
import { Link as RouterLink } from 'react-router-dom';

const Robots: React.FC = () => {
  // Mock data - in real app this would come from API
  const robots = [
    {
      id: '1',
      name: 'NavBot-001',
      type: 'Navigation',
      status: 'online',
      project: 'Autonomous Navigation Robot',
      lastSeen: '2024-01-15 14:30',
    },
    {
      id: '2',
      name: 'AssembleBot-001',
      type: 'Assembly',
      status: 'offline',
      project: 'Industrial Assembly Bot',
      lastSeen: '2024-01-14 09:15',
    },
    {
      id: '3',
      name: 'SurveillanceBot-001',
      type: 'Surveillance',
      status: 'maintenance',
      project: 'Surveillance Drone',
      lastSeen: '2024-01-13 16:45',
    },
  ];

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
      <Box
        sx={{
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
          mb: 3,
        }}
      >
        <Typography variant="h4" component="h1" gutterBottom>
          Robots
        </Typography>
        <Button
          variant="contained"
          startIcon={<AddIcon />}
          component={RouterLink}
          to="/robots/new"
        >
          Add Robot
        </Button>
      </Box>

      <Box
        sx={{
          display: 'grid',
          gridTemplateColumns: 'repeat(auto-fill, minmax(300px, 1fr))',
          gap: 3,
        }}
      >
        {robots.map((robot) => (
          <Card
            key={robot.id}
            sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}
          >
            <CardContent sx={{ flexGrow: 1 }}>
              <Box sx={{ display: 'flex', alignItems: 'center', mb: 2 }}>
                <Avatar sx={{ mr: 2, bgcolor: 'primary.main' }}>
                  <RobotIcon />
                </Avatar>
                <Box>
                  <Typography variant="h6" component="h2" gutterBottom>
                    {robot.name}
                  </Typography>
                  <Chip
                    label={robot.status}
                    color={getStatusColor(robot.status) as any}
                    size="small"
                  />
                </Box>
              </Box>
              <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
                Type: {robot.type}
              </Typography>
              <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
                Project: {robot.project}
              </Typography>
              <Typography variant="caption" color="text.secondary">
                Last seen: {robot.lastSeen}
              </Typography>
            </CardContent>
            <Box sx={{ p: 2, pt: 0 }}>
              <Button
                size="small"
                component={RouterLink}
                to={`/robots/${robot.id}`}
                fullWidth
              >
                View Details
              </Button>
            </Box>
          </Card>
        ))}
      </Box>
    </Box>
  );
};

export default Robots;
