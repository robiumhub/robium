import React from 'react';
import {
  Box,
  Typography,
  Card,
  CardContent,
  Button,
  Chip,
  Avatar,
  Divider,
} from '@mui/material';
import { useParams, Link as RouterLink } from 'react-router-dom';
import {
  ArrowBack as ArrowBackIcon,
  Edit as EditIcon,
  SmartToy as RobotIcon,
} from '@mui/icons-material';

const RobotDetails: React.FC = () => {
  const { robotId } = useParams<{ robotId: string }>();

  // Mock data - in real app this would come from API
  const robot = {
    id: robotId,
    name: 'NavBot-001',
    type: 'Navigation',
    status: 'online',
    project: 'Autonomous Navigation Robot',
    lastSeen: '2024-01-15 14:30',
    createdDate: '2023-12-01',
    batteryLevel: 85,
    location: 'Building A, Floor 2',
    firmware: 'v2.1.0',
    sensors: ['LIDAR', 'Camera', 'IMU', 'GPS'],
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
          to="/robots"
          startIcon={<ArrowBackIcon />}
          sx={{ mr: 2 }}
        >
          Back to Robots
        </Button>
        <Typography variant="h4" component="h1" sx={{ flexGrow: 1 }}>
          {robot.name}
        </Typography>
        <Button variant="outlined" startIcon={<EditIcon />} sx={{ ml: 2 }}>
          Edit Robot
        </Button>
      </Box>

      <Box sx={{ display: 'grid', gridTemplateColumns: '2fr 1fr', gap: 3 }}>
        <Box>
          <Card sx={{ mb: 3 }}>
            <CardContent>
              <Box sx={{ display: 'flex', alignItems: 'center', mb: 3 }}>
                <Avatar
                  sx={{ mr: 2, bgcolor: 'primary.main', width: 64, height: 64 }}
                >
                  <RobotIcon sx={{ fontSize: 32 }} />
                </Avatar>
                <Box>
                  <Typography variant="h5" gutterBottom>
                    {robot.name}
                  </Typography>
                  <Chip
                    label={robot.status}
                    color={getStatusColor(robot.status) as any}
                    size="medium"
                  />
                </Box>
              </Box>

              <Typography variant="body1" color="text.secondary" paragraph>
                Type: {robot.type}
              </Typography>
              <Typography variant="body1" color="text.secondary" paragraph>
                Project: {robot.project}
              </Typography>
              <Typography variant="body1" color="text.secondary" paragraph>
                Location: {robot.location}
              </Typography>
              <Typography variant="body1" color="text.secondary" paragraph>
                Battery Level: {robot.batteryLevel}%
              </Typography>
              <Typography variant="body1" color="text.secondary" paragraph>
                Firmware: {robot.firmware}
              </Typography>
            </CardContent>
          </Card>

          <Card>
            <CardContent>
              <Typography variant="h6" gutterBottom>
                Sensors
              </Typography>
              <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
                {robot.sensors.map((sensor) => (
                  <Chip key={sensor} label={sensor} variant="outlined" />
                ))}
              </Box>
            </CardContent>
          </Card>
        </Box>

        <Box>
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
                  to={`/robots/${robotId}/control`}
                >
                  Control Robot
                </Button>
                <Button
                  variant="outlined"
                  fullWidth
                  component={RouterLink}
                  to={`/robots/${robotId}/edit`}
                >
                  Edit Robot
                </Button>
                <Button
                  variant="outlined"
                  fullWidth
                  component={RouterLink}
                  to={`/robots/${robotId}/logs`}
                >
                  View Logs
                </Button>
                <Button
                  variant="outlined"
                  fullWidth
                  component={RouterLink}
                  to={`/robots/${robotId}/analytics`}
                >
                  Analytics
                </Button>
              </Box>
            </CardContent>
          </Card>
        </Box>
      </Box>
    </Box>
  );
};

export default RobotDetails;
