import React, { useEffect, useState } from 'react';
import { Box, Grid, Card, CardContent, Typography, Chip } from '@mui/material';
import { ApiService } from '../services/api';

interface RobotInfo {
  code: string;
  name: string;
  module_count: number;
}

const Robots: React.FC = () => {
  const [robots, setRobots] = useState<RobotInfo[]>([]);

  useEffect(() => {
    const load = async () => {
      try {
        const data = await ApiService.get<any[]>('/robots');
        setRobots(Array.isArray(data) ? data : []);
      } catch (e) {
        setRobots([
          { code: 'turtlebot3', name: 'TurtleBot 3', module_count: 0 },
          { code: 'turtlebot4', name: 'TurtleBot 4', module_count: 0 },
          { code: 'raspberrypi', name: 'Raspberry Pi', module_count: 0 },
          { code: 'nvidia-orin', name: 'NVIDIA Orin', module_count: 0 },
        ]);
      }
    };
    load();
  }, []);

  return (
    <Box>
      <Typography variant="h4" component="h1" gutterBottom>
        Robots
      </Typography>
      <Typography variant="body2" color="text.secondary" gutterBottom>
        Currently supported robots and their available modules
      </Typography>
      <Grid container spacing={3}>
        {robots.map((r) => (
          <Grid key={r.code} item xs={12} sm={6} md={4} lg={3}>
            <Card>
              <CardContent>
                <Typography variant="h6">{r.name}</Typography>
                <Chip
                  label={`${r.module_count} modules`}
                  size="small"
                  sx={{ mt: 1 }}
                />
              </CardContent>
            </Card>
          </Grid>
        ))}
      </Grid>
    </Box>
  );
};

export default Robots;
