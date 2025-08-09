import React, { useEffect, useState } from 'react';
import { Box, Grid, Card, CardContent, Typography, Chip } from '@mui/material';
import { RobotsService, Robot } from '../services/robotsService';

const Robots: React.FC = () => {
  const [robots, setRobots] = useState<Robot[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const load = async () => {
      try {
        setLoading(true);
        const robotsData = await RobotsService.getRobots();
        setRobots(robotsData);
      } catch (e) {
        console.error('Failed to load robots:', e);
        setRobots([]);
      } finally {
        setLoading(false);
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
      {loading ? (
        <Typography>Loading robots...</Typography>
      ) : robots.length === 0 ? (
        <Typography color="text.secondary">No robots found.</Typography>
      ) : (
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
      )}
    </Box>
  );
};

export default Robots;
