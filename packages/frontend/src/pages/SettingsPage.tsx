import React from 'react';
import { Typography, Box } from '@mui/material';

const SettingsPage: React.FC = () => {
  return (
    <Box>
      <Typography variant="h4" gutterBottom>
        Settings
      </Typography>
      <Typography variant="body1" color="text.secondary">
        Application settings will appear here.
      </Typography>
    </Box>
  );
};

export default SettingsPage;
