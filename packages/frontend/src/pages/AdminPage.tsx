import React from 'react';
import { Typography, Box } from '@mui/material';

const AdminPage: React.FC = () => {
  return (
    <Box>
      <Typography variant="h4" gutterBottom>
        Admin Dashboard
      </Typography>
      <Typography variant="body1" color="text.secondary">
        Admin controls will appear here.
      </Typography>
    </Box>
  );
};

export default AdminPage;
