import React from 'react';
import {
  Box,
  Typography,
  Button,
  Alert,
} from '@mui/material';
import {
  ArrowBack as ArrowBackIcon,
} from '@mui/icons-material';
import { useParams, useNavigate } from 'react-router-dom';

const ProjectEditPage: React.FC = () => {
  const { projectId } = useParams<{ projectId: string }>();
  const navigate = useNavigate();

  const handleBack = () => {
    navigate(`/projects/${projectId}`);
  };

  return (
    <Box sx={{ p: 3 }}>
      <Box sx={{ display: 'flex', alignItems: 'center', mb: 3 }}>
        <Button
          startIcon={<ArrowBackIcon />}
          onClick={handleBack}
          sx={{ mr: 2 }}
        >
          Back to Project
        </Button>
        <Typography variant="h4" sx={{ flexGrow: 1 }}>
          Edit Project {projectId}
        </Typography>
      </Box>

      <Alert severity="info" sx={{ mb: 3 }}>
        Project edit functionality will be implemented here.
      </Alert>

      <Typography variant="body1">
        This page will contain the project editing form and configuration options.
      </Typography>
    </Box>
  );
};

export default ProjectEditPage;
