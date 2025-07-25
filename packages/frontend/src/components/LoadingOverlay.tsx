import React from 'react';
import { Backdrop, CircularProgress, Typography, Box } from '@mui/material';

interface LoadingOverlayProps {
  open: boolean;
  message?: string;
  size?: 'small' | 'medium' | 'large';
}

const LoadingOverlay: React.FC<LoadingOverlayProps> = ({
  open,
  message = 'Loading...',
  size = 'medium',
}) => {
  const getSize = () => {
    switch (size) {
      case 'small':
        return 40;
      case 'large':
        return 80;
      default:
        return 60;
    }
  };

  return (
    <Backdrop
      sx={{
        color: '#fff',
        zIndex: (theme) => theme.zIndex.drawer + 1,
        flexDirection: 'column',
        gap: 2,
      }}
      open={open}
    >
      <CircularProgress color="inherit" size={getSize()} />
      {message && (
        <Typography
          variant="body1"
          sx={{
            color: 'inherit',
            textAlign: 'center',
            maxWidth: '300px',
          }}
        >
          {message}
        </Typography>
      )}
    </Backdrop>
  );
};

export default LoadingOverlay;
