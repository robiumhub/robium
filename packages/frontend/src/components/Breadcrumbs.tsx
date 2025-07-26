import React from 'react';
import {
  Breadcrumbs as MuiBreadcrumbs,
  Link,
  Typography,
  Box,
} from '@mui/material';
import { Link as RouterLink } from 'react-router-dom';
import { NavigateNext as NavigateNextIcon } from '@mui/icons-material';
import { useNavigation } from '../contexts/NavigationContext';

const Breadcrumbs: React.FC = () => {
  const { breadcrumbs } = useNavigation();

  if (breadcrumbs.length <= 1) {
    return null;
  }

  return (
    <Box sx={{ mb: 3 }}>
      <MuiBreadcrumbs
        separator={<NavigateNextIcon fontSize="small" />}
        aria-label="breadcrumb"
        sx={{
          '& .MuiBreadcrumbs-separator': {
            mx: { xs: 0.5, sm: 1 },
          },
        }}
      >
        {breadcrumbs.map((breadcrumb, index) => {
          const isLast = index === breadcrumbs.length - 1;

          if (isLast) {
            return (
              <Typography
                key={breadcrumb.label}
                color="text.primary"
                variant="body2"
                sx={{
                  fontSize: { xs: '0.875rem', sm: '1rem' },
                }}
              >
                {breadcrumb.label}
              </Typography>
            );
          }

          return (
            <Link
              key={breadcrumb.label}
              component={RouterLink}
              to={breadcrumb.path || '#'}
              color="inherit"
              underline="hover"
              variant="body2"
              sx={{
                fontSize: { xs: '0.875rem', sm: '1rem' },
                '&:hover': {
                  color: 'primary.main',
                },
              }}
            >
              {breadcrumb.label}
            </Link>
          );
        })}
      </MuiBreadcrumbs>
    </Box>
  );
};

export default Breadcrumbs;
