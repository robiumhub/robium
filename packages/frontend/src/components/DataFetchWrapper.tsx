import React, { ReactNode } from 'react';
import { Box } from '@mui/material';
import LoadingSpinner from './LoadingSpinner';
import ErrorMessage from './ErrorMessage';
import { DashboardSkeleton } from './Skeleton';

interface DataFetchWrapperProps {
  loading: boolean;
  error?: string | null;
  errorTitle?: string;
  errorDetails?: string;
  onRetry?: () => void;
  children: ReactNode;
  loadingMessage?: string;
  showSkeleton?: boolean;
  skeletonType?: 'dashboard' | 'spinner';
  minHeight?: string | number;
}

const DataFetchWrapper: React.FC<DataFetchWrapperProps> = ({
  loading,
  error,
  errorTitle,
  errorDetails,
  onRetry,
  children,
  loadingMessage = 'Loading data...',
  showSkeleton = false,
  skeletonType = 'spinner',
  minHeight = '200px',
}) => {
  if (loading) {
    if (showSkeleton && skeletonType === 'dashboard') {
      return (
        <Box sx={{ minHeight }}>
          <DashboardSkeleton />
        </Box>
      );
    }

    return (
      <Box
        sx={{
          minHeight,
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
        }}
      >
        <LoadingSpinner message={loadingMessage} />
      </Box>
    );
  }

  if (error) {
    return (
      <Box
        sx={{
          minHeight,
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
        }}
      >
        <ErrorMessage
          title={errorTitle}
          message={error}
          details={errorDetails}
          onRetry={onRetry}
          fullWidth
        />
      </Box>
    );
  }

  return <>{children}</>;
};

export default DataFetchWrapper;
