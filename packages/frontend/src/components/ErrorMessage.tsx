import React from 'react';
import {
  Alert,
  AlertTitle,
  Box,
  Button,
  Collapse,
  Typography,
} from '@mui/material';
import {
  Error as ErrorIcon,
  ExpandMore as ExpandMoreIcon,
  ExpandLess as ExpandLessIcon,
  Refresh as RefreshIcon,
} from '@mui/icons-material';

interface ErrorMessageProps {
  title?: string;
  message: string;
  details?: string;
  severity?: 'error' | 'warning' | 'info';
  onRetry?: () => void;
  showDetails?: boolean;
  fullWidth?: boolean;
}

const ErrorMessage: React.FC<ErrorMessageProps> = ({
  title,
  message,
  details,
  severity = 'error',
  onRetry,
  showDetails = false,
  fullWidth = false,
}) => {
  const [expanded, setExpanded] = React.useState(showDetails);

  const handleExpandClick = () => {
    setExpanded(!expanded);
  };

  const getIcon = () => {
    switch (severity) {
      case 'warning':
        return null; // Material-UI will use default warning icon
      case 'info':
        return null; // Material-UI will use default info icon
      default:
        return <ErrorIcon />;
    }
  };

  return (
    <Box sx={{ width: fullWidth ? '100%' : 'auto' }}>
      <Alert
        severity={severity}
        icon={getIcon()}
        action={
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            {details && (
              <Button
                color="inherit"
                size="small"
                onClick={handleExpandClick}
                endIcon={expanded ? <ExpandLessIcon /> : <ExpandMoreIcon />}
              >
                {expanded ? 'Hide' : 'Details'}
              </Button>
            )}
            {onRetry && (
              <Button
                color="inherit"
                size="small"
                onClick={onRetry}
                startIcon={<RefreshIcon />}
              >
                Retry
              </Button>
            )}
          </Box>
        }
      >
        {title && <AlertTitle>{title}</AlertTitle>}
        <Typography variant="body2">{message}</Typography>
      </Alert>

      {details && (
        <Collapse in={expanded} timeout="auto" unmountOnExit>
          <Box
            sx={{
              mt: 1,
              p: 2,
              bgcolor: 'grey.50',
              borderRadius: 1,
              border: '1px solid',
              borderColor: 'grey.300',
            }}
          >
            <Typography
              variant="body2"
              component="pre"
              sx={{
                fontFamily: 'monospace',
                fontSize: '0.75rem',
                whiteSpace: 'pre-wrap',
                wordBreak: 'break-word',
                color: 'text.secondary',
              }}
            >
              {details}
            </Typography>
          </Box>
        </Collapse>
      )}
    </Box>
  );
};

export default ErrorMessage;
