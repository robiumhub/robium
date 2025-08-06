import React from 'react';
import {
  Chip,
  CircularProgress,
  LinearProgress,
  Tooltip,
  Box,
  Typography,
  IconButton,
  Badge,
} from '@mui/material';
import {
  CheckCircle,
  Error,
  Warning,
  Info,
  Pending,
  Stop,
  PlayArrow,
  Pause,
  Refresh,
  Notifications,
} from '@mui/icons-material';

export type StatusType = 'success' | 'error' | 'warning' | 'info' | 'pending' | 'stopped' | 'running' | 'paused';

export interface StatusIndicatorProps {
  status: StatusType;
  label?: string;
  message?: string;
  progress?: number;
  showProgress?: boolean;
  showIcon?: boolean;
  showBadge?: boolean;
  badgeContent?: number | string;
  size?: 'small' | 'medium' | 'large';
  variant?: 'chip' | 'progress' | 'icon' | 'badge' | 'text';
  onClick?: () => void;
  onRefresh?: () => void;
  className?: string;
  style?: React.CSSProperties;
}

const statusConfig = {
  success: {
    color: 'success' as const,
    icon: CheckCircle,
    label: 'Success',
    bgColor: '#e8f5e8',
    textColor: '#2e7d32',
  },
  error: {
    color: 'error' as const,
    icon: Error,
    label: 'Error',
    bgColor: '#ffebee',
    textColor: '#c62828',
  },
  warning: {
    color: 'warning' as const,
    icon: Warning,
    label: 'Warning',
    bgColor: '#fff3e0',
    textColor: '#ef6c00',
  },
  info: {
    color: 'info' as const,
    icon: Info,
    label: 'Info',
    bgColor: '#e3f2fd',
    textColor: '#1565c0',
  },
  pending: {
    color: 'default' as const,
    icon: Pending,
    label: 'Pending',
    bgColor: '#f5f5f5',
    textColor: '#757575',
  },
  stopped: {
    color: 'default' as const,
    icon: Stop,
    label: 'Stopped',
    bgColor: '#fafafa',
    textColor: '#9e9e9e',
  },
  running: {
    color: 'success' as const,
    icon: PlayArrow,
    label: 'Running',
    bgColor: '#e8f5e8',
    textColor: '#2e7d32',
  },
  paused: {
    color: 'warning' as const,
    icon: Pause,
    label: 'Paused',
    bgColor: '#fff3e0',
    textColor: '#ef6c00',
  },
};

export const StatusIndicator: React.FC<StatusIndicatorProps> = ({
  status,
  label,
  message,
  progress,
  showProgress = false,
  showIcon = true,
  showBadge = false,
  badgeContent,
  size = 'medium',
  variant = 'chip',
  onClick,
  onRefresh,
  className,
  style,
}) => {
  const config = statusConfig[status];
  const IconComponent = config.icon;

  const getSizeConfig = () => {
    switch (size) {
      case 'small':
        return { fontSize: '0.75rem', padding: '4px 8px', iconSize: 16 };
      case 'large':
        return { fontSize: '1.1rem', padding: '8px 16px', iconSize: 24 };
      default:
        return { fontSize: '0.875rem', padding: '6px 12px', iconSize: 20 };
    }
  };

  const sizeConfig = getSizeConfig();

  const renderChip = () => (
    <Chip
      icon={showIcon ? <IconComponent style={{ fontSize: sizeConfig.iconSize }} /> : undefined}
      label={label || config.label}
      color={config.color}
      size={size}
      variant="outlined"
      onClick={onClick}
      style={{
        backgroundColor: config.bgColor,
        color: config.textColor,
        borderColor: config.textColor,
        fontSize: sizeConfig.fontSize,
        padding: sizeConfig.padding,
        ...style,
      }}
      className={className}
    />
  );

  const renderProgress = () => (
    <Box sx={{ width: '100%', position: 'relative' }}>
      <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
        {showIcon && (
          <IconComponent
            style={{
              fontSize: sizeConfig.iconSize,
              color: config.textColor,
              marginRight: 8,
            }}
          />
        )}
        <Typography
          variant="body2"
          style={{ color: config.textColor, fontSize: sizeConfig.fontSize }}
        >
          {label || config.label}
        </Typography>
        {onRefresh && (
          <IconButton
            size="small"
            onClick={onRefresh}
            style={{ marginLeft: 'auto', color: config.textColor }}
          >
            <Refresh style={{ fontSize: sizeConfig.iconSize }} />
          </IconButton>
        )}
      </Box>
      <LinearProgress
        variant="determinate"
        value={progress || 0}
        sx={{
          height: 6,
          borderRadius: 3,
          backgroundColor: '#e0e0e0',
          '& .MuiLinearProgress-bar': {
            backgroundColor: config.textColor,
          },
        }}
      />
      {message && (
        <Typography
          variant="caption"
          style={{ color: config.textColor, fontSize: sizeConfig.fontSize * 0.8 }}
          sx={{ mt: 0.5, display: 'block' }}
        >
          {message}
        </Typography>
      )}
    </Box>
  );

  const renderIcon = () => (
    <Tooltip title={message || label || config.label}>
      <Box
        sx={{
          display: 'inline-flex',
          alignItems: 'center',
          cursor: onClick ? 'pointer' : 'default',
        }}
        onClick={onClick}
        className={className}
        style={style}
      >
        <IconComponent
          style={{
            fontSize: sizeConfig.iconSize,
            color: config.textColor,
          }}
        />
        {label && (
          <Typography
            variant="body2"
            style={{
              color: config.textColor,
              fontSize: sizeConfig.fontSize,
              marginLeft: 4,
            }}
          >
            {label}
          </Typography>
        )}
      </Box>
    </Tooltip>
  );

  const renderBadge = () => (
    <Badge
      badgeContent={badgeContent}
      color={config.color}
      sx={{
        '& .MuiBadge-badge': {
          backgroundColor: config.textColor,
          color: config.bgColor,
        },
      }}
    >
      <IconComponent
        style={{
          fontSize: sizeConfig.iconSize,
          color: config.textColor,
        }}
      />
    </Badge>
  );

  const renderText = () => (
    <Typography
      variant="body2"
      style={{
        color: config.textColor,
        fontSize: sizeConfig.fontSize,
        backgroundColor: config.bgColor,
        padding: sizeConfig.padding,
        borderRadius: 4,
        display: 'inline-flex',
        alignItems: 'center',
        cursor: onClick ? 'pointer' : 'default',
      }}
      onClick={onClick}
      className={className}
    >
      {showIcon && (
        <IconComponent
          style={{
            fontSize: sizeConfig.iconSize,
            marginRight: 4,
          }}
        />
      )}
      {label || config.label}
    </Typography>
  );

  const renderContent = () => {
    switch (variant) {
      case 'progress':
        return renderProgress();
      case 'icon':
        return renderIcon();
      case 'badge':
        return renderBadge();
      case 'text':
        return renderText();
      default:
        return renderChip();
    }
  };

  return <>{renderContent()}</>;
};

// Specialized status indicators for common use cases
export const ContainerStatusIndicator: React.FC<{
  status: 'running' | 'stopped' | 'starting' | 'stopping' | 'error' | 'paused';
  containerName: string;
  health?: 'healthy' | 'unhealthy' | 'starting' | 'unknown';
  onClick?: () => void;
}> = ({ status, containerName, health, onClick }) => {
  const getStatusType = (): StatusType => {
    switch (status) {
      case 'running':
        return health === 'unhealthy' ? 'warning' : 'running';
      case 'stopped':
        return 'stopped';
      case 'starting':
        return 'pending';
      case 'stopping':
        return 'pending';
      case 'error':
        return 'error';
      case 'paused':
        return 'paused';
      default:
        return 'info';
    }
  };

  const getHealthMessage = () => {
    if (status !== 'running') return undefined;
    switch (health) {
      case 'healthy':
        return 'Container is healthy';
      case 'unhealthy':
        return 'Container health check failed';
      case 'starting':
        return 'Health check in progress';
      case 'unknown':
        return 'Health status unknown';
      default:
        return undefined;
    }
  };

  return (
    <StatusIndicator
      status={getStatusType()}
      label={containerName}
      message={getHealthMessage()}
      variant="chip"
      onClick={onClick}
    />
  );
};

export const SystemHealthIndicator: React.FC<{
  health: 'healthy' | 'warning' | 'critical' | 'offline';
  systemName: string;
  details?: string;
  metrics?: {
    cpu: number;
    memory: number;
    disk: number;
  };
}> = ({ health, systemName, details, metrics }) => {
  const getStatusType = (): StatusType => {
    switch (health) {
      case 'healthy':
        return 'success';
      case 'warning':
        return 'warning';
      case 'critical':
        return 'error';
      case 'offline':
        return 'stopped';
      default:
        return 'info';
    }
  };

  const getMessage = () => {
    if (details) return details;
    if (metrics) {
      return `CPU: ${metrics.cpu}%, Memory: ${metrics.memory}%, Disk: ${metrics.disk}%`;
    }
    return undefined;
  };

  return (
    <StatusIndicator
      status={getStatusType()}
      label={systemName}
      message={getMessage()}
      variant="progress"
      progress={metrics ? Math.max(metrics.cpu, metrics.memory, metrics.disk) : undefined}
      showProgress={!!metrics}
    />
  );
};

export const ProgressIndicator: React.FC<{
  progress: number;
  label: string;
  message?: string;
  status?: StatusType;
  onCancel?: () => void;
}> = ({ progress, label, message, status = 'pending', onCancel }) => {
  return (
    <StatusIndicator
      status={status}
      label={label}
      message={message}
      progress={progress}
      showProgress={true}
      variant="progress"
    />
  );
};

export default StatusIndicator; 