import React from 'react';
import {
  Dialog,
  DialogProps,
  DialogTitle,
  DialogContent,
  DialogActions,
  DialogContentText,
  IconButton,
  Typography,
  Box,
  Divider,
} from '@mui/material';
import {
  Close as CloseIcon,
  Warning as WarningIcon,
  Info as InfoIcon,
  Error as ErrorIcon,
  CheckCircle as SuccessIcon,
} from '@mui/icons-material';

// Import button components (these should be available from the Button component)
import { PrimaryButton, SecondaryButton } from './Button';

// Extended modal props
export interface ModalProps extends Omit<DialogProps, 'variant'> {
  variant?: 'default' | 'alert' | 'confirm' | 'form';
  size?: 'small' | 'medium' | 'large' | 'full';
  title?: string;
  subtitle?: string;
  description?: string;
  icon?: 'warning' | 'info' | 'error' | 'success' | 'none';
  showCloseButton?: boolean;
  onClose?: () => void;
  actions?: React.ReactNode;
  children?: React.ReactNode;
}

// Icon mapping
const iconMap = {
  warning: WarningIcon,
  info: InfoIcon,
  error: ErrorIcon,
  success: SuccessIcon,
  none: null,
};

// Get modal size
const getModalSize = (size: string) => {
  switch (size) {
    case 'small':
      return 'xs';
    case 'large':
      return 'lg';
    case 'full':
      return false;
    default:
      return 'md';
  }
};

// Get icon color
const getIconColor = (icon: string) => {
  switch (icon) {
    case 'warning':
      return 'warning.main';
    case 'info':
      return 'info.main';
    case 'error':
      return 'error.main';
    case 'success':
      return 'success.main';
    default:
      return 'primary.main';
  }
};

// Modal component
const Modal: React.FC<ModalProps> = ({
  variant = 'default',
  size = 'medium',
  title,
  subtitle,
  description,
  icon = 'none',
  showCloseButton = true,
  onClose,
  actions,
  children,
  open,
  ...props
}) => {
  const IconComponent = iconMap[icon];
  const maxWidth = getModalSize(size);
  const fullWidth = size === 'full';

  const handleClose = (
    event: React.MouseEvent<HTMLElement>,
    reason: string
  ) => {
    if (reason === 'backdropClick' && variant === 'alert') {
      return; // Prevent closing on backdrop click for alerts
    }
    onClose?.();
  };

  return (
    <Dialog
      open={open}
      onClose={handleClose}
      maxWidth={maxWidth}
      fullWidth={fullWidth}
      fullScreen={size === 'full'}
      sx={{
        '& .MuiDialog-paper': {
          borderRadius: size === 'full' ? 0 : 2,
        },
      }}
      {...props}
    >
      {/* Dialog Header */}
      {(title || subtitle || icon !== 'none') && (
        <>
          <DialogTitle
            sx={{
              display: 'flex',
              alignItems: 'center',
              gap: 2,
              pb: 1,
            }}
          >
            {IconComponent && (
              <IconComponent
                sx={{
                  color: getIconColor(icon),
                  fontSize: 24,
                }}
              />
            )}
            <Box sx={{ flexGrow: 1 }}>
              {title && (
                <Typography variant="h6" component="div">
                  {title}
                </Typography>
              )}
              {subtitle && (
                <Typography variant="body2" color="text.secondary">
                  {subtitle}
                </Typography>
              )}
            </Box>
            {showCloseButton && (
              <IconButton
                aria-label="close"
                onClick={onClose}
                sx={{
                  color: 'text.secondary',
                  '&:hover': {
                    color: 'text.primary',
                  },
                }}
              >
                <CloseIcon />
              </IconButton>
            )}
          </DialogTitle>
          <Divider />
        </>
      )}

      {/* Dialog Content */}
      <DialogContent sx={{ pt: 2 }}>
        {description && (
          <DialogContentText sx={{ mb: 2 }}>{description}</DialogContentText>
        )}
        {children}
      </DialogContent>

      {/* Dialog Actions */}
      {actions && (
        <>
          <Divider />
          <DialogActions sx={{ p: 2 }}>{actions}</DialogActions>
        </>
      )}
    </Dialog>
  );
};

// Alert Modal
export const AlertModal: React.FC<
  Omit<ModalProps, 'variant' | 'icon'> & {
    severity?: 'warning' | 'info' | 'error' | 'success';
    onConfirm?: () => void;
    confirmText?: string;
  }
> = ({
  severity = 'info',
  onConfirm,
  confirmText = 'OK',
  onClose,
  ...props
}) => {
  const handleConfirm = () => {
    onConfirm?.();
    onClose?.();
  };

  return (
    <Modal
      variant="alert"
      icon={severity}
      actions={
        <Box sx={{ display: 'flex', gap: 1 }}>
          <PrimaryButton onClick={handleConfirm}>{confirmText}</PrimaryButton>
        </Box>
      }
      onClose={onClose}
      {...props}
    />
  );
};

// Confirm Modal
export const ConfirmModal: React.FC<
  Omit<ModalProps, 'variant' | 'icon'> & {
    severity?: 'warning' | 'info' | 'error' | 'success';
    onConfirm?: () => void;
    onCancel?: () => void;
    confirmText?: string;
    cancelText?: string;
    destructive?: boolean;
  }
> = ({
  severity = 'warning',
  onConfirm,
  onCancel,
  confirmText = 'Confirm',
  cancelText = 'Cancel',
  destructive = false,
  onClose,
  ...props
}) => {
  const handleConfirm = () => {
    onConfirm?.();
    onClose?.();
  };

  const handleCancel = () => {
    onCancel?.();
    onClose?.();
  };

  return (
    <Modal
      variant="confirm"
      icon={severity}
      actions={
        <Box sx={{ display: 'flex', gap: 1 }}>
          <SecondaryButton onClick={handleCancel}>{cancelText}</SecondaryButton>
          <PrimaryButton
            onClick={handleConfirm}
            color={destructive ? 'error' : 'primary'}
          >
            {confirmText}
          </PrimaryButton>
        </Box>
      }
      onClose={onClose}
      {...props}
    />
  );
};

// Form Modal
export const FormModal: React.FC<
  Omit<ModalProps, 'variant'> & {
    onSave?: () => void;
    onCancel?: () => void;
    saveText?: string;
    cancelText?: string;
    loading?: boolean;
  }
> = ({
  onSave,
  onCancel,
  saveText = 'Save',
  cancelText = 'Cancel',
  loading = false,
  onClose,
  ...props
}) => {
  const handleSave = () => {
    onSave?.();
  };

  const handleCancel = () => {
    onCancel?.();
    onClose?.();
  };

  return (
    <Modal
      variant="form"
      size="medium"
      actions={
        <Box sx={{ display: 'flex', gap: 1 }}>
          <SecondaryButton onClick={handleCancel}>{cancelText}</SecondaryButton>
          <PrimaryButton
            onClick={handleSave}
            loading={loading}
            disabled={loading}
          >
            {saveText}
          </PrimaryButton>
        </Box>
      }
      onClose={onClose}
      {...props}
    />
  );
};

export default Modal;
