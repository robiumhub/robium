import React from 'react';
import {
  Button as MuiButton,
  ButtonProps as MuiButtonProps,
  CircularProgress,
  Box,
} from '@mui/material';
import {
  Add as AddIcon,
  Edit as EditIcon,
  Delete as DeleteIcon,
  Save as SaveIcon,
  Cancel as CancelIcon,
  Refresh as RefreshIcon,
  Download as DownloadIcon,
  Upload as UploadIcon,
  Visibility as VisibilityIcon,
  VisibilityOff as VisibilityOffIcon,
} from '@mui/icons-material';

// Extended button props
export interface ButtonProps extends Omit<MuiButtonProps, 'variant'> {
  variant?: 'contained' | 'outlined' | 'text' | 'ghost';
  size?: 'small' | 'medium' | 'large';
  loading?: boolean;
  icon?:
    | 'add'
    | 'edit'
    | 'delete'
    | 'save'
    | 'cancel'
    | 'refresh'
    | 'download'
    | 'upload'
    | 'visibility'
    | 'visibilityOff';
  iconPosition?: 'start' | 'end';
  fullWidth?: boolean;
  children: React.ReactNode;
}

// Icon mapping
const iconMap = {
  add: AddIcon,
  edit: EditIcon,
  delete: DeleteIcon,
  save: SaveIcon,
  cancel: CancelIcon,
  refresh: RefreshIcon,
  download: DownloadIcon,
  upload: UploadIcon,
  visibility: VisibilityIcon,
  visibilityOff: VisibilityOffIcon,
};

// Button component
const Button: React.FC<ButtonProps> = ({
  variant = 'contained',
  size = 'medium',
  loading = false,
  icon,
  iconPosition = 'start',
  fullWidth = false,
  disabled,
  children,
  ...props
}) => {
  // Map custom variants to Material-UI variants
  const muiVariant = variant === 'ghost' ? 'text' : variant;

  // Get icon component
  const IconComponent = icon ? iconMap[icon] : null;

  // Determine if button should be disabled
  const isDisabled = disabled || loading;

  // Loading spinner size based on button size
  const getSpinnerSize = () => {
    switch (size) {
      case 'small':
        return 16;
      case 'large':
        return 24;
      default:
        return 20;
    }
  };

  return (
    <MuiButton
      variant={muiVariant}
      size={size}
      disabled={isDisabled}
      fullWidth={fullWidth}
      sx={{
        // Ghost variant styling
        ...(variant === 'ghost' && {
          backgroundColor: 'transparent',
          color: 'primary.main',
          '&:hover': {
            backgroundColor: 'primary.50',
          },
          '&:disabled': {
            backgroundColor: 'transparent',
            color: 'text.disabled',
          },
        }),
        // Loading state styling
        ...(loading && {
          '& .MuiButton-startIcon, & .MuiButton-endIcon': {
            opacity: 0,
          },
        }),
      }}
      startIcon={
        loading ? (
          <CircularProgress size={getSpinnerSize()} color="inherit" />
        ) : iconPosition === 'start' && IconComponent ? (
          <IconComponent />
        ) : undefined
      }
      endIcon={
        iconPosition === 'end' && IconComponent ? <IconComponent /> : undefined
      }
      {...props}
    >
      {children}
    </MuiButton>
  );
};

// Specialized button components
export const PrimaryButton: React.FC<Omit<ButtonProps, 'variant'>> = (
  props
) => <Button variant="contained" color="primary" {...props} />;

export const SecondaryButton: React.FC<Omit<ButtonProps, 'variant'>> = (
  props
) => <Button variant="outlined" color="primary" {...props} />;

export const GhostButton: React.FC<Omit<ButtonProps, 'variant'>> = (props) => (
  <Button variant="ghost" {...props} />
);

export const TextButton: React.FC<Omit<ButtonProps, 'variant'>> = (props) => (
  <Button variant="text" {...props} />
);

export const SuccessButton: React.FC<Omit<ButtonProps, 'variant'>> = (
  props
) => <Button variant="contained" color="success" {...props} />;

export const WarningButton: React.FC<Omit<ButtonProps, 'variant'>> = (
  props
) => <Button variant="contained" color="warning" {...props} />;

export const ErrorButton: React.FC<Omit<ButtonProps, 'variant'>> = (props) => (
  <Button variant="contained" color="error" {...props} />
);

// Action buttons
export const AddButton: React.FC<Omit<ButtonProps, 'icon' | 'variant'>> = (
  props
) => <Button icon="add" variant="contained" color="primary" {...props} />;

export const EditButton: React.FC<Omit<ButtonProps, 'icon' | 'variant'>> = (
  props
) => <Button icon="edit" variant="outlined" color="primary" {...props} />;

export const DeleteButton: React.FC<Omit<ButtonProps, 'icon' | 'variant'>> = (
  props
) => <Button icon="delete" variant="outlined" color="error" {...props} />;

export const SaveButton: React.FC<Omit<ButtonProps, 'icon' | 'variant'>> = (
  props
) => <Button icon="save" variant="contained" color="success" {...props} />;

export const CancelButton: React.FC<Omit<ButtonProps, 'icon' | 'variant'>> = (
  props
) => <Button icon="cancel" variant="outlined" color="inherit" {...props} />;

export const RefreshButton: React.FC<Omit<ButtonProps, 'icon' | 'variant'>> = (
  props
) => <Button icon="refresh" variant="outlined" color="primary" {...props} />;

export default Button;
