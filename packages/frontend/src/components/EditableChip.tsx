import React, { useState, useRef, useEffect } from 'react';
import { Chip, TextField, Box, IconButton, Tooltip } from '@mui/material';
import {
  Edit as EditIcon,
  Delete as DeleteIcon,
  Check as CheckIcon,
  Close as CloseIcon,
} from '@mui/icons-material';

interface EditableChipProps {
  value: string;
  displayName: string;
  description?: string;
  onEdit: () => void;
  disabled?: boolean;
  color?: 'default' | 'primary' | 'secondary' | 'error' | 'info' | 'success' | 'warning';
  variant?: 'filled' | 'outlined';
  size?: 'small' | 'medium';
}

const EditableChip: React.FC<EditableChipProps> = ({
  value,
  displayName,
  description,
  onEdit,
  disabled = false,
  color = 'default',
  variant = 'outlined',
  size = 'medium',
}) => {
  const handleChipClick = () => {
    if (!disabled) {
      onEdit();
    }
  };

  return (
    <Tooltip
      title={
        description ? `${displayName} (${value}): ${description}` : `${displayName} (${value})`
      }
      placement="top"
    >
      <Chip
        label={displayName}
        color={color}
        variant={variant}
        size={size}
        onClick={handleChipClick}
        disabled={disabled}
        sx={{
          cursor: disabled ? 'default' : 'pointer',
          '&:hover': {
            backgroundColor: disabled ? undefined : 'action.hover',
          },
        }}
      />
    </Tooltip>
  );
};

export default EditableChip;
