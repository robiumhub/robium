import React, { forwardRef } from 'react';
import {
  TextField,
  TextFieldProps,
  InputAdornment,
  IconButton,
  FormHelperText,
  Box,
} from '@mui/material';
import {
  Visibility as VisibilityIcon,
  VisibilityOff as VisibilityOffIcon,
  Search as SearchIcon,
  Clear as ClearIcon,
  Email as EmailIcon,
  Lock as LockIcon,
  Person as PersonIcon,
} from '@mui/icons-material';

// Extended input props
export interface InputProps extends Omit<TextFieldProps, 'variant'> {
  variant?: 'outlined' | 'filled' | 'standard';
  size?: 'small' | 'medium';
  showPasswordToggle?: boolean;
  showClearButton?: boolean;
  icon?: 'search' | 'email' | 'lock' | 'person';
  iconPosition?: 'start' | 'end';
  helperText?: string;
  error?: boolean;
  success?: boolean;
  fullWidth?: boolean;
  onClear?: () => void;
}

// Icon mapping
const iconMap = {
  search: SearchIcon,
  email: EmailIcon,
  lock: LockIcon,
  person: PersonIcon,
};

// Password input component
const PasswordInput = forwardRef<HTMLDivElement, InputProps>(
  (
    {
      showPasswordToggle = true,
      icon = 'lock',
      iconPosition = 'start',
      ...props
    },
    ref
  ) => {
    const [showPassword, setShowPassword] = React.useState(false);
    const IconComponent = iconMap[icon];

    const handleTogglePassword = () => {
      setShowPassword(!showPassword);
    };

    return (
      <TextField
        ref={ref}
        type={showPassword ? 'text' : 'password'}
        variant="outlined"
        InputProps={{
          startAdornment:
            iconPosition === 'start' && IconComponent ? (
              <InputAdornment position="start">
                <IconComponent color="action" />
              </InputAdornment>
            ) : undefined,
          endAdornment: (
            <InputAdornment position="end">
              {showPasswordToggle && (
                <IconButton
                  aria-label="toggle password visibility"
                  onClick={handleTogglePassword}
                  edge="end"
                  size="small"
                >
                  {showPassword ? <VisibilityOffIcon /> : <VisibilityIcon />}
                </IconButton>
              )}
              {iconPosition === 'end' && IconComponent && (
                <IconComponent color="action" />
              )}
            </InputAdornment>
          ),
        }}
        {...props}
      />
    );
  }
);

PasswordInput.displayName = 'PasswordInput';

// Search input component
const SearchInput = forwardRef<HTMLDivElement, InputProps>(
  (
    {
      showClearButton = true,
      icon = 'search',
      iconPosition = 'start',
      onClear,
      value,
      onChange,
      ...props
    },
    ref
  ) => {
    const IconComponent = iconMap[icon];
    const hasValue = Boolean(value);

    const handleClear = () => {
      if (onClear) {
        onClear();
      } else if (onChange) {
        // Create a synthetic event to clear the input
        const event = {
          target: { value: '' },
        } as React.ChangeEvent<HTMLInputElement>;
        onChange(event);
      }
    };

    return (
      <TextField
        ref={ref}
        variant="outlined"
        placeholder="Search..."
        InputProps={{
          startAdornment:
            iconPosition === 'start' && IconComponent ? (
              <InputAdornment position="start">
                <IconComponent color="action" />
              </InputAdornment>
            ) : undefined,
          endAdornment: (
            <InputAdornment position="end">
              {showClearButton && hasValue && (
                <IconButton
                  aria-label="clear search"
                  onClick={handleClear}
                  edge="end"
                  size="small"
                >
                  <ClearIcon />
                </IconButton>
              )}
              {iconPosition === 'end' && IconComponent && (
                <IconComponent color="action" />
              )}
            </InputAdornment>
          ),
        }}
        value={value}
        onChange={onChange}
        {...props}
      />
    );
  }
);

SearchInput.displayName = 'SearchInput';

// Enhanced text input component
const Input = forwardRef<HTMLDivElement, InputProps>(
  (
    {
      variant = 'outlined',
      size = 'medium',
      icon,
      iconPosition = 'start',
      helperText,
      error,
      success,
      fullWidth = true,
      ...props
    },
    ref
  ) => {
    const IconComponent = icon ? iconMap[icon] : null;

    return (
      <Box>
        <TextField
          ref={ref}
          variant={variant}
          size={size}
          fullWidth={fullWidth}
          error={error}
          InputProps={{
            startAdornment:
              iconPosition === 'start' && IconComponent ? (
                <InputAdornment position="start">
                  <IconComponent color="action" />
                </InputAdornment>
              ) : undefined,
            endAdornment:
              iconPosition === 'end' && IconComponent ? (
                <InputAdornment position="end">
                  <IconComponent color="action" />
                </InputAdornment>
              ) : undefined,
          }}
          sx={{
            '& .MuiOutlinedInput-root': {
              ...(success && {
                '&.Mui-focused .MuiOutlinedInput-notchedOutline': {
                  borderColor: 'success.main',
                },
                '&:hover .MuiOutlinedInput-notchedOutline': {
                  borderColor: 'success.light',
                },
              }),
            },
          }}
          {...props}
        />
        {helperText && (
          <FormHelperText
            error={error}
            sx={{
              ...(success && {
                color: 'success.main',
              }),
            }}
          >
            {helperText}
          </FormHelperText>
        )}
      </Box>
    );
  }
);

Input.displayName = 'Input';

// Specialized input components
export const EmailInput = forwardRef<HTMLDivElement, InputProps>(
  (props, ref) => (
    <Input
      ref={ref}
      type="email"
      icon="email"
      placeholder="Enter your email"
      {...props}
    />
  )
);

export const UsernameInput = forwardRef<HTMLDivElement, InputProps>(
  (props, ref) => (
    <Input
      ref={ref}
      type="text"
      icon="person"
      placeholder="Enter your username"
      {...props}
    />
  )
);

// Export components
export { PasswordInput, SearchInput };
export default Input;
