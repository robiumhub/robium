import React, { useState, useEffect } from 'react';
import {
  TextField,
  Button,
  Box,
  Alert,
  FormControl,
  FormHelperText,
  InputLabel,
  Select,
  MenuItem,
  Chip,
  OutlinedInput,
  SelectChangeEvent,
} from '@mui/material';
import { useAccessibility } from './AccessibilityProvider';

interface FormField {
  name: string;
  label: string;
  type: 'text' | 'email' | 'password' | 'select' | 'multiselect';
  required?: boolean;
  validation?: {
    pattern?: RegExp;
    minLength?: number;
    maxLength?: number;
    custom?: (value: string) => string | null;
  };
  options?: Array<{ value: string; label: string }>;
  placeholder?: string;
  helperText?: string;
}

interface AccessibleFormProps {
  fields: FormField[];
  onSubmit: (data: Record<string, any>) => void;
  submitLabel?: string;
  title?: string;
  description?: string;
  loading?: boolean;
  initialValues?: Record<string, any>;
}

export const AccessibleForm: React.FC<AccessibleFormProps> = ({
  fields,
  onSubmit,
  submitLabel = 'Submit',
  title,
  description,
  loading = false,
  initialValues = {},
}) => {
  // Initialize form data properly
  const getInitialFormData = () => {
    const initialData: Record<string, any> = {};
    fields.forEach((field) => {
      initialData[field.name] = initialValues[field.name] || '';
    });
    return initialData;
  };

  const [formData, setFormData] =
    useState<Record<string, any>>(getInitialFormData);
  const [errors, setErrors] = useState<Record<string, string>>({});
  const [touched, setTouched] = useState<Record<string, boolean>>({});
  const [isSubmitting, setIsSubmitting] = useState(false);
  const { announceToScreenReader } = useAccessibility();

  const validateField = (name: string, value: any): string | null => {
    const field = fields.find((f) => f.name === name);
    if (!field) return null;

    // Required validation
    if (
      field.required &&
      (!value || (Array.isArray(value) && value.length === 0))
    ) {
      return `${field.label} is required`;
    }

    if (!value) return null;

    // Type validation
    if (field.type === 'email') {
      const emailPattern = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
      if (!emailPattern.test(value)) {
        return 'Please enter a valid email address';
      }
    }

    // Pattern validation
    if (field.validation?.pattern && !field.validation.pattern.test(value)) {
      return `Please enter a valid ${field.label.toLowerCase()}`;
    }

    // Length validation
    if (
      field.validation?.minLength &&
      value.length < field.validation.minLength
    ) {
      return `${field.label} must be at least ${field.validation.minLength} characters`;
    }

    if (
      field.validation?.maxLength &&
      value.length > field.validation.maxLength
    ) {
      return `${field.label} must be no more than ${field.validation.maxLength} characters`;
    }

    // Custom validation
    if (field.validation?.custom) {
      const customError = field.validation.custom(value);
      if (customError) return customError;
    }

    return null;
  };

  const handleFieldChange = (name: string, value: any) => {
    console.log(`Field ${name} changed to:`, value);
    setFormData((prev) => ({ ...prev, [name]: value }));

    // Validate on change if field has been touched
    if (touched[name]) {
      const error = validateField(name, value);
      setErrors((prev) => ({ ...prev, [name]: error || '' }));

      // Announce error to screen reader
      if (error) {
        announceToScreenReader(
          `Error in ${fields.find((f) => f.name === name)?.label}: ${error}`,
          'assertive'
        );
      }
    }
  };

  const handleFieldBlur = (name: string) => {
    setTouched((prev) => ({ ...prev, [name]: true }));
    const value = formData[name];
    const error = validateField(name, value);
    setErrors((prev) => ({ ...prev, [name]: error || '' }));
  };

  const validateForm = (): boolean => {
    const newErrors: Record<string, string> = {};
    let hasErrors = false;

    fields.forEach((field) => {
      const error = validateField(field.name, formData[field.name]);
      if (error) {
        newErrors[field.name] = error;
        hasErrors = true;
      }
    });

    setErrors(newErrors);
    setTouched((prev) => {
      const newTouched = { ...prev };
      fields.forEach((field) => {
        newTouched[field.name] = true;
      });
      return newTouched;
    });

    if (hasErrors) {
      const errorCount = Object.keys(newErrors).length;
      announceToScreenReader(
        `Form has ${errorCount} error${errorCount > 1 ? 's' : ''}. Please review and correct the highlighted fields.`,
        'assertive'
      );
    }

    return !hasErrors;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validateForm()) {
      return;
    }

    setIsSubmitting(true);
    announceToScreenReader('Submitting form...', 'polite');

    try {
      await onSubmit(formData);
      announceToScreenReader('Form submitted successfully', 'polite');
    } catch (error) {
      const errorMessage =
        error instanceof Error
          ? error.message
          : 'An error occurred while submitting the form';
      announceToScreenReader(`Error: ${errorMessage}`, 'assertive');
    } finally {
      setIsSubmitting(false);
    }
  };

  const renderField = (field: FormField) => {
    const value = formData[field.name];
    const error = errors[field.name];
    const isTouched = touched[field.name];

    const commonProps = {
      id: field.name,
      name: field.name,
      label: field.label,
      value: value,
      error: isTouched && !!error,
      helperText: isTouched && error ? error : field.helperText,
      required: field.required,
      disabled: loading || isSubmitting,
      onBlur: () => handleFieldBlur(field.name),
      'aria-describedby':
        isTouched && error ? `${field.name}-error` : undefined,
      'aria-invalid': isTouched && !!error,
    };

    switch (field.type) {
      case 'select':
        return (
          <FormControl
            key={field.name}
            fullWidth
            error={isTouched && !!error}
            required={field.required}
            disabled={loading || isSubmitting}
          >
            <InputLabel id={`${field.name}-label`}>{field.label}</InputLabel>
            <Select
              labelId={`${field.name}-label`}
              value={value}
              label={field.label}
              onChange={(e: SelectChangeEvent) =>
                handleFieldChange(field.name, e.target.value)
              }
              onBlur={() => handleFieldBlur(field.name)}
              aria-describedby={
                isTouched && error ? `${field.name}-error` : undefined
              }
              aria-invalid={isTouched && !!error}
            >
              {field.options?.map((option) => (
                <MenuItem key={option.value} value={option.value}>
                  {option.label}
                </MenuItem>
              ))}
            </Select>
            {isTouched && error && (
              <FormHelperText id={`${field.name}-error`} error>
                {error}
              </FormHelperText>
            )}
          </FormControl>
        );

      case 'multiselect':
        return (
          <FormControl
            key={field.name}
            fullWidth
            error={isTouched && !!error}
            required={field.required}
            disabled={loading || isSubmitting}
          >
            <InputLabel id={`${field.name}-label`}>{field.label}</InputLabel>
            <Select
              labelId={`${field.name}-label`}
              multiple
              value={Array.isArray(value) ? value : []}
              label={field.label}
              onChange={(e) => handleFieldChange(field.name, e.target.value)}
              onBlur={() => handleFieldBlur(field.name)}
              input={<OutlinedInput label={field.label} />}
              renderValue={(selected: string[]) => (
                <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 0.5 }}>
                  {selected.map((selectedValue: string) => {
                    const option = field.options?.find(
                      (opt) => opt.value === selectedValue
                    );
                    return (
                      <Chip
                        key={selectedValue}
                        label={option?.label || selectedValue}
                        size="small"
                      />
                    );
                  })}
                </Box>
              )}
              aria-describedby={
                isTouched && error ? `${field.name}-error` : undefined
              }
              aria-invalid={isTouched && !!error}
            >
              {field.options?.map((option) => (
                <MenuItem key={option.value} value={option.value}>
                  {option.label}
                </MenuItem>
              ))}
            </Select>
            {isTouched && error && (
              <FormHelperText id={`${field.name}-error`} error>
                {error}
              </FormHelperText>
            )}
          </FormControl>
        );

      default:
        return (
          <TextField
            key={field.name}
            {...commonProps}
            type={field.type}
            placeholder={field.placeholder}
            onChange={(e) => handleFieldChange(field.name, e.target.value)}
            fullWidth
            margin="normal"
          />
        );
    }
  };

  return (
    <Box
      component="form"
      onSubmit={handleSubmit}
      sx={{ width: '100%', maxWidth: 600, mx: 'auto' }}
      aria-labelledby={title ? 'form-title' : undefined}
      aria-describedby={description ? 'form-description' : undefined}
    >
      {title && (
        <Box component="h2" id="form-title" sx={{ mb: 2 }}>
          {title}
        </Box>
      )}

      {description && (
        <Box
          component="p"
          id="form-description"
          sx={{ mb: 3, color: 'text.secondary' }}
        >
          {description}
        </Box>
      )}

      {Object.keys(errors).length > 0 && (
        <Alert
          severity="error"
          sx={{ mb: 2 }}
          role="alert"
          aria-live="assertive"
        >
          Please correct the following errors:
          <ul>
            {Object.entries(errors).map(([fieldName, error]) => (
              <li key={fieldName}>
                <a
                  href={`#${fieldName}`}
                  onClick={(e) => {
                    e.preventDefault();
                    document.getElementById(fieldName)?.focus();
                  }}
                >
                  {fields.find((f) => f.name === fieldName)?.label}: {error}
                </a>
              </li>
            ))}
          </ul>
        </Alert>
      )}

      {fields.map(renderField)}

      <Button
        type="submit"
        fullWidth
        variant="contained"
        sx={{ mt: 3, mb: 2 }}
        disabled={loading || isSubmitting}
        aria-describedby={isSubmitting ? 'submitting-status' : undefined}
      >
        {isSubmitting ? 'Submitting...' : submitLabel}
      </Button>

      {isSubmitting && (
        <div
          id="submitting-status"
          aria-live="polite"
          style={{ position: 'absolute', left: '-10000px' }}
        >
          Submitting form...
        </div>
      )}
    </Box>
  );
};

export default AccessibleForm;
