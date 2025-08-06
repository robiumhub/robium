import { useState, useEffect, useCallback, useRef, useMemo } from 'react';
import {
  ValidationError,
  ValidationResult,
  ValidationRule,
  createValidationDebouncer,
  validateField,
  validateConfiguration,
  validateDependencies,
  getErrorSuggestions
} from '../utils/validation';

export interface ValidationState {
  isValid: boolean;
  errors: ValidationError[];
  warnings: ValidationError[];
  info: ValidationError[];
  isValidationInProgress: boolean;
  lastValidated: Date | null;
}

export interface UseValidationOptions {
  debounceDelay?: number;
  validateOnChange?: boolean;
  validateOnBlur?: boolean;
  validateOnMount?: boolean;
  dependencies?: Record<string, string[]>;
}

export interface UseValidationReturn {
  validationState: ValidationState;
  validate: (value: any, rules?: ValidationRule[]) => ValidationResult;
  validateConfiguration: (content: string, format: 'json' | 'yaml', schema?: any) => ValidationResult;
  validateForm: (data: Record<string, any>, fieldValidations: Record<string, ValidationRule[]>) => ValidationResult;
  clearErrors: () => void;
  getFieldErrors: (fieldName: string) => ValidationError[];
  getErrorSuggestions: (error: ValidationError) => string[];
  setValidationState: (state: Partial<ValidationState>) => void;
  // Additional helper methods
  debouncedValidate: (validationFn: () => ValidationResult) => void;
  createFieldValidator: (fieldName: string, rules: ValidationRule[], value: any) => () => ValidationResult;
  createFormValidator: (data: Record<string, any>, fieldValidations: Record<string, ValidationRule[]>) => () => ValidationResult;
  createConfigValidator: (content: string, format: 'json' | 'yaml', schema?: any) => () => ValidationResult;
}

export const useValidation = (options: UseValidationOptions = {}): UseValidationReturn => {
  const {
    debounceDelay = 300,
    validateOnChange = true,
    validateOnBlur = true,
    validateOnMount = false,
    dependencies = {}
  } = options;

  const [validationState, setValidationState] = useState<ValidationState>({
    isValid: true,
    errors: [],
    warnings: [],
    info: [],
    isValidationInProgress: false,
    lastValidated: null
  });

  const debouncedValidation = useRef(createValidationDebouncer(debounceDelay));
  const validationTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  // Clear validation errors
  const clearErrors = useCallback(() => {
    setValidationState(prev => ({
      ...prev,
      errors: [],
      warnings: [],
      info: [],
      isValid: true
    }));
  }, []);

  // Get errors for a specific field
  const getFieldErrors = useCallback((fieldName: string): ValidationError[] => {
    return validationState.errors.filter(error => error.field === fieldName);
  }, [validationState.errors]);

  // Validate a single field
  const validate = useCallback((value: any, rules: ValidationRule[] = []): ValidationResult => {
    if (rules.length === 0) {
      return {
        isValid: true,
        errors: [],
        warnings: [],
        info: []
      };
    }

    const errors = validateField(value, rules, 'field');
    
    return {
      isValid: errors.length === 0,
      errors: errors.filter(e => e.severity === 'error'),
      warnings: errors.filter(e => e.severity === 'warning'),
      info: errors.filter(e => e.severity === 'info')
    };
  }, []);

  // Validate configuration (JSON/YAML)
  const validateConfig = useCallback((content: string, format: 'json' | 'yaml', schema?: any): ValidationResult => {
    return validateConfiguration(content, format, schema);
  }, []);

  // Validate entire form
  const validateForm = useCallback((
    data: Record<string, any>, 
    fieldValidations: Record<string, ValidationRule[]>
  ): ValidationResult => {
    const allErrors: ValidationError[] = [];
    const allWarnings: ValidationError[] = [];
    const allInfo: ValidationError[] = [];

    // Validate each field
    Object.entries(fieldValidations).forEach(([fieldName, rules]) => {
      const fieldValue = data[fieldName];
      const fieldErrors = validateField(fieldValue, rules, fieldName);
      
      allErrors.push(...fieldErrors.filter(e => e.severity === 'error'));
      allWarnings.push(...fieldErrors.filter(e => e.severity === 'warning'));
      allInfo.push(...fieldErrors.filter(e => e.severity === 'info'));
    });

    // Validate dependencies
    if (Object.keys(dependencies).length > 0) {
      const dependencyResult = validateDependencies(data, dependencies);
      allErrors.push(...dependencyResult.errors);
      allWarnings.push(...dependencyResult.warnings);
      allInfo.push(...dependencyResult.info);
    }

    return {
      isValid: allErrors.length === 0,
      errors: allErrors,
      warnings: allWarnings,
      info: allInfo
    };
  }, [dependencies]);

  // Ref to prevent infinite validation loops
  const validationInProgressRef = useRef(false);
  
  // Debounced validation function
  const debouncedValidate = useCallback((validationFn: () => ValidationResult) => {
    // Prevent multiple simultaneous validations
    if (validationInProgressRef.current) return;
    
    validationInProgressRef.current = true;
    setValidationState(prev => ({ ...prev, isValidationInProgress: true }));

    debouncedValidation.current(() => {
      try {
        const result = validationFn();
        setValidationState({
          isValid: result.isValid,
          errors: result.errors,
          warnings: result.warnings,
          info: result.info,
          isValidationInProgress: false,
          lastValidated: new Date()
        });
        validationInProgressRef.current = false;
      } catch (error) {
        console.error('Validation error:', error);
        setValidationState(prev => ({
          ...prev,
          isValidationInProgress: false,
          lastValidated: new Date()
        }));
      } finally {
        validationInProgressRef.current = false;
      }
    });
  }, [setValidationState]);

  // Create field validation handler
  const createFieldValidator = useCallback((
    fieldName: string,
    rules: ValidationRule[],
    value: any
  ) => {
    return () => validate(value, rules);
  }, [validate]);

  // Create form validation handler
  const createFormValidator = useCallback((
    data: Record<string, any>,
    fieldValidations: Record<string, ValidationRule[]>
  ) => {
    return () => validateForm(data, fieldValidations);
  }, [validateForm]);

  // Create configuration validation handler
  const createConfigValidator = useCallback((
    content: string,
    format: 'json' | 'yaml',
    schema?: any
  ) => {
    return () => validateConfig(content, format, schema);
  }, [validateConfig]);

  // Update validation state
  const updateValidationState = useCallback((updates: Partial<ValidationState>) => {
    setValidationState(prev => ({ ...prev, ...updates }));
  }, []);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (validationTimeoutRef.current) {
        clearTimeout(validationTimeoutRef.current);
      }
    };
  }, []);

  return {
    validationState,
    validate,
    validateConfiguration: validateConfig,
    validateForm,
    clearErrors,
    getFieldErrors,
    getErrorSuggestions,
    setValidationState: updateValidationState,
    // Additional helper methods
    debouncedValidate,
    createFieldValidator,
    createFormValidator,
    createConfigValidator
  };
};

// Hook for real-time field validation
export const useFieldValidation = (
  fieldName: string,
  rules: ValidationRule[],
  value: any,
  options: UseValidationOptions = {}
) => {
  const validation = useValidation(options);
  const { debouncedValidate, createFieldValidator } = validation;

  const mountedRef = useRef(false);
  
  useEffect(() => {
    // Skip validation on initial mount
    if (!mountedRef.current) {
      mountedRef.current = true;
      return;
    }
    
    if (options.validateOnChange && value !== undefined) {
      const validator = createFieldValidator(fieldName, rules, value);
      debouncedValidate(validator);
    }
  }, [value, fieldName, JSON.stringify(rules), options.validateOnChange]);

  const fieldErrors = validation.getFieldErrors(fieldName);
  const hasErrors = fieldErrors.length > 0;
  const errorMessage = hasErrors ? fieldErrors[0].message : '';

  return {
    ...validation,
    fieldErrors,
    hasErrors,
    errorMessage,
    isValid: !hasErrors
  };
};

// Hook for real-time form validation
export const useFormValidation = (
  data: Record<string, any>,
  fieldValidations: Record<string, ValidationRule[]>,
  options: UseValidationOptions = {}
) => {
  const validation = useValidation(options);
  const { debouncedValidate, createFormValidator } = validation;

  // Memoize the data and field validations to prevent infinite loops
  const memoizedData = useMemo(() => data, [JSON.stringify(data)]);
  const memoizedFieldValidations = useMemo(() => fieldValidations, [JSON.stringify(fieldValidations)]);
  const formMountedRef = useRef(false);

  useEffect(() => {
    // Skip validation on initial mount
    if (!formMountedRef.current) {
      formMountedRef.current = true;
      return;
    }
    
    if (options.validateOnChange && Object.keys(memoizedData).length > 0) {
      const validator = createFormValidator(memoizedData, memoizedFieldValidations);
      debouncedValidate(validator);
    }
  }, [memoizedData, memoizedFieldValidations, options.validateOnChange, debouncedValidate, createFormValidator]);

  return {
    ...validation,
    formErrors: validation.validationState.errors,
    formWarnings: validation.validationState.warnings,
    formInfo: validation.validationState.info,
    hasFormErrors: validation.validationState.errors.length > 0,
    hasFormWarnings: validation.validationState.warnings.length > 0,
    hasFormInfo: validation.validationState.info.length > 0
  };
};

// Hook for real-time configuration validation
export const useConfigurationValidation = (
  content: string,
  format: 'json' | 'yaml',
  schema?: any,
  options: UseValidationOptions = {}
) => {
  const validation = useValidation(options);
  const { debouncedValidate, createConfigValidator } = validation;

  useEffect(() => {
    if (options.validateOnChange && content !== undefined) {
      const validator = createConfigValidator(content, format, schema);
      debouncedValidate(validator);
    }
  }, [content, format, JSON.stringify(schema), options.validateOnChange]);

  return {
    ...validation,
    configErrors: validation.validationState.errors,
    configWarnings: validation.validationState.warnings,
    configInfo: validation.validationState.info,
    hasConfigErrors: validation.validationState.errors.length > 0,
    hasConfigWarnings: validation.validationState.warnings.length > 0,
    hasConfigInfo: validation.validationState.info.length > 0
  };
}; 