// Validation Utilities for Real-time Form and Configuration Validation

export interface ValidationError {
  path: string;
  message: string;
  severity: 'error' | 'warning' | 'info';
  line?: number;
  column?: number;
  field?: string;
  code?: string;
}

export interface ValidationRule {
  type: 'required' | 'pattern' | 'minLength' | 'maxLength' | 'min' | 'max' | 'email' | 'url' | 'custom';
  value?: any;
  message: string;
  severity?: 'error' | 'warning' | 'info';
}

export interface FieldValidation {
  name: string;
  rules: ValidationRule[];
  dependencies?: string[];
}

export interface ValidationResult {
  isValid: boolean;
  errors: ValidationError[];
  warnings: ValidationError[];
  info: ValidationError[];
}

// JSON Validation
export const validateJSON = (content: string): ValidationResult => {
  const errors: ValidationError[] = [];
  const warnings: ValidationError[] = [];
  const info: ValidationError[] = [];

  if (!content.trim()) {
    return {
      isValid: true,
      errors: [],
      warnings: [],
      info: []
    };
  }

  try {
    const parsed = JSON.parse(content);
    
    // Check for common issues
    if (typeof parsed === 'object' && parsed !== null) {
      // Check for empty objects
      if (Object.keys(parsed).length === 0) {
        warnings.push({
          path: 'root',
          message: 'Empty JSON object detected',
          severity: 'warning',
          code: 'EMPTY_OBJECT'
        });
      }

      // Check for potential issues
      Object.entries(parsed).forEach(([key, value]) => {
        if (value === null) {
          warnings.push({
            path: key,
            message: `Property '${key}' is null, consider using undefined or removing it`,
            severity: 'warning',
            code: 'NULL_VALUE'
          });
        }
      });
    }
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : 'Invalid JSON';
    const lineMatch = errorMessage.match(/position (\d+)/);
    const line = lineMatch ? Math.floor(parseInt(lineMatch[1]) / 80) + 1 : undefined;
    
    errors.push({
      path: 'root',
      message: `JSON syntax error: ${errorMessage}`,
      severity: 'error',
      line,
      code: 'JSON_SYNTAX_ERROR'
    });
  }

  return {
    isValid: errors.length === 0,
    errors,
    warnings,
    info
  };
};

// YAML Validation (basic)
export const validateYAML = (content: string): ValidationResult => {
  const errors: ValidationError[] = [];
  const warnings: ValidationError[] = [];
  const info: ValidationError[] = [];

  if (!content.trim()) {
    return {
      isValid: true,
      errors: [],
      warnings: [],
      info: []
    };
  }

  // Basic YAML validation - check for common syntax issues
  const lines = content.split('\n');
  
  lines.forEach((line, index) => {
    const lineNumber = index + 1;
    
    // Check for tabs (YAML doesn't support tabs)
    if (line.includes('\t')) {
      errors.push({
        path: `line ${lineNumber}`,
        message: 'Tabs are not allowed in YAML, use spaces instead',
        severity: 'error',
        line: lineNumber,
        code: 'YAML_TAB_ERROR'
      });
    }

    // Check for inconsistent indentation
    if (line.trim() && !line.startsWith(' ') && !line.startsWith('-') && !line.startsWith('#')) {
      const prevLine = lines[index - 1];
      if (prevLine && prevLine.trim() && prevLine.includes(':')) {
        const expectedIndent = prevLine.match(/^(\s*)/)?.[0].length || 0;
        const actualIndent = line.match(/^(\s*)/)?.[0].length || 0;
        
        if (actualIndent > expectedIndent + 2) {
          warnings.push({
            path: `line ${lineNumber}`,
            message: 'Inconsistent indentation detected',
            severity: 'warning',
            line: lineNumber,
            code: 'YAML_INDENT_WARNING'
          });
        }
      }
    }
  });

  return {
    isValid: errors.length === 0,
    errors,
    warnings,
    info
  };
};

// Schema-based validation
export const validateAgainstSchema = (
  data: any, 
  schema: any, 
  path: string = 'root'
): ValidationResult => {
  const errors: ValidationError[] = [];
  const warnings: ValidationError[] = [];
  const info: ValidationError[] = [];

  if (!schema || !data) {
    return { isValid: true, errors, warnings, info };
  }

  // Required fields validation
  if (schema.required && Array.isArray(schema.required)) {
    schema.required.forEach((field: string) => {
      if (data[field] === undefined || data[field] === null || data[field] === '') {
        errors.push({
          path: `${path}.${field}`,
          message: `Required field '${field}' is missing`,
          severity: 'error',
          field,
          code: 'REQUIRED_FIELD_MISSING'
        });
      }
    });
  }

  // Properties validation
  if (schema.properties && typeof data === 'object') {
    Object.entries(schema.properties).forEach(([fieldName, fieldSchema]: [string, any]) => {
      const fieldValue = data[fieldName];
      const fieldPath = `${path}.${fieldName}`;

      if (fieldValue !== undefined && fieldValue !== null) {
        // Type validation
        if (fieldSchema.type) {
          const isValidType = validateType(fieldValue, fieldSchema.type);
          if (!isValidType) {
            errors.push({
              path: fieldPath,
              message: `Field '${fieldName}' should be of type '${fieldSchema.type}'`,
              severity: 'error',
              field: fieldName,
              code: 'TYPE_MISMATCH'
            });
          }
        }

        // Pattern validation
        if (fieldSchema.pattern && typeof fieldValue === 'string') {
          const regex = new RegExp(fieldSchema.pattern);
          if (!regex.test(fieldValue)) {
            errors.push({
              path: fieldPath,
              message: `Field '${fieldName}' does not match required pattern`,
              severity: 'error',
              field: fieldName,
              code: 'PATTERN_MISMATCH'
            });
          }
        }

        // Min/Max validation
        if (fieldSchema.minimum !== undefined && typeof fieldValue === 'number') {
          if (fieldValue < fieldSchema.minimum) {
            errors.push({
              path: fieldPath,
              message: `Field '${fieldName}' must be at least ${fieldSchema.minimum}`,
              severity: 'error',
              field: fieldName,
              code: 'MIN_VALUE_VIOLATION'
            });
          }
        }

        if (fieldSchema.maximum !== undefined && typeof fieldValue === 'number') {
          if (fieldValue > fieldSchema.maximum) {
            errors.push({
              path: fieldPath,
              message: `Field '${fieldName}' must be at most ${fieldSchema.maximum}`,
              severity: 'error',
              field: fieldName,
              code: 'MAX_VALUE_VIOLATION'
            });
          }
        }

        // MinLength/MaxLength validation
        if (fieldSchema.minLength !== undefined && typeof fieldValue === 'string') {
          if (fieldValue.length < fieldSchema.minLength) {
            errors.push({
              path: fieldPath,
              message: `Field '${fieldName}' must be at least ${fieldSchema.minLength} characters long`,
              severity: 'error',
              field: fieldName,
              code: 'MIN_LENGTH_VIOLATION'
            });
          }
        }

        if (fieldSchema.maxLength !== undefined && typeof fieldValue === 'string') {
          if (fieldValue.length > fieldSchema.maxLength) {
            errors.push({
              path: fieldPath,
              message: `Field '${fieldName}' must be at most ${fieldSchema.maxLength} characters long`,
              severity: 'error',
              field: fieldName,
              code: 'MAX_LENGTH_VIOLATION'
            });
          }
        }

        // Enum validation
        if (fieldSchema.enum && Array.isArray(fieldSchema.enum)) {
          if (!fieldSchema.enum.includes(fieldValue)) {
            errors.push({
              path: fieldPath,
              message: `Field '${fieldName}' must be one of: ${fieldSchema.enum.join(', ')}`,
              severity: 'error',
              field: fieldName,
              code: 'ENUM_VIOLATION'
            });
          }
        }

        // Recursive validation for nested objects
        if (fieldSchema.properties && typeof fieldValue === 'object' && !Array.isArray(fieldValue)) {
          const nestedResult = validateAgainstSchema(fieldValue, fieldSchema, fieldPath);
          errors.push(...nestedResult.errors);
          warnings.push(...nestedResult.warnings);
          info.push(...nestedResult.info);
        }

        // Array validation
        if (fieldSchema.items && Array.isArray(fieldValue)) {
          fieldValue.forEach((item: any, index: number) => {
            const itemPath = `${fieldPath}[${index}]`;
            const itemResult = validateAgainstSchema(item, fieldSchema.items, itemPath);
            errors.push(...itemResult.errors);
            warnings.push(...itemResult.warnings);
            info.push(...itemResult.info);
          });
        }
      }
    });
  }

  return {
    isValid: errors.length === 0,
    errors,
    warnings,
    info
  };
};

// Type validation helper
const validateType = (value: any, expectedType: string): boolean => {
  switch (expectedType) {
    case 'string':
      return typeof value === 'string';
    case 'number':
      return typeof value === 'number' && !isNaN(value);
    case 'integer':
      return typeof value === 'number' && Number.isInteger(value);
    case 'boolean':
      return typeof value === 'boolean';
    case 'object':
      return typeof value === 'object' && value !== null && !Array.isArray(value);
    case 'array':
      return Array.isArray(value);
    case 'null':
      return value === null;
    default:
      return true;
  }
};

// Form field validation
export const validateField = (
  value: any, 
  rules: ValidationRule[], 
  fieldName: string
): ValidationError[] => {
  const errors: ValidationError[] = [];

  rules.forEach(rule => {
    const severity = rule.severity || 'error';
    
    switch (rule.type) {
      case 'required':
        if (!value || (Array.isArray(value) && value.length === 0)) {
          errors.push({
            path: fieldName,
            message: rule.message,
            severity,
            field: fieldName,
            code: 'REQUIRED_FIELD'
          });
        }
        break;

      case 'pattern':
        if (value && rule.value && typeof value === 'string') {
          const regex = new RegExp(rule.value);
          if (!regex.test(value)) {
            errors.push({
              path: fieldName,
              message: rule.message,
              severity,
              field: fieldName,
              code: 'PATTERN_MISMATCH'
            });
          }
        }
        break;

      case 'minLength':
        if (value && typeof value === 'string' && value.length < rule.value) {
          errors.push({
            path: fieldName,
            message: rule.message,
            severity,
            field: fieldName,
            code: 'MIN_LENGTH'
          });
        }
        break;

      case 'maxLength':
        if (value && typeof value === 'string' && value.length > rule.value) {
          errors.push({
            path: fieldName,
            message: rule.message,
            severity,
            field: fieldName,
            code: 'MAX_LENGTH'
          });
        }
        break;

      case 'min':
        if (value !== undefined && value !== null && value < rule.value) {
          errors.push({
            path: fieldName,
            message: rule.message,
            severity,
            field: fieldName,
            code: 'MIN_VALUE'
          });
        }
        break;

      case 'max':
        if (value !== undefined && value !== null && value > rule.value) {
          errors.push({
            path: fieldName,
            message: rule.message,
            severity,
            field: fieldName,
            code: 'MAX_VALUE'
          });
        }
        break;

      case 'email':
        if (value && typeof value === 'string') {
          const emailPattern = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
          if (!emailPattern.test(value)) {
            errors.push({
              path: fieldName,
              message: rule.message || 'Please enter a valid email address',
              severity,
              field: fieldName,
              code: 'INVALID_EMAIL'
            });
          }
        }
        break;

      case 'url':
        if (value && typeof value === 'string') {
          try {
            new URL(value);
          } catch {
            errors.push({
              path: fieldName,
              message: rule.message || 'Please enter a valid URL',
              severity,
              field: fieldName,
              code: 'INVALID_URL'
            });
          }
        }
        break;

      case 'custom':
        if (rule.value && typeof rule.value === 'function') {
          const customError = rule.value(value);
          if (customError) {
            errors.push({
              path: fieldName,
              message: customError,
              severity,
              field: fieldName,
              code: 'CUSTOM_VALIDATION'
            });
          }
        }
        break;
    }
  });

  return errors;
};

// Dependency validation
export const validateDependencies = (
  data: Record<string, any>,
  dependencies: Record<string, string[]>
): ValidationResult => {
  const errors: ValidationError[] = [];
  const warnings: ValidationError[] = [];
  const info: ValidationError[] = [];

  Object.entries(dependencies).forEach(([field, requiredFields]) => {
    if (data[field]) {
      requiredFields.forEach(requiredField => {
        if (!data[requiredField]) {
          warnings.push({
            path: requiredField,
            message: `Field '${requiredField}' is required when '${field}' is set`,
            severity: 'warning',
            field: requiredField,
            code: 'DEPENDENCY_WARNING'
          });
        }
      });
    }
  });

  return {
    isValid: errors.length === 0,
    errors,
    warnings,
    info
  };
};

// Configuration validation
export const validateConfiguration = (
  content: string,
  format: 'json' | 'yaml',
  schema?: any
): ValidationResult => {
  let syntaxResult: ValidationResult;

  // Syntax validation
  if (format === 'json') {
    syntaxResult = validateJSON(content);
  } else {
    syntaxResult = validateYAML(content);
  }

  // If syntax is invalid, return syntax errors only
  if (!syntaxResult.isValid) {
    return syntaxResult;
  }

  // Schema validation (if schema provided)
  if (schema) {
    try {
      const parsed = format === 'json' ? JSON.parse(content) : parseYAML(content);
      const schemaResult = validateAgainstSchema(parsed, schema);
      
      return {
        isValid: syntaxResult.isValid && schemaResult.isValid,
        errors: [...syntaxResult.errors, ...schemaResult.errors],
        warnings: [...syntaxResult.warnings, ...schemaResult.warnings],
        info: [...syntaxResult.info, ...schemaResult.info]
      };
    } catch (error) {
      return {
        isValid: false,
        errors: [{
          path: 'root',
          message: 'Failed to parse content for schema validation',
          severity: 'error',
          code: 'PARSE_ERROR'
        }],
        warnings: syntaxResult.warnings,
        info: syntaxResult.info
      };
    }
  }

  return syntaxResult;
};

// Simple YAML parser (for validation purposes)
const parseYAML = (content: string): any => {
  // This is a simplified YAML parser for validation
  // In production, you might want to use a proper YAML library
  const lines = content.split('\n');
  const result: any = {};
  let currentKey = '';
  let currentValue = '';

  lines.forEach(line => {
    const trimmed = line.trim();
    if (trimmed && !trimmed.startsWith('#')) {
      const colonIndex = trimmed.indexOf(':');
      if (colonIndex > 0) {
        if (currentKey && currentValue) {
          result[currentKey] = currentValue.trim();
        }
        currentKey = trimmed.substring(0, colonIndex).trim();
        currentValue = trimmed.substring(colonIndex + 1).trim();
      } else if (currentKey) {
        currentValue += ' ' + trimmed;
      }
    }
  });

  if (currentKey && currentValue) {
    result[currentKey] = currentValue.trim();
  }

  return result;
};

// Real-time validation debouncer
export const createValidationDebouncer = (delay: number = 300) => {
  let timeoutId: NodeJS.Timeout | null = null;
  
  return (callback: () => void) => {
    if (timeoutId) {
      clearTimeout(timeoutId);
    }
    timeoutId = setTimeout(callback, delay);
  };
};

// Validation error formatter
export const formatValidationErrors = (errors: ValidationError[]): string => {
  return errors
    .map(error => `${error.path}: ${error.message}`)
    .join('\n');
};

// Get error suggestions
export const getErrorSuggestions = (error: ValidationError): string[] => {
  const suggestions: string[] = [];

  switch (error.code) {
    case 'JSON_SYNTAX_ERROR':
      suggestions.push('Check for missing commas, brackets, or quotes');
      suggestions.push('Verify that all strings are properly quoted');
      suggestions.push('Ensure all brackets and braces are properly closed');
      break;
    
    case 'YAML_TAB_ERROR':
      suggestions.push('Replace all tabs with spaces');
      suggestions.push('Use consistent indentation (2 or 4 spaces)');
      break;
    
    case 'REQUIRED_FIELD_MISSING':
      suggestions.push('Add the missing required field');
      suggestions.push('Check if the field name is spelled correctly');
      break;
    
    case 'TYPE_MISMATCH':
      suggestions.push('Ensure the field value matches the expected type');
      suggestions.push('Check if quotes are needed for string values');
      break;
    
    case 'PATTERN_MISMATCH':
      suggestions.push('Check the field format against the required pattern');
      suggestions.push('Verify special characters and formatting');
      break;
    
    default:
      suggestions.push('Review the field value and requirements');
      suggestions.push('Check the documentation for field specifications');
  }

  return suggestions;
}; 