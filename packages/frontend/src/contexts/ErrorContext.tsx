import React, { createContext, useContext, useState, ReactNode } from 'react';

// Error interface
interface AppError {
  id: string;
  message: string;
  type: 'error' | 'warning' | 'info';
  timestamp: Date;
  details?: any;
  source?: string;
}

// Error context interface
interface ErrorContextType {
  errors: AppError[];
  addError: (
    message: string,
    type?: 'error' | 'warning' | 'info',
    details?: any,
    source?: string
  ) => void;
  removeError: (id: string) => void;
  clearErrors: () => void;
  logError: (error: Error, source?: string) => void;
  hasErrors: boolean;
}

// Create context
const ErrorContext = createContext<ErrorContextType | undefined>(undefined);

// Error provider props
interface ErrorProviderProps {
  children: ReactNode;
}

// Error provider component
export const ErrorProvider: React.FC<ErrorProviderProps> = ({ children }) => {
  const [errors, setErrors] = useState<AppError[]>([]);

  const addError = (
    message: string,
    type: 'error' | 'warning' | 'info' = 'error',
    details?: any,
    source?: string
  ) => {
    const newError: AppError = {
      id: Date.now().toString(),
      message,
      type,
      timestamp: new Date(),
      details,
      source,
    };

    setErrors((prev) => [...prev, newError]);

    // Log error to console in development
    if (process.env.NODE_ENV === 'development') {
      console.error(`[${source || 'App'}] ${message}`, details);
    }

    // Auto-remove info and warning messages after 10 seconds
    if (type !== 'error') {
      setTimeout(() => {
        removeError(newError.id);
      }, 10000);
    }
  };

  const removeError = (id: string) => {
    setErrors((prev) => prev.filter((error) => error.id !== id));
  };

  const clearErrors = () => {
    setErrors([]);
  };

  const logError = (error: Error, source?: string) => {
    addError(
      error.message,
      'error',
      {
        stack: error.stack,
        name: error.name,
      },
      source
    );
  };

  const hasErrors = errors.some((error) => error.type === 'error');

  const value: ErrorContextType = {
    errors,
    addError,
    removeError,
    clearErrors,
    logError,
    hasErrors,
  };

  return (
    <ErrorContext.Provider value={value}>{children}</ErrorContext.Provider>
  );
};

// Custom hook to use error context
export const useError = () => {
  const context = useContext(ErrorContext);
  if (context === undefined) {
    throw new Error('useError must be used within an ErrorProvider');
  }
  return context;
};

export default ErrorProvider;
