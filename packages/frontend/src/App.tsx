import React from 'react';
import { BrowserRouter as Router } from 'react-router-dom';
import { ThemeProvider } from '@mui/material/styles';
import CssBaseline from '@mui/material/CssBaseline';
import { AuthProvider } from './contexts/AuthContext';
import { ErrorProvider } from './contexts/ErrorContext';
import { NavigationProvider } from './contexts/NavigationContext';
import ToastProvider from './components/Toast';
import ErrorBoundary from './components/ErrorBoundary';
import AccessibilityProvider from './components/AccessibilityProvider';
import AppRoutes from './routes';
import theme from './design/theme';

function App() {
  return (
    <ErrorBoundary>
      <ThemeProvider theme={theme}>
        <CssBaseline />
        <ErrorProvider>
          <ToastProvider>
            <AuthProvider>
              <AccessibilityProvider>
                <Router>
                  <NavigationProvider>
                    <AppRoutes />
                  </NavigationProvider>
                </Router>
              </AccessibilityProvider>
            </AuthProvider>
          </ToastProvider>
        </ErrorProvider>
      </ThemeProvider>
    </ErrorBoundary>
  );
}

export default App;
