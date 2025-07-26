import React from 'react';
import { BrowserRouter as Router } from 'react-router-dom';
import { ThemeProvider } from '@mui/material/styles';
import { CssBaseline } from '@mui/material';
import theme from './design/theme';
import { AccessibilityProvider } from './components/AccessibilityProvider';
import { ErrorProvider } from './contexts/ErrorContext';
import { AuthProvider } from './contexts/AuthContext';
import { NavigationProvider } from './contexts/NavigationContext';
import { ToastProvider } from './components/Toast';
import AppRoutes from './routes';

function App() {
  return (
    <Router>
      <ThemeProvider theme={theme}>
        <CssBaseline />
        <ErrorProvider>
          <AuthProvider>
            <NavigationProvider>
              <AccessibilityProvider>
                <ToastProvider>
                  <AppRoutes />
                </ToastProvider>
              </AccessibilityProvider>
            </NavigationProvider>
          </AuthProvider>
        </ErrorProvider>
      </ThemeProvider>
    </Router>
  );
}

export default App;
