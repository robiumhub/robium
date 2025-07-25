import React from 'react';
import {
  BrowserRouter as Router,
  Routes,
  Route,
  Navigate,
} from 'react-router-dom';
import { ThemeProvider } from '@mui/material/styles';
import CssBaseline from '@mui/material/CssBaseline';
import { AuthProvider } from './contexts/AuthContext';
import { ErrorProvider } from './contexts/ErrorContext';
import ToastProvider from './components/Toast';
import { ProtectedRoute } from './components/ProtectedRoute';
import { RoleBasedRoute } from './components/RoleBasedRoute';
import Layout from './components/Layout';
import ErrorBoundary from './components/ErrorBoundary';
import Login from './pages/Login';
import Register from './pages/Register';
import Dashboard from './pages/Dashboard';
import DesignSystemShowcase from './pages/DesignSystemShowcase';
import theme from './design/theme';

function App() {
  return (
    <ErrorBoundary>
      <ThemeProvider theme={theme}>
        <CssBaseline />
        <ErrorProvider>
          <ToastProvider>
            <AuthProvider>
              <Router>
                <Routes>
                  {/* Public routes */}
                  <Route path="/login" element={<Login />} />
                  <Route path="/register" element={<Register />} />

                  {/* Protected routes */}
                  <Route
                    path="/dashboard"
                    element={
                      <ProtectedRoute>
                        <Layout>
                          <Dashboard />
                        </Layout>
                      </ProtectedRoute>
                    }
                  />

                  {/* Design System Showcase */}
                  <Route
                    path="/design-system"
                    element={
                      <ProtectedRoute>
                        <Layout>
                          <DesignSystemShowcase />
                        </Layout>
                      </ProtectedRoute>
                    }
                  />

                  {/* Admin-only routes */}
                  <Route
                    path="/admin"
                    element={
                      <RoleBasedRoute allowedRoles={['ADMIN']}>
                        <Layout>
                          <div>
                            Admin Dashboard (Only visible to ADMIN users)
                          </div>
                        </Layout>
                      </RoleBasedRoute>
                    }
                  />

                  {/* Default redirect */}
                  <Route
                    path="/"
                    element={<Navigate to="/dashboard" replace />}
                  />
                  <Route
                    path="*"
                    element={<Navigate to="/dashboard" replace />}
                  />
                </Routes>
              </Router>
            </AuthProvider>
          </ToastProvider>
        </ErrorProvider>
      </ThemeProvider>
    </ErrorBoundary>
  );
}

export default App;
