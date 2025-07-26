import React, { useState } from 'react';
import { Box, Paper, Typography, Link, Alert } from '@mui/material';
import { Link as RouterLink, useNavigate } from 'react-router-dom';
import { useAuth } from '../contexts/AuthContext';
import AccessibleForm from '../components/AccessibleForm';

const Login: React.FC = () => {
  const { login, error } = useAuth();
  const navigate = useNavigate();

  const loginFields = [
    {
      name: 'email',
      label: 'Email Address',
      type: 'email' as const,
      required: true,
      validation: {
        pattern: /^[^\s@]+@[^\s@]+\.[^\s@]+$/,
      },
      placeholder: 'Enter your email address',
    },
    {
      name: 'password',
      label: 'Password',
      type: 'password' as const,
      required: true,
      validation: {
        minLength: 6,
      },
      placeholder: 'Enter your password',
    },
  ];

  const handleSubmit = async (formData: Record<string, any>) => {
    console.log('Login form submitted with data:', formData);
    await login(formData.email, formData.password);
    navigate('/dashboard');
  };

  return (
    <Box
      sx={{
        minHeight: '100vh',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        bgcolor: 'grey.100',
        p: 2,
      }}
    >
      <Paper
        elevation={3}
        sx={{
          p: 4,
          width: '100%',
          maxWidth: 400,
        }}
      >
        <Box sx={{ textAlign: 'center', mb: 3 }}>
          <Typography variant="h4" component="h1" gutterBottom>
            Welcome to Robium
          </Typography>
          <Typography variant="body2" color="text.secondary">
            Sign in to your account
          </Typography>
        </Box>

        {error && (
          <Alert severity="error" sx={{ mb: 2 }} role="alert">
            {error}
          </Alert>
        )}

        <AccessibleForm
          fields={loginFields}
          onSubmit={handleSubmit}
          submitLabel="Sign In"
          title="Welcome to Robium"
          description="Sign in to your account"
        />

        <Box sx={{ textAlign: 'center', mt: 2 }}>
          <Link component={RouterLink} to="/register" variant="body2">
            {"Don't have an account? Sign Up"}
          </Link>
        </Box>
      </Paper>
    </Box>
  );
};

export default Login;
