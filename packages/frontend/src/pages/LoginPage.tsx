import React, { useState } from 'react';
import { Box, Card, CardContent, TextField, Button, Typography, Link, Alert } from '@mui/material';
import { Link as RouterLink, useNavigate } from 'react-router-dom';
import { useAuth } from '../contexts/AuthContext';

const LoginPage: React.FC = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const { login } = useAuth();
  const navigate = useNavigate();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      await login({ email, password });
      navigate('/projects');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Login failed');
    } finally {
      setLoading(false);
    }
  };

  return (
    <Box
      display="flex"
      justifyContent="center"
      alignItems="center"
      minHeight="100vh"
      bgcolor="background.default"
    >
      <Card sx={{ maxWidth: 400, width: '100%', mx: 2 }}>
        <CardContent sx={{ p: 4 }}>
          <Typography variant="h4" component="h1" gutterBottom align="center">
            Welcome to Robium
          </Typography>
          <Typography variant="body2" color="text.secondary" align="center" sx={{ mb: 3 }}>
            Sign in to your account
          </Typography>

          {error && (
            <Alert severity="error" sx={{ mb: 2 }}>
              {error}
            </Alert>
          )}

          <Box component="form" onSubmit={handleSubmit}>
            <TextField
              fullWidth
              label="Email"
              type="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              margin="normal"
              required
              autoComplete="email"
            />
            <TextField
              fullWidth
              label="Password"
              type="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              margin="normal"
              required
              autoComplete="current-password"
            />
            <Button
              type="submit"
              fullWidth
              variant="contained"
              size="large"
              disabled={loading}
              sx={{ mt: 3, mb: 2 }}
            >
              {loading ? 'Signing in...' : 'Sign In'}
            </Button>
            <Box textAlign="center">
              <Link component={RouterLink} to="/register" variant="body2">
                Don't have an account? Sign up
              </Link>
            </Box>
          </Box>

          {/* Testing Convenience Buttons */}
          <Box sx={{ mt: 4, pt: 3, borderTop: 1, borderColor: 'divider' }}>
            <Typography variant="body2" color="text.secondary" align="center" sx={{ mb: 2 }}>
              🧪 Testing Convenience
            </Typography>
            <Box display="flex" gap={2}>
              <Button
                fullWidth
                variant="outlined"
                color="primary"
                onClick={async () => {
                  setLoading(true);
                  setError('');
                  try {
                    await login({ email: 'admin@robium.com', password: 'password123' });
                    navigate('/projects');
                  } catch (err) {
                    setError(err instanceof Error ? err.message : 'Admin login failed');
                  } finally {
                    setLoading(false);
                  }
                }}
                disabled={loading}
                sx={{ fontSize: '0.875rem' }}
              >
                Login as Admin
              </Button>
              <Button
                fullWidth
                variant="outlined"
                color="secondary"
                onClick={async () => {
                  setLoading(true);
                  setError('');
                  try {
                    await login({ email: 'user@robium.com', password: 'password123' });
                    navigate('/projects');
                  } catch (err) {
                    setError(err instanceof Error ? err.message : 'User login failed');
                  } finally {
                    setLoading(false);
                  }
                }}
                disabled={loading}
                sx={{ fontSize: '0.875rem' }}
              >
                Login as User
              </Button>
            </Box>
            <Typography variant="caption" color="text.secondary" align="center" display="block" sx={{ mt: 1 }}>
              Admin: admin@robium.com | User: user@robium.com | Password: password123
            </Typography>
          </Box>
        </CardContent>
      </Card>
    </Box>
  );
};

export default LoginPage;
