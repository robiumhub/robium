import React, { useState, useEffect } from 'react';
import {
  Box,
  Typography,
  Paper,
  Grid,
  Card,
  CardContent,
  CardActions,
  Button,
  Chip,
  CircularProgress,
  Alert,
  Tabs,
  Tab,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  TextField,
  FormControl,
  FormLabel,
  FormHelperText,
  Select,
  MenuItem,
  Switch,
  FormControlLabel,
} from '@mui/material';
import {
  Add as AddIcon,
  Edit as EditIcon,
  Settings as SettingsIcon,
  FilterList as FilterListIcon,
  People as PeopleIcon,
} from '@mui/icons-material';
import { useNavigate } from 'react-router-dom';
import { useAuth } from '../contexts/AuthContext';
import { ApiService } from '../services/api';

interface TabPanelProps {
  children?: React.ReactNode;
  index: number;
  value: number;
}

function TabPanel(props: TabPanelProps) {
  const { children, value, index, ...other } = props;

  return (
    <div
      role="tabpanel"
      hidden={value !== index}
      id={`admin-tabpanel-${index}`}
      aria-labelledby={`admin-tab-${index}`}
      {...other}
    >
      {value === index && <Box sx={{ p: 3 }}>{children}</Box>}
    </div>
  );
}

const AdminPage: React.FC = () => {
  const { user } = useAuth();
  const navigate = useNavigate();
  const [tabValue, setTabValue] = useState(0);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);

  const [isSubmitting, setIsSubmitting] = useState(false);

  // Manage Categories Dialog State
  const [manageCategoriesDialogOpen, setManageCategoriesDialogOpen] = useState(false);
  const [categories, setCategories] = useState<any[]>([]);
  const [editingCategory, setEditingCategory] = useState<any>(null);
  const [categoryForm, setCategoryForm] = useState({
    name: '',
    displayName: '',
    description: '',
    isActive: true,
    sortOrder: 0,
  });

  // User Management State
  const [users, setUsers] = useState([
    {
      id: '1',
      email: 'admin@robium.com',
      username: 'admin',
      role: 'admin',
      createdAt: new Date(),
      updatedAt: new Date(),
    },
    {
      id: '2',
      email: 'user@robium.com',
      username: 'user',
      role: 'user',
      createdAt: new Date(),
      updatedAt: new Date(),
    },
  ]);
  const [editingUser, setEditingUser] = useState<any>(null);
  const [userForm, setUserForm] = useState({
    email: '',
    username: '',
    password: '',
    role: 'user' as 'admin' | 'user',
  });
  const [manageUsersDialogOpen, setManageUsersDialogOpen] = useState(false);

  // Value Management State
  const [editingValue, setEditingValue] = useState<any>(null);
  const [valueForm, setValueForm] = useState({
    name: '',
    displayName: '',
    description: '',
    categoryId: '',
    isActive: true,
    sortOrder: 0,
  });

  // Check if user is admin
  useEffect(() => {
    if (user && user.role !== 'admin') {
      navigate('/');
    }
  }, [user, navigate]);

  // Load categories from API
  const loadCategories = async () => {
    try {
      setLoading(true);
      const response = await ApiService.getFilterCategories();
      if (response.success && response.data) {
        setCategories(response.data.categories);
      }
    } catch (err) {
      console.error('Error loading categories:', err);
      setError('Failed to load categories');
    } finally {
      setLoading(false);
    }
  };

  // Load users from API
  const loadUsers = async () => {
    try {
      const response = await ApiService.getUsers();
      if (response.success && response.data) {
        setUsers(response.data.users);
      }
    } catch (err) {
      console.error('Error loading users:', err);
      setError('Failed to load users');
    }
  };

  // Load data on component mount
  useEffect(() => {
    if (user?.role === 'admin') {
      loadCategories();
      loadUsers();
    }
  }, [user]);

  const handleTabChange = (event: React.SyntheticEvent, newValue: number) => {
    setTabValue(newValue);
  };





  const handleManageCategoriesClick = () => {
    setManageCategoriesDialogOpen(true);
  };

  const handleManageCategoriesClose = () => {
    setManageCategoriesDialogOpen(false);
    setEditingCategory(null);
    setCategoryForm({
      name: '',
      displayName: '',
      description: '',
      isActive: true,
      sortOrder: 0,
    });
  };

  const handleEditCategory = (category: any) => {
    setEditingCategory(category);
    setCategoryForm({
      name: category.name,
      displayName: category.displayName,
      description: category.description || '',
      isActive: category.isActive,
      sortOrder: category.sortOrder,
    });
  };

  const handleCategoryFormChange = (field: string, value: any) => {
    setCategoryForm((prev) => ({
      ...prev,
      [field]: value,
    }));
  };

  const handleSaveCategory = async () => {
    if (!categoryForm.name.trim() || !categoryForm.displayName.trim()) {
      setError('Name and Display Name are required');
      return;
    }

    setIsSubmitting(true);
    setError(null);

    try {
      if (editingCategory) {
        // Update existing category
        const response = await ApiService.updateFilterCategory(editingCategory.id, {
          displayName: categoryForm.displayName,
          description: categoryForm.description,
          isActive: categoryForm.isActive,
          sortOrder: categoryForm.sortOrder,
        });

        if (response.success && response.data) {
          const updatedCategories = categories.map((cat) =>
            cat.id === editingCategory.id ? response.data : cat
          );
          setCategories(updatedCategories);
        } else {
          throw new Error(response.error || 'Failed to update category');
        }
      } else {
        // Add new category
        const response = await ApiService.createFilterCategory({
          name: categoryForm.name,
          displayName: categoryForm.displayName,
          description: categoryForm.description,
          isActive: categoryForm.isActive,
          sortOrder: categoryForm.sortOrder,
        });

        if (response.success && response.data) {
          setCategories((prev) => [...prev, response.data]);
        } else {
          throw new Error(response.error || 'Failed to create category');
        }
      }

      handleManageCategoriesClose();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to save category. Please try again.');
      console.error('Error saving category:', err);
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleDeleteCategory = async (categoryId: string) => {
    if (
      window.confirm('Are you sure you want to delete this category? This action cannot be undone.')
    ) {
      try {
        const response = await ApiService.deleteFilterCategory(categoryId);

        if (response.success) {
          setCategories((prev) => prev.filter((cat) => cat.id !== categoryId));
        } else {
          throw new Error(response.error || 'Failed to delete category');
        }
      } catch (err) {
        setError(
          err instanceof Error ? err.message : 'Failed to delete category. Please try again.'
        );
        console.error('Error deleting category:', err);
      }
    }
  };

  const handleManageUsersClick = () => {
    setManageUsersDialogOpen(true);
  };

  const handleManageUsersClose = () => {
    setManageUsersDialogOpen(false);
    setEditingUser(null);
    setUserForm({
      email: '',
      username: '',
      password: '',
      role: 'user',
    });
  };

  const handleEditUser = (userData: any) => {
    setEditingUser(userData);
    setUserForm({
      email: userData.email,
      username: userData.username,
      password: '', // Don't populate password for security
      role: userData.role,
    });
  };

  const handleUserFormChange = (field: string, value: any) => {
    setUserForm((prev) => ({
      ...prev,
      [field]: value,
    }));
  };

  const handleSaveUser = async () => {
    if (!userForm.email.trim() || !userForm.username.trim()) {
      setError('Email and Username are required');
      return;
    }

    if (!editingUser && !userForm.password.trim()) {
      setError('Password is required for new users');
      return;
    }

    setIsSubmitting(true);
    setError(null);

    try {
      if (editingUser) {
        // Update existing user
        const response = await ApiService.updateUser(editingUser.id, {
          email: userForm.email,
          username: userForm.username,
          role: userForm.role,
          ...(userForm.password && { password: userForm.password }), // Only update password if provided
        });

        if (response.success && response.data) {
          // Update local state
          setUsers((prev) => prev.map((u) => (u.id === editingUser.id ? response.data! : u)));
          setSuccess(`User "${response.data.username}" updated successfully!`);
          setTimeout(() => handleManageUsersClose(), 1500);
        } else {
          throw new Error(response.error || 'Failed to update user');
        }
      } else {
        // Create new user
        const response = await ApiService.createUser({
          email: userForm.email,
          username: userForm.username,
          password: userForm.password,
          role: userForm.role,
        });

        if (response.success && response.data) {
          // Add to local state
          setUsers((prev) => [...prev, response.data!]);
          setSuccess(`User "${response.data.username}" created successfully!`);
          setTimeout(() => handleManageUsersClose(), 1500);
        } else {
          throw new Error(response.error || 'Failed to create user');
        }
      }
    } catch (err) {
      const errorMessage =
        err instanceof Error ? err.message : 'Failed to save user. Please try again.';
      setError(errorMessage);
      console.error('Error saving user:', err);
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleDeleteUser = async (userId: string) => {
    // Prevent deleting the current user
    if (userId === user?.id) {
      setError('You cannot delete your own account');
      return;
    }

    if (
      window.confirm('Are you sure you want to delete this user? This action cannot be undone.')
    ) {
      try {
        const response = await ApiService.deleteUser(userId);

        if (response.success) {
          // Remove from local state
          setUsers((prev) => prev.filter((u) => u.id !== userId));
          setSuccess('User deleted successfully!');
        } else {
          throw new Error(response.error || 'Failed to delete user');
        }
      } catch (err) {
        const errorMessage =
          err instanceof Error ? err.message : 'Failed to delete user. Please try again.';
        setError(errorMessage);
        console.error('Error deleting user:', err);
      }
    }
  };

  const handleEditValue = (value: any) => {
    setEditingValue(value);
    setValueForm({
      name: value.name,
      displayName: value.displayName,
      description: value.description || '',
      categoryId: value.categoryId,
      isActive: value.isActive,
      sortOrder: value.sortOrder,
    });
  };

  const handleValueFormChange = (field: string, value: any) => {
    setValueForm((prev) => ({
      ...prev,
      [field]: value,
    }));
  };

  const handleSaveValue = async () => {
    if (!valueForm.name.trim() || !valueForm.displayName.trim()) {
      setError('Name and Display Name are required');
      return;
    }

    setIsSubmitting(true);
    setError(null);

    try {
      // For now, just log the action - API integration will be added later
      if (editingValue) {
        console.log('Updating value:', { ...editingValue, ...valueForm });
      } else {
        console.log('Adding new value:', valueForm);
      }

      // Clear the form
      setEditingValue(null);
      setValueForm({
        name: '',
        displayName: '',
        description: '',
        categoryId: '',
        isActive: true,
        sortOrder: 0,
      });
    } catch (err) {
      setError('Failed to save value. Please try again.');
      console.error('Error saving value:', err);
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleDeleteValue = async (valueId: string) => {
    if (
      window.confirm('Are you sure you want to delete this value? This action cannot be undone.')
    ) {
      try {
        console.log('Deleting value:', valueId);
        // API integration will be added later
      } catch (err) {
        setError('Failed to delete value. Please try again.');
        console.error('Error deleting value:', err);
      }
    }
  };

  if (!user || user.role !== 'admin') {
    return (
      <Box display="flex" justifyContent="center" alignItems="center" minHeight="400px">
        <Alert severity="error">Access denied. Admin privileges required.</Alert>
      </Box>
    );
  }

  if (loading) {
    return (
      <Box display="flex" justifyContent="center" alignItems="center" minHeight="400px">
        <CircularProgress />
      </Box>
    );
  }

  return (
    <Box sx={{ p: 3 }}>
      <Typography variant="h4" gutterBottom>
        Admin Panel
      </Typography>
      <Typography variant="body1" color="text.secondary" sx={{ mb: 3 }}>
        Manage system settings, filters, and user data
      </Typography>

      <Paper sx={{ width: '100%' }}>
        <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
          <Tabs value={tabValue} onChange={handleTabChange} aria-label="admin tabs">
            <Tab icon={<FilterListIcon />} label="Filter Management" iconPosition="start" />
            <Tab icon={<PeopleIcon />} label="User Management" iconPosition="start" />
            <Tab icon={<SettingsIcon />} label="System Settings" iconPosition="start" />
          </Tabs>
        </Box>

        <TabPanel value={tabValue} index={0}>
          <Box>
            <Box
              sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}
            >
              <Typography variant="h6">Filter Management</Typography>
              <Button variant="contained" startIcon={<AddIcon />} onClick={handleManageCategoriesClick}>
                Add New Category
              </Button>
            </Box>

            {/* Categories List */}
            <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
              {categories.map((category) => (
                <Card key={category.id}>
                  <CardContent>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start' }}>
                      <Box sx={{ flex: 1 }}>
                        <Typography variant="h6" gutterBottom>
                          {category.displayName}
                        </Typography>
                        <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
                          {category.description || 'No description'}
                        </Typography>
                        <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                          <Chip
                            label={category.isActive ? 'Active' : 'Inactive'}
                            color={category.isActive ? 'success' : 'default'}
                            size="small"
                          />
                          <Chip
                            label={`Sort: ${category.sortOrder}`}
                            variant="outlined"
                            size="small"
                          />
                          <Chip
                            label={`${category.values?.length || 0} values`}
                            variant="outlined"
                            size="small"
                          />
                        </Box>
                      </Box>
                      <Box sx={{ display: 'flex', gap: 1 }}>
                        <Button
                          size="small"
                          startIcon={<EditIcon />}
                          onClick={() => handleEditCategory(category)}
                        >
                          Edit
                        </Button>
                        <Button
                          size="small"
                          color="error"
                          onClick={() => handleDeleteCategory(category.id)}
                        >
                          Delete
                        </Button>
                      </Box>
                    </Box>
                  </CardContent>
                </Card>
              ))}
              
              {categories.length === 0 && (
                <Card>
                  <CardContent>
                    <Typography variant="body1" color="text.secondary" align="center" sx={{ py: 4 }}>
                      No filter categories found. Click "Add New Category" to create your first category.
                    </Typography>
                  </CardContent>
                </Card>
              )}
            </Box>
          </Box>
        </TabPanel>

        <TabPanel value={tabValue} index={1}>
          <Box>
            <Box
              sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}
            >
              <Typography variant="h6">User Management</Typography>
              <Button variant="contained" startIcon={<AddIcon />} onClick={handleManageUsersClick}>
                Manage Users
              </Button>
            </Box>

            <Grid container spacing={3}>
              <Grid item xs={12}>
                <Card>
                  <CardContent>
                    <Typography variant="h6" gutterBottom>
                      System Users
                    </Typography>
                    <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
                      Manage user accounts, roles, and permissions
                    </Typography>
                    <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                      {users.map((userData) => (
                        <Chip
                          key={userData.id}
                          label={`${userData.username} (${userData.role})`}
                          color={userData.role === 'admin' ? 'primary' : 'default'}
                          variant={userData.id === user?.id ? 'filled' : 'outlined'}
                        />
                      ))}
                    </Box>
                  </CardContent>
                  <CardActions>
                    <Button size="small" startIcon={<EditIcon />} onClick={handleManageUsersClick}>
                      Manage Users
                    </Button>
                  </CardActions>
                </Card>
              </Grid>
            </Grid>
          </Box>
        </TabPanel>

        <TabPanel value={tabValue} index={2}>
          <Typography variant="h6" gutterBottom>
            System Settings
          </Typography>
          <Typography variant="body2" color="text.secondary">
            System settings functionality will be implemented here.
          </Typography>
        </TabPanel>
      </Paper>



      {/* Manage Categories Dialog */}
      <Dialog
        open={manageCategoriesDialogOpen}
        onClose={handleManageCategoriesClose}
        maxWidth="lg"
        fullWidth
      >
        <DialogTitle>
          {editingCategory ? `Edit Category: ${editingCategory.displayName}` : 'Add New Category'}
        </DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 1 }}>
            {error && (
              <Alert severity="error" sx={{ mb: 2 }}>
                {error}
              </Alert>
            )}
            {success && (
              <Alert severity="success" sx={{ mb: 2 }}>
                {success}
              </Alert>
            )}

            {/* Category Form */}
            <Box sx={{ mb: 3 }}>
              <Typography variant="h6" gutterBottom>
                Category Details
              </Typography>

              <Grid container spacing={2}>
                <Grid item xs={12} md={6}>
                  <FormControl fullWidth sx={{ mb: 2 }}>
                    <FormLabel>Name *</FormLabel>
                    <TextField
                      value={categoryForm.name}
                      onChange={(e) => handleCategoryFormChange('name', e.target.value)}
                      placeholder="e.g., use_cases"
                      size="small"
                      error={!categoryForm.name.trim()}
                      helperText={
                        !categoryForm.name.trim()
                          ? 'Name is required'
                          : 'Internal name (no spaces, lowercase)'
                      }
                      disabled={!!editingCategory}
                    />
                  </FormControl>
                </Grid>

                <Grid item xs={12} md={6}>
                  <FormControl fullWidth sx={{ mb: 2 }}>
                    <FormLabel>Display Name *</FormLabel>
                    <TextField
                      value={categoryForm.displayName}
                      onChange={(e) => handleCategoryFormChange('displayName', e.target.value)}
                      placeholder="e.g., Use Cases"
                      size="small"
                      error={!categoryForm.displayName.trim()}
                      helperText={
                        !categoryForm.displayName.trim()
                          ? 'Display name is required'
                          : 'User-friendly name shown in the UI'
                      }
                    />
                  </FormControl>
                </Grid>

                <Grid item xs={12}>
                  <FormControl fullWidth sx={{ mb: 2 }}>
                    <FormLabel>Description</FormLabel>
                    <TextField
                      value={categoryForm.description}
                      onChange={(e) => handleCategoryFormChange('description', e.target.value)}
                      placeholder="Optional description"
                      multiline
                      rows={2}
                      size="small"
                    />
                  </FormControl>
                </Grid>

                <Grid item xs={12} md={6}>
                  <FormControl fullWidth sx={{ mb: 2 }}>
                    <FormLabel>Sort Order</FormLabel>
                    <TextField
                      type="number"
                      value={categoryForm.sortOrder}
                      onChange={(e) =>
                        handleCategoryFormChange('sortOrder', parseInt(e.target.value) || 0)
                      }
                      placeholder="0"
                      size="small"
                    />
                    <FormHelperText>Lower numbers appear first in the list</FormHelperText>
                  </FormControl>
                </Grid>

                <Grid item xs={12} md={6}>
                  <FormControlLabel
                    control={
                      <Switch
                        checked={categoryForm.isActive}
                        onChange={(e) => handleCategoryFormChange('isActive', e.target.checked)}
                      />
                    }
                    label="Active"
                  />
                </Grid>
              </Grid>
            </Box>

            {/* Values Section - Only show when editing a category */}
            {editingCategory && (
              <Box sx={{ borderTop: 1, borderColor: 'divider', pt: 2 }}>
                <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
                  <Typography variant="h6">Category Values</Typography>
                  <Button
                    variant="contained"
                    size="small"
                    startIcon={<AddIcon />}
                    onClick={() => {
                      setEditingValue(null);
                      setValueForm({
                        name: '',
                        displayName: '',
                        description: '',
                        categoryId: editingCategory.id,
                        isActive: true,
                        sortOrder: (editingCategory.values?.length || 0) + 1,
                      });
                    }}
                  >
                    Add New Value
                  </Button>
                </Box>

                {/* Values List */}
                <Box sx={{ display: 'flex', flexDirection: 'column', gap: 1, mb: 2 }}>
                  {editingCategory.values?.map((value: any) => (
                    <Paper
                      key={value.id}
                      sx={{
                        p: 2,
                        display: 'flex',
                        justifyContent: 'space-between',
                        alignItems: 'center',
                      }}
                    >
                      <Box>
                        <Typography variant="subtitle1">{value.displayName}</Typography>
                        <Typography variant="body2" color="text.secondary">
                          {value.description || 'No description'}
                        </Typography>
                        <Box sx={{ display: 'flex', gap: 1, mt: 1 }}>
                          <Chip
                            label={value.isActive ? 'Active' : 'Inactive'}
                            color={value.isActive ? 'success' : 'default'}
                            size="small"
                          />
                          <Chip label={`Sort: ${value.sortOrder}`} variant="outlined" size="small" />
                        </Box>
                      </Box>
                      <Box sx={{ display: 'flex', gap: 1 }}>
                        <Button size="small" onClick={() => handleEditValue(value)}>
                          Edit
                        </Button>
                        <Button
                          size="small"
                          color="error"
                          onClick={() => handleDeleteValue(value.id)}
                        >
                          Delete
                        </Button>
                      </Box>
                    </Paper>
                  ))}
                  
                  {(!editingCategory.values || editingCategory.values.length === 0) && (
                    <Paper sx={{ p: 3, textAlign: 'center' }}>
                      <Typography variant="body2" color="text.secondary">
                        No values in this category yet. Click "Add New Value" to create the first one.
                      </Typography>
                    </Paper>
                  )}
                </Box>

                {/* Value Form */}
                {(editingValue || valueForm.name) && (
                  <Box sx={{ borderTop: 1, borderColor: 'divider', pt: 2 }}>
                    <Typography variant="h6" gutterBottom>
                      {editingValue ? 'Edit Value' : 'Add New Value'}
                    </Typography>

                    <Grid container spacing={2}>
                      <Grid item xs={12} md={6}>
                        <FormControl fullWidth sx={{ mb: 2 }}>
                          <FormLabel>Name *</FormLabel>
                          <TextField
                            value={valueForm.name}
                            onChange={(e) => handleValueFormChange('name', e.target.value)}
                            placeholder="e.g., navigation"
                            size="small"
                            error={!valueForm.name.trim()}
                            helperText={
                              !valueForm.name.trim()
                                ? 'Name is required'
                                : 'Internal name (no spaces, lowercase)'
                            }
                            disabled={!!editingValue}
                          />
                        </FormControl>
                      </Grid>

                      <Grid item xs={12} md={6}>
                        <FormControl fullWidth sx={{ mb: 2 }}>
                          <FormLabel>Display Name *</FormLabel>
                          <TextField
                            value={valueForm.displayName}
                            onChange={(e) => handleValueFormChange('displayName', e.target.value)}
                            placeholder="e.g., Navigation"
                            size="small"
                            error={!valueForm.displayName.trim()}
                            helperText={
                              !valueForm.displayName.trim()
                                ? 'Display name is required'
                                : 'User-friendly name shown in the UI'
                              }
                          />
                        </FormControl>
                      </Grid>

                      <Grid item xs={12}>
                        <FormControl fullWidth sx={{ mb: 2 }}>
                          <FormLabel>Description</FormLabel>
                          <TextField
                            value={valueForm.description}
                            onChange={(e) => handleValueFormChange('description', e.target.value)}
                            placeholder="Optional description"
                            multiline
                            rows={2}
                            size="small"
                          />
                        </FormControl>
                      </Grid>

                      <Grid item xs={12} md={6}>
                        <FormControl fullWidth sx={{ mb: 2 }}>
                          <FormLabel>Sort Order</FormLabel>
                          <TextField
                            type="number"
                            value={valueForm.sortOrder}
                            onChange={(e) =>
                              handleValueFormChange('sortOrder', parseInt(e.target.value) || 0)
                            }
                            placeholder="0"
                            size="small"
                          />
                          <FormHelperText>Lower numbers appear first in the list</FormHelperText>
                        </FormControl>
                      </Grid>

                      <Grid item xs={12} md={6}>
                        <FormControlLabel
                          control={
                            <Switch
                              checked={valueForm.isActive}
                              onChange={(e) => handleValueFormChange('isActive', e.target.checked)}
                            />
                          }
                          label="Active"
                        />
                      </Grid>
                    </Grid>
                  </Box>
                )}
              </Box>
            )}
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleManageCategoriesClose} disabled={isSubmitting}>
            Close
          </Button>
          {(editingValue || valueForm.name) && (
            <Button
              onClick={handleSaveValue}
              variant="contained"
              disabled={
                isSubmitting ||
                !valueForm.name.trim() ||
                !valueForm.displayName.trim()
              }
            >
              {isSubmitting ? 'Saving...' : editingValue ? 'Update Value' : 'Add Value'}
            </Button>
          )}
          {(editingCategory || categoryForm.name) && (
            <Button
              onClick={handleSaveCategory}
              variant="contained"
              disabled={
                isSubmitting || !categoryForm.name.trim() || !categoryForm.displayName.trim()
              }
            >
              {isSubmitting ? 'Saving...' : editingCategory ? 'Update Category' : 'Add Category'}
            </Button>
          )}
        </DialogActions>
      </Dialog>

      {/* Manage Users Dialog */}
      <Dialog open={manageUsersDialogOpen} onClose={handleManageUsersClose} maxWidth="md" fullWidth>
        <DialogTitle>Manage Users</DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 1 }}>
            {error && (
              <Alert severity="error" sx={{ mb: 2 }}>
                {error}
              </Alert>
            )}

            {/* Users List */}
            <Box sx={{ mb: 3 }}>
              <Box
                sx={{
                  display: 'flex',
                  justifyContent: 'space-between',
                  alignItems: 'center',
                  mb: 2,
                }}
              >
                <Typography variant="h6">Users</Typography>
                <Button
                  variant="contained"
                  size="small"
                  onClick={() => {
                    setEditingUser(null);
                    setUserForm({
                      email: '',
                      username: '',
                      password: '',
                      role: 'user',
                    });
                  }}
                >
                  Add New User
                </Button>
              </Box>

              <Box sx={{ display: 'flex', flexDirection: 'column', gap: 1 }}>
                {users.map((userData) => (
                  <Paper
                    key={userData.id}
                    sx={{
                      p: 2,
                      display: 'flex',
                      justifyContent: 'space-between',
                      alignItems: 'center',
                    }}
                  >
                    <Box>
                      <Typography variant="subtitle1">{userData.username}</Typography>
                      <Typography variant="body2" color="text.secondary">
                        {userData.email}
                      </Typography>
                      <Box sx={{ display: 'flex', gap: 1, mt: 1 }}>
                        <Chip
                          label={userData.role}
                          color={userData.role === 'admin' ? 'primary' : 'default'}
                          size="small"
                        />
                        {userData.id === user?.id && (
                          <Chip label="Current User" color="warning" size="small" />
                        )}
                        <Typography variant="caption" color="text.secondary">
                          Created: {new Date(userData.createdAt).toLocaleDateString()}
                        </Typography>
                      </Box>
                    </Box>
                    <Box sx={{ display: 'flex', gap: 1 }}>
                      <Button size="small" onClick={() => handleEditUser(userData)}>
                        Edit
                      </Button>
                      {userData.id !== user?.id && (
                        <Button
                          size="small"
                          color="error"
                          onClick={() => handleDeleteUser(userData.id)}
                        >
                          Delete
                        </Button>
                      )}
                    </Box>
                  </Paper>
                ))}
              </Box>
            </Box>

            {/* User Form */}
            {(editingUser || userForm.email || userForm.username) && (
              <Box sx={{ borderTop: 1, borderColor: 'divider', pt: 2 }}>
                <Typography variant="h6" gutterBottom>
                  {editingUser ? 'Edit User' : 'Add New User'}
                </Typography>

                <FormControl fullWidth sx={{ mb: 2 }}>
                  <FormLabel>Email *</FormLabel>
                  <TextField
                    value={userForm.email}
                    onChange={(e) => handleUserFormChange('email', e.target.value)}
                    placeholder="user@example.com"
                    size="small"
                    error={!userForm.email.trim()}
                    helperText={!userForm.email.trim() ? 'Email is required' : ''}
                    type="email"
                  />
                </FormControl>

                <FormControl fullWidth sx={{ mb: 2 }}>
                  <FormLabel>Username *</FormLabel>
                  <TextField
                    value={userForm.username}
                    onChange={(e) => handleUserFormChange('username', e.target.value)}
                    placeholder="username"
                    size="small"
                    error={!userForm.username.trim()}
                    helperText={!userForm.username.trim() ? 'Username is required' : ''}
                  />
                </FormControl>

                <FormControl fullWidth sx={{ mb: 2 }}>
                  <FormLabel>
                    {editingUser ? 'Password (leave blank to keep current)' : 'Password *'}
                  </FormLabel>
                  <TextField
                    value={userForm.password}
                    onChange={(e) => handleUserFormChange('password', e.target.value)}
                    placeholder={editingUser ? 'Leave blank to keep current' : 'Enter password'}
                    size="small"
                    type="password"
                    error={!editingUser && !userForm.password.trim()}
                    helperText={
                      !editingUser && !userForm.password.trim()
                        ? 'Password is required for new users'
                        : ''
                    }
                  />
                </FormControl>

                <FormControl fullWidth sx={{ mb: 2 }}>
                  <FormLabel>Role *</FormLabel>
                  <Select
                    value={userForm.role}
                    onChange={(e) => handleUserFormChange('role', e.target.value)}
                    size="small"
                  >
                    <MenuItem value="user">User</MenuItem>
                    <MenuItem value="admin">Admin</MenuItem>
                  </Select>
                  <FormHelperText>Admin users have full system access</FormHelperText>
                </FormControl>
              </Box>
            )}
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleManageUsersClose} disabled={isSubmitting}>
            Close
          </Button>
          {(editingUser || userForm.email || userForm.username) && (
            <Button
              onClick={handleSaveUser}
              variant="contained"
              disabled={
                isSubmitting ||
                !userForm.email.trim() ||
                !userForm.username.trim() ||
                (!editingUser && !userForm.password.trim())
              }
            >
              {isSubmitting ? 'Saving...' : editingUser ? 'Update User' : 'Add User'}
            </Button>
          )}
        </DialogActions>
      </Dialog>


    </Box>
  );
};

export default AdminPage;
