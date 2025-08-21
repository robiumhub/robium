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
  IconButton,
  Tooltip,
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
  Delete as DeleteIcon,
  Settings as SettingsIcon,
  FilterList as FilterListIcon,
  People as PeopleIcon,
} from '@mui/icons-material';
import { useNavigate } from 'react-router-dom';
import { useAuth } from '../contexts/AuthContext';

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
  
  // Add Filter Dialog State
  const [addFilterDialogOpen, setAddFilterDialogOpen] = useState(false);
  const [filterType, setFilterType] = useState<'category' | 'value'>('category');
  const [filterForm, setFilterForm] = useState({
    name: '',
    displayName: '',
    description: '',
    categoryId: '',
    value: '',
    isActive: true,
    sortOrder: 0,
  });
  const [isSubmitting, setIsSubmitting] = useState(false);

  // Manage Categories Dialog State
  const [manageCategoriesDialogOpen, setManageCategoriesDialogOpen] = useState(false);
  const [categories, setCategories] = useState([
    { id: 'use_cases', name: 'Use Cases', displayName: 'Use Cases', description: 'Different use cases for robotics projects', isActive: true, sortOrder: 1 },
    { id: 'capabilities', name: 'Capabilities', displayName: 'Capabilities', description: 'Robot capabilities and features', isActive: true, sortOrder: 2 },
    { id: 'robots', name: 'Robot Targets', displayName: 'Robot Targets', description: 'Target robot platforms', isActive: true, sortOrder: 3 },
    { id: 'tags', name: 'Tags', displayName: 'Tags', description: 'General project tags', isActive: true, sortOrder: 4 },
  ]);
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
    { id: '1', email: 'admin@robium.com', username: 'admin', role: 'admin', createdAt: new Date(), updatedAt: new Date() },
    { id: '2', email: 'user@robium.com', username: 'user', role: 'user', createdAt: new Date(), updatedAt: new Date() },
  ]);
  const [editingUser, setEditingUser] = useState<any>(null);
  const [userForm, setUserForm] = useState({
    email: '',
    username: '',
    password: '',
    role: 'user' as 'admin' | 'user',
  });
  const [manageUsersDialogOpen, setManageUsersDialogOpen] = useState(false);

  // Manage Values Dialog State
  const [manageValuesDialogOpen, setManageValuesDialogOpen] = useState(false);
  const [values, setValues] = useState([
    { id: '1', name: 'navigation', displayName: 'Navigation', categoryId: 'use_cases', description: 'Navigation and pathfinding', isActive: true, sortOrder: 1 },
    { id: '2', name: 'slam', displayName: 'SLAM', categoryId: 'capabilities', description: 'Simultaneous Localization and Mapping', isActive: true, sortOrder: 2 },
    { id: '3', name: 'turtlebot3', displayName: 'TurtleBot3', categoryId: 'robots', description: 'TurtleBot3 robot platform', isActive: true, sortOrder: 3 },
    { id: '4', name: 'ros2', displayName: 'ROS2', categoryId: 'tags', description: 'ROS2 framework', isActive: true, sortOrder: 4 },
    { id: '5', name: 'computer_vision', displayName: 'Computer Vision', categoryId: 'capabilities', description: 'Computer vision and image processing', isActive: true, sortOrder: 5 },
    { id: '6', name: 'autonomous_driving', displayName: 'Autonomous Driving', categoryId: 'use_cases', description: 'Autonomous vehicle navigation', isActive: true, sortOrder: 6 },
  ]);
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

  const handleTabChange = (event: React.SyntheticEvent, newValue: number) => {
    setTabValue(newValue);
  };

  const handleAddFilterClick = () => {
    setAddFilterDialogOpen(true);
    setFilterType('category');
    setFilterForm({
      name: '',
      displayName: '',
      description: '',
      categoryId: '',
      value: '',
      isActive: true,
      sortOrder: 0,
    });
  };

  const handleAddFilterClose = () => {
    setAddFilterDialogOpen(false);
    setFilterForm({
      name: '',
      displayName: '',
      description: '',
      categoryId: '',
      value: '',
      isActive: true,
      sortOrder: 0,
    });
    setIsSubmitting(false);
  };

  const handleFilterFormChange = (field: string, value: any) => {
    setFilterForm(prev => ({
      ...prev,
      [field]: value,
    }));
  };

  const handleAddFilterSubmit = async () => {
    if (!filterForm.name.trim() || !filterForm.displayName.trim()) {
      setError('Name and Display Name are required');
      return;
    }

    setIsSubmitting(true);
    setError(null);

    try {
      if (filterType === 'category') {
        // TODO: Implement API call to add filter category
        console.log('Adding filter category:', filterForm);
        // Simulate API call
        await new Promise(resolve => setTimeout(resolve, 1000));
      } else {
        // TODO: Implement API call to add filter value
        console.log('Adding filter value:', filterForm);
        // Simulate API call
        await new Promise(resolve => setTimeout(resolve, 1000));
      }
      
      handleAddFilterClose();
      // TODO: Refresh filter data
    } catch (err) {
      setError('Failed to add filter. Please try again.');
      console.error('Error adding filter:', err);
    } finally {
      setIsSubmitting(false);
    }
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
    setCategoryForm(prev => ({
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
        const updatedCategories = categories.map(cat => 
          cat.id === editingCategory.id 
            ? { ...cat, ...categoryForm }
            : cat
        );
        setCategories(updatedCategories);
        console.log('Updating category:', { ...editingCategory, ...categoryForm });
      } else {
        // Add new category
        const newCategory = {
          id: categoryForm.name.toLowerCase().replace(/\s+/g, '_'),
          ...categoryForm,
        };
        setCategories(prev => [...prev, newCategory]);
        console.log('Adding new category:', newCategory);
      }
      
      handleManageCategoriesClose();
    } catch (err) {
      setError('Failed to save category. Please try again.');
      console.error('Error saving category:', err);
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleDeleteCategory = async (categoryId: string) => {
    if (window.confirm('Are you sure you want to delete this category? This action cannot be undone.')) {
      try {
        setCategories(prev => prev.filter(cat => cat.id !== categoryId));
        console.log('Deleting category:', categoryId);
      } catch (err) {
        setError('Failed to delete category. Please try again.');
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
    setUserForm(prev => ({
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
        const updatedUsers = users.map(u => 
          u.id === editingUser.id 
            ? { 
                ...u, 
                email: userForm.email,
                username: userForm.username,
                role: userForm.role,
                updatedAt: new Date(),
                ...(userForm.password && { password: userForm.password }) // Only update password if provided
              }
            : u
        );
        setUsers(updatedUsers);
        console.log('Updating user:', { ...editingUser, ...userForm });
      } else {
        // Add new user
        const newUser = {
          id: Date.now().toString(),
          email: userForm.email,
          username: userForm.username,
          password: userForm.password,
          role: userForm.role,
          createdAt: new Date(),
          updatedAt: new Date(),
        };
        setUsers(prev => [...prev, newUser]);
        console.log('Adding new user:', newUser);
      }
      
      handleManageUsersClose();
    } catch (err) {
      setError('Failed to save user. Please try again.');
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

    if (window.confirm('Are you sure you want to delete this user? This action cannot be undone.')) {
      try {
        setUsers(prev => prev.filter(u => u.id !== userId));
        console.log('Deleting user:', userId);
      } catch (err) {
        setError('Failed to delete user. Please try again.');
        console.error('Error deleting user:', err);
      }
    }
  };

  // Manage Values Handlers
  const handleManageValuesClick = () => {
    setManageValuesDialogOpen(true);
  };

  const handleManageValuesClose = () => {
    setManageValuesDialogOpen(false);
    setEditingValue(null);
    setValueForm({
      name: '',
      displayName: '',
      description: '',
      categoryId: '',
      isActive: true,
      sortOrder: 0,
    });
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
    setValueForm(prev => ({
      ...prev,
      [field]: value,
    }));
  };

  const handleSaveValue = async () => {
    if (!valueForm.name.trim() || !valueForm.displayName.trim() || !valueForm.categoryId) {
      setError('Name, Display Name, and Category are required');
      return;
    }

    setIsSubmitting(true);
    setError(null);

    try {
      if (editingValue) {
        // Update existing value
        const updatedValues = values.map(val => 
          val.id === editingValue.id 
            ? { ...val, ...valueForm }
            : val
        );
        setValues(updatedValues);
        console.log('Updating value:', { ...editingValue, ...valueForm });
      } else {
        // Add new value
        const newValue = {
          id: Date.now().toString(),
          ...valueForm,
        };
        setValues(prev => [...prev, newValue]);
        console.log('Adding new value:', newValue);
      }
      
      handleManageValuesClose();
    } catch (err) {
      setError('Failed to save value. Please try again.');
      console.error('Error saving value:', err);
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleDeleteValue = async (valueId: string) => {
    if (window.confirm('Are you sure you want to delete this value? This action cannot be undone.')) {
      try {
        setValues(prev => prev.filter(val => val.id !== valueId));
        console.log('Deleting value:', valueId);
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
              <Button
                variant="contained"
                startIcon={<AddIcon />}
                onClick={handleAddFilterClick}
              >
                Add Filter
              </Button>
            </Box>

            <Grid container spacing={3}>
              <Grid item xs={12} md={6}>
                <Card>
                  <CardContent>
                    <Typography variant="h6" gutterBottom>
                      Filter Categories
                    </Typography>
                    <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
                      Manage filter categories like Use Cases, Capabilities, etc.
                    </Typography>
                    <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                      <Chip label="Use Cases" color="primary" />
                      <Chip label="Capabilities" color="primary" />
                      <Chip label="Robot Targets" color="primary" />
                      <Chip label="Tags" color="primary" />
                    </Box>
                  </CardContent>
                  <CardActions>
                    <Button size="small" startIcon={<EditIcon />} onClick={handleManageCategoriesClick}>
                      Manage Categories
                    </Button>
                  </CardActions>
                </Card>
              </Grid>

              <Grid item xs={12} md={6}>
                <Card>
                  <CardContent>
                    <Typography variant="h6" gutterBottom>
                      Filter Values
                    </Typography>
                    <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
                      Manage individual filter values within each category
                    </Typography>
                    <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                      <Chip label="Navigation" variant="outlined" />
                      <Chip label="SLAM" variant="outlined" />
                      <Chip label="TurtleBot3" variant="outlined" />
                      <Chip label="+12 more" variant="outlined" />
                    </Box>
                  </CardContent>
                  <CardActions>
                    <Button size="small" startIcon={<EditIcon />} onClick={handleManageValuesClick}>
                      Manage Values
                    </Button>
                  </CardActions>
                </Card>
              </Grid>
            </Grid>
          </Box>
        </TabPanel>

        <TabPanel value={tabValue} index={1}>
          <Box>
            <Box
              sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}
            >
              <Typography variant="h6">User Management</Typography>
              <Button
                variant="contained"
                startIcon={<AddIcon />}
                onClick={handleManageUsersClick}
              >
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

      {/* Add Filter Dialog */}
      <Dialog 
        open={addFilterDialogOpen} 
        onClose={handleAddFilterClose}
        maxWidth="sm"
        fullWidth
      >
        <DialogTitle>
          Add New Filter
        </DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 1 }}>
            {error && (
              <Alert severity="error" sx={{ mb: 2 }}>
                {error}
              </Alert>
            )}

            <FormControl fullWidth sx={{ mb: 2 }}>
              <FormLabel>Filter Type *</FormLabel>
              <Select
                value={filterType}
                onChange={(e) => setFilterType(e.target.value as 'category' | 'value')}
                size="small"
              >
                <MenuItem value="category">Filter Category</MenuItem>
                <MenuItem value="value">Filter Value</MenuItem>
              </Select>
              <FormHelperText>
                Choose whether to add a new filter category or a value within an existing category
              </FormHelperText>
            </FormControl>

            <FormControl fullWidth sx={{ mb: 2 }}>
              <FormLabel>Name *</FormLabel>
              <TextField
                value={filterForm.name}
                onChange={(e) => handleFilterFormChange('name', e.target.value)}
                placeholder={filterType === 'category' ? 'e.g., use_cases' : 'e.g., navigation'}
                size="small"
                error={!filterForm.name.trim()}
                helperText={!filterForm.name.trim() ? 'Name is required' : 'Internal name (no spaces, lowercase)'}
              />
            </FormControl>

            <FormControl fullWidth sx={{ mb: 2 }}>
              <FormLabel>Display Name *</FormLabel>
              <TextField
                value={filterForm.displayName}
                onChange={(e) => handleFilterFormChange('displayName', e.target.value)}
                placeholder={filterType === 'category' ? 'e.g., Use Cases' : 'e.g., Navigation'}
                size="small"
                error={!filterForm.displayName.trim()}
                helperText={!filterForm.displayName.trim() ? 'Display name is required' : 'User-friendly name shown in the UI'}
              />
            </FormControl>

            <FormControl fullWidth sx={{ mb: 2 }}>
              <FormLabel>Description</FormLabel>
              <TextField
                value={filterForm.description}
                onChange={(e) => handleFilterFormChange('description', e.target.value)}
                placeholder="Optional description"
                multiline
                rows={2}
                size="small"
              />
            </FormControl>

            {filterType === 'value' && (
              <FormControl fullWidth sx={{ mb: 2 }}>
                <FormLabel>Category *</FormLabel>
                <Select
                  value={filterForm.categoryId}
                  onChange={(e) => handleFilterFormChange('categoryId', e.target.value)}
                  size="small"
                  error={!filterForm.categoryId}
                >
                  <MenuItem value="use_cases">Use Cases</MenuItem>
                  <MenuItem value="capabilities">Capabilities</MenuItem>
                  <MenuItem value="robots">Robot Targets</MenuItem>
                  <MenuItem value="tags">Tags</MenuItem>
                </Select>
                <FormHelperText>
                  Select the category this filter value belongs to
                </FormHelperText>
              </FormControl>
            )}

            <FormControl fullWidth sx={{ mb: 2 }}>
              <FormLabel>Sort Order</FormLabel>
              <TextField
                type="number"
                value={filterForm.sortOrder}
                onChange={(e) => handleFilterFormChange('sortOrder', parseInt(e.target.value) || 0)}
                placeholder="0"
                size="small"
              />
              <FormHelperText>
                Lower numbers appear first in the list
              </FormHelperText>
            </FormControl>

            <FormControlLabel
              control={
                <Switch
                  checked={filterForm.isActive}
                  onChange={(e) => handleFilterFormChange('isActive', e.target.checked)}
                />
              }
              label="Active"
            />
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleAddFilterClose} disabled={isSubmitting}>
            Cancel
          </Button>
          <Button 
            onClick={handleAddFilterSubmit} 
            variant="contained"
            disabled={isSubmitting || !filterForm.name.trim() || !filterForm.displayName.trim() || (filterType === 'value' && !filterForm.categoryId)}
          >
            {isSubmitting ? 'Adding...' : 'Add Filter'}
          </Button>
        </DialogActions>
      </Dialog>

      {/* Manage Categories Dialog */}
      <Dialog 
        open={manageCategoriesDialogOpen} 
        onClose={handleManageCategoriesClose}
        maxWidth="md"
        fullWidth
      >
        <DialogTitle>
          Manage Filter Categories
        </DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 1 }}>
            {error && (
              <Alert severity="error" sx={{ mb: 2 }}>
                {error}
              </Alert>
            )}

            {/* Categories List */}
            <Box sx={{ mb: 3 }}>
              <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
                <Typography variant="h6">Categories</Typography>
                <Button
                  variant="contained"
                  size="small"
                  onClick={() => {
                    setEditingCategory(null);
                    setCategoryForm({
                      name: '',
                      displayName: '',
                      description: '',
                      isActive: true,
                      sortOrder: categories.length + 1,
                    });
                  }}
                >
                  Add New Category
                </Button>
              </Box>
              
              <Box sx={{ display: 'flex', flexDirection: 'column', gap: 1 }}>
                {categories.map((category) => (
                  <Paper key={category.id} sx={{ p: 2, display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                    <Box>
                      <Typography variant="subtitle1">{category.displayName}</Typography>
                      <Typography variant="body2" color="text.secondary">
                        {category.description || 'No description'}
                      </Typography>
                      <Box sx={{ display: 'flex', gap: 1, mt: 1 }}>
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
                      </Box>
                    </Box>
                    <Box sx={{ display: 'flex', gap: 1 }}>
                      <Button
                        size="small"
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
                  </Paper>
                ))}
              </Box>
            </Box>

            {/* Category Form */}
            {(editingCategory || categoryForm.name) && (
              <Box sx={{ borderTop: 1, borderColor: 'divider', pt: 2 }}>
                <Typography variant="h6" gutterBottom>
                  {editingCategory ? 'Edit Category' : 'Add New Category'}
                </Typography>
                
                <FormControl fullWidth sx={{ mb: 2 }}>
                  <FormLabel>Name *</FormLabel>
                  <TextField
                    value={categoryForm.name}
                    onChange={(e) => handleCategoryFormChange('name', e.target.value)}
                    placeholder="e.g., use_cases"
                    size="small"
                    error={!categoryForm.name.trim()}
                    helperText={!categoryForm.name.trim() ? 'Name is required' : 'Internal name (no spaces, lowercase)'}
                    disabled={!!editingCategory}
                  />
                </FormControl>

                <FormControl fullWidth sx={{ mb: 2 }}>
                  <FormLabel>Display Name *</FormLabel>
                  <TextField
                    value={categoryForm.displayName}
                    onChange={(e) => handleCategoryFormChange('displayName', e.target.value)}
                    placeholder="e.g., Use Cases"
                    size="small"
                    error={!categoryForm.displayName.trim()}
                    helperText={!categoryForm.displayName.trim() ? 'Display name is required' : 'User-friendly name shown in the UI'}
                  />
                </FormControl>

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

                <FormControl fullWidth sx={{ mb: 2 }}>
                  <FormLabel>Sort Order</FormLabel>
                  <TextField
                    type="number"
                    value={categoryForm.sortOrder}
                    onChange={(e) => handleCategoryFormChange('sortOrder', parseInt(e.target.value) || 0)}
                    placeholder="0"
                    size="small"
                  />
                  <FormHelperText>
                    Lower numbers appear first in the list
                  </FormHelperText>
                </FormControl>

                <FormControlLabel
                  control={
                    <Switch
                      checked={categoryForm.isActive}
                      onChange={(e) => handleCategoryFormChange('isActive', e.target.checked)}
                    />
                  }
                  label="Active"
                />
              </Box>
            )}
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleManageCategoriesClose} disabled={isSubmitting}>
            Close
          </Button>
          {(editingCategory || categoryForm.name) && (
            <Button 
              onClick={handleSaveCategory} 
              variant="contained"
              disabled={isSubmitting || !categoryForm.name.trim() || !categoryForm.displayName.trim()}
            >
              {isSubmitting ? 'Saving...' : (editingCategory ? 'Update Category' : 'Add Category')}
            </Button>
          )}
        </DialogActions>
      </Dialog>

      {/* Manage Users Dialog */}
      <Dialog 
        open={manageUsersDialogOpen} 
        onClose={handleManageUsersClose}
        maxWidth="md"
        fullWidth
      >
        <DialogTitle>
          Manage Users
        </DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 1 }}>
            {error && (
              <Alert severity="error" sx={{ mb: 2 }}>
                {error}
              </Alert>
            )}

            {/* Users List */}
            <Box sx={{ mb: 3 }}>
              <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
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
                  <Paper key={userData.id} sx={{ p: 2, display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
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
                          <Chip 
                            label="Current User" 
                            color="warning"
                            size="small"
                          />
                        )}
                        <Typography variant="caption" color="text.secondary">
                          Created: {userData.createdAt.toLocaleDateString()}
                        </Typography>
                      </Box>
                    </Box>
                    <Box sx={{ display: 'flex', gap: 1 }}>
                      <Button
                        size="small"
                        onClick={() => handleEditUser(userData)}
                      >
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
                  <FormLabel>{editingUser ? 'Password (leave blank to keep current)' : 'Password *'}</FormLabel>
                  <TextField
                    value={userForm.password}
                    onChange={(e) => handleUserFormChange('password', e.target.value)}
                    placeholder={editingUser ? 'Leave blank to keep current' : 'Enter password'}
                    size="small"
                    type="password"
                    error={!editingUser && !userForm.password.trim()}
                    helperText={!editingUser && !userForm.password.trim() ? 'Password is required for new users' : ''}
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
                  <FormHelperText>
                    Admin users have full system access
                  </FormHelperText>
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
              disabled={isSubmitting || !userForm.email.trim() || !userForm.username.trim() || (!editingUser && !userForm.password.trim())}
            >
              {isSubmitting ? 'Saving...' : (editingUser ? 'Update User' : 'Add User')}
            </Button>
          )}
        </DialogActions>
      </Dialog>

      {/* Manage Values Dialog */}
      <Dialog 
        open={manageValuesDialogOpen} 
        onClose={handleManageValuesClose}
        maxWidth="md"
        fullWidth
      >
        <DialogTitle>
          Manage Filter Values
        </DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 1 }}>
            {error && (
              <Alert severity="error" sx={{ mb: 2 }}>
                {error}
              </Alert>
            )}

            {/* Values List */}
            <Box sx={{ mb: 3 }}>
              <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
                <Typography variant="h6">Filter Values</Typography>
                <Button
                  variant="contained"
                  size="small"
                  onClick={() => {
                    setEditingValue(null);
                    setValueForm({
                      name: '',
                      displayName: '',
                      description: '',
                      categoryId: '',
                      isActive: true,
                      sortOrder: values.length + 1,
                    });
                  }}
                >
                  Add New Value
                </Button>
              </Box>
              
              <Box sx={{ display: 'flex', flexDirection: 'column', gap: 1 }}>
                {values.map((value) => (
                  <Paper key={value.id} sx={{ p: 2, display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                    <Box>
                      <Typography variant="subtitle1">{value.displayName}</Typography>
                      <Typography variant="body2" color="text.secondary">
                        {value.description || 'No description'}
                      </Typography>
                      <Box sx={{ display: 'flex', gap: 1, mt: 1 }}>
                        <Chip 
                          label={categories.find(cat => cat.id === value.categoryId)?.displayName || value.categoryId} 
                          color="primary"
                          size="small"
                        />
                        <Chip 
                          label={value.isActive ? 'Active' : 'Inactive'} 
                          color={value.isActive ? 'success' : 'default'}
                          size="small"
                        />
                        <Chip 
                          label={`Sort: ${value.sortOrder}`} 
                          variant="outlined"
                          size="small"
                        />
                      </Box>
                    </Box>
                    <Box sx={{ display: 'flex', gap: 1 }}>
                      <Button
                        size="small"
                        onClick={() => handleEditValue(value)}
                      >
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
              </Box>
            </Box>

            {/* Value Form */}
            {(editingValue || valueForm.name) && (
              <Box sx={{ borderTop: 1, borderColor: 'divider', pt: 2 }}>
                <Typography variant="h6" gutterBottom>
                  {editingValue ? 'Edit Value' : 'Add New Value'}
                </Typography>
                
                <FormControl fullWidth sx={{ mb: 2 }}>
                  <FormLabel>Name *</FormLabel>
                  <TextField
                    value={valueForm.name}
                    onChange={(e) => handleValueFormChange('name', e.target.value)}
                    placeholder="e.g., navigation"
                    size="small"
                    error={!valueForm.name.trim()}
                    helperText={!valueForm.name.trim() ? 'Name is required' : 'Internal name (no spaces, lowercase)'}
                    disabled={!!editingValue}
                  />
                </FormControl>

                <FormControl fullWidth sx={{ mb: 2 }}>
                  <FormLabel>Display Name *</FormLabel>
                  <TextField
                    value={valueForm.displayName}
                    onChange={(e) => handleValueFormChange('displayName', e.target.value)}
                    placeholder="e.g., Navigation"
                    size="small"
                    error={!valueForm.displayName.trim()}
                    helperText={!valueForm.displayName.trim() ? 'Display name is required' : 'User-friendly name shown in the UI'}
                  />
                </FormControl>

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

                <FormControl fullWidth sx={{ mb: 2 }}>
                  <FormLabel>Category *</FormLabel>
                  <Select
                    value={valueForm.categoryId}
                    onChange={(e) => handleValueFormChange('categoryId', e.target.value)}
                    size="small"
                    error={!valueForm.categoryId}
                  >
                    {categories.map((category) => (
                      <MenuItem key={category.id} value={category.id}>
                        {category.displayName}
                      </MenuItem>
                    ))}
                  </Select>
                  <FormHelperText>
                    Select the category this value belongs to
                  </FormHelperText>
                </FormControl>

                <FormControl fullWidth sx={{ mb: 2 }}>
                  <FormLabel>Sort Order</FormLabel>
                  <TextField
                    type="number"
                    value={valueForm.sortOrder}
                    onChange={(e) => handleValueFormChange('sortOrder', parseInt(e.target.value) || 0)}
                    placeholder="0"
                    size="small"
                  />
                  <FormHelperText>
                    Lower numbers appear first in the list
                  </FormHelperText>
                </FormControl>

                <FormControlLabel
                  control={
                    <Switch
                      checked={valueForm.isActive}
                      onChange={(e) => handleValueFormChange('isActive', e.target.checked)}
                    />
                  }
                  label="Active"
                />
              </Box>
            )}
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleManageValuesClose} disabled={isSubmitting}>
            Close
          </Button>
          {(editingValue || valueForm.name) && (
            <Button 
              onClick={handleSaveValue} 
              variant="contained"
              disabled={isSubmitting || !valueForm.name.trim() || !valueForm.displayName.trim() || !valueForm.categoryId}
            >
              {isSubmitting ? 'Saving...' : (editingValue ? 'Update Value' : 'Add Value')}
            </Button>
          )}
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default AdminPage;
