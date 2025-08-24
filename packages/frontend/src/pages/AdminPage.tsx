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
} from '@mui/material';
import {
  Add as AddIcon,
  Edit as EditIcon,
  Delete as DeleteIcon,
  FilterList as FilterListIcon,
  People as PeopleIcon,
} from '@mui/icons-material';
import { useNavigate } from 'react-router-dom';
import { useAuth } from '../contexts/AuthContext';
import { ApiService } from '../services/api';
import { FilterValue } from '@robium/shared';
import { filterEventManager } from '../utils/filterEvents';
import EditableChip from '../components/EditableChip';

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

  // Add Filter Dialog State
  const [addFilterDialogOpen, setAddFilterDialogOpen] = useState(false);
  const [filterForm, setFilterForm] = useState({
    name: '',
    displayName: '',
    description: '',
    categoryId: '',
    value: '',
  });
  const [isSubmitting, setIsSubmitting] = useState(false);

  // Filter Management State
  const [addValueDialogOpen, setAddValueDialogOpen] = useState(false);
  const [categories, setCategories] = useState<any[]>([]);
  const [editingCategory, setEditingCategory] = useState<any>(null);
  const [categoryForm, setCategoryForm] = useState({
    name: '',
    displayName: '',
    description: '',
  });
  const [categoryValues, setCategoryValues] = useState<FilterValue[]>([]);
  const [editingValue, setEditingValue] = useState<any>(null);
  const [valueForm, setValueForm] = useState({
    value: '',
    displayName: '',
    description: '',
    categoryId: '',
  });

  // Edit Category Dialog State (separate from Manage Categories)
  const [editCategoryDialogOpen, setEditCategoryDialogOpen] = useState(false);

  // Edit Value Dialog State
  const [editValueDialogOpen, setEditValueDialogOpen] = useState(false);
  const [editingValueData, setEditingValueData] = useState<any>(null);
  const [editValueForm, setEditValueForm] = useState({
    displayName: '',
    description: '',
  });

  // All values state for one-page view
  const [allValues, setAllValues] = useState<{ [categoryId: string]: FilterValue[] }>({});

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

  // Note: values state removed as we now use categoryValues for managing filter values

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
        console.log('Loaded categories:', response.data.categories);
        // Check for categories with missing IDs
        const categoriesWithMissingIds = response.data.categories.filter((cat: any) => !cat.id);
        if (categoriesWithMissingIds.length > 0) {
          console.error('Categories with missing IDs:', categoriesWithMissingIds);
          console.warn('Filtering out categories with missing IDs to prevent UI issues');
        }
        // Filter out categories with null or missing IDs to prevent React key conflicts
        const validCategories = response.data.categories.filter(
          (cat: any) => cat.id && cat.id !== null
        );
        setCategories(validCategories);

        // Load values for all categories
        await loadAllCategoryValues(validCategories);
      }
    } catch (err) {
      console.error('Error loading categories:', err);
      setError('Failed to load categories');
    } finally {
      setLoading(false);
    }
  };

  // Load values for all categories
  const loadAllCategoryValues = async (categoriesToLoad: any[]) => {
    try {
      const valuesMap: { [categoryId: string]: FilterValue[] } = {};

      for (const category of categoriesToLoad) {
        try {
          const response = await ApiService.getCategoryValues(category.id);
          if (response.success && response.data) {
            valuesMap[category.id] = response.data.values;
          } else {
            valuesMap[category.id] = [];
          }
        } catch (err) {
          console.error(`Error loading values for category ${category.id}:`, err);
          valuesMap[category.id] = [];
        }
      }

      setAllValues(valuesMap);
    } catch (err) {
      console.error('Error loading category values:', err);
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

  const handleAddFilterClick = () => {
    // Reset the form for adding a new category
    setEditingCategory(null);
    setCategoryForm({
      name: '',
      displayName: '',
      description: '',
    });
    setEditCategoryDialogOpen(true);
  };

  // Validation helper functions
  const validateName = (name: string): string | null => {
    if (!name.trim()) return 'Name is required';
    if (name.length < 2) return 'Name must be at least 2 characters';
    if (name.length > 50) return 'Name must be less than 50 characters';
    if (!/^[a-z0-9_]+$/.test(name))
      return 'Name must contain only lowercase letters, numbers, and underscores';
    if (name.includes(' ')) return 'Name cannot contain spaces';
    return null;
  };

  const validateDisplayName = (displayName: string): string | null => {
    if (!displayName.trim()) return 'Display name is required';
    if (displayName.length < 2) return 'Display name must be at least 2 characters';
    if (displayName.length > 100) return 'Display name must be less than 100 characters';
    return null;
  };

  const validateDescription = (description: string): string | null => {
    if (description.length > 500) return 'Description must be less than 500 characters';
    return null;
  };

  const handleAddFilterClose = () => {
    setAddFilterDialogOpen(false);
    setFilterForm({
      name: '',
      displayName: '',
      description: '',
      categoryId: '',
      value: '',
    });
    setIsSubmitting(false);
    setError(null);
    setSuccess(null);
  };

  const handleFilterFormChange = (field: string, value: any) => {
    setFilterForm((prev) => ({
      ...prev,
      [field]: value,
    }));
  };

  const handleAddFilterSubmit = async () => {
    // Enhanced validation
    const nameError = validateName(filterForm.name);
    const displayNameError = validateDisplayName(filterForm.displayName);
    const descriptionError = validateDescription(filterForm.description);

    if (nameError) {
      setError(nameError);
      return;
    }

    if (displayNameError) {
      setError(displayNameError);
      return;
    }

    if (descriptionError) {
      setError(descriptionError);
      return;
    }

    setIsSubmitting(true);
    setError(null);
    setSuccess(null);

    try {
      const response = await ApiService.createFilterCategory({
        name: filterForm.name.trim(),
        displayName: filterForm.displayName.trim(),
        description: filterForm.description.trim() || undefined,
      });

      if (response.success && response.data) {
        // Add the new category to the local state
        const newCategory = response.data;
        setCategories((prev) => [...prev, newCategory]);
        // Initialize empty values array for the new category
        setAllValues((prev) => ({
          ...prev,
          [newCategory.id]: [],
        }));
        // Notify other components that categories have changed
        filterEventManager.notifyCategoriesChanged();
        setSuccess(`Filter category "${newCategory.displayName}" created successfully!`);
        setTimeout(() => handleAddFilterClose(), 1500); // Close after showing success message
      } else {
        throw new Error(response.error || 'Failed to create filter category');
      }
    } catch (err) {
      const errorMessage =
        err instanceof Error ? err.message : 'Failed to add category. Please try again.';
      setError(errorMessage);
      console.error('Error adding category:', err);
    } finally {
      setIsSubmitting(false);
    }
  };

  const loadCategoryValues = async (categoryId: string) => {
    try {
      console.log('Loading category values for:', categoryId);
      const response = await ApiService.getCategoryValues(categoryId);
      if (response.success && response.data?.values) {
        console.log('Loaded category values:', response.data.values);
        setCategoryValues(response.data.values);
        // Also update allValues state
        setAllValues((prev) => ({
          ...prev,
          [categoryId]: response.data!.values,
        }));
      } else {
        console.log('No values found for category:', categoryId);
        setCategoryValues([]);
        setAllValues((prev) => ({
          ...prev,
          [categoryId]: [],
        }));
      }
    } catch (err) {
      console.error('Error loading category values:', err);
      setCategoryValues([]);
      setAllValues((prev) => ({
        ...prev,
        [categoryId]: [],
      }));
    }
  };

  const handleEditCategory = async (category: any) => {
    setEditingCategory(category);
    setCategoryForm({
      name: category.name,
      displayName: category.displayName,
      description: category.description || '',
    });
    setEditCategoryDialogOpen(true); // Open the separate Edit Category Dialog
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
    setSuccess(null);

    try {
      console.log('Saving category:', { categoryForm, editingCategory });

      if (editingCategory) {
        // Update existing category
        const response = await ApiService.updateFilterCategory(editingCategory.id, {
          displayName: categoryForm.displayName,
          description: categoryForm.description || '',
        });

        if (response.success && response.data) {
          const updatedCategories = categories.map((cat) =>
            cat.id === editingCategory.id ? response.data : cat
          );
          setCategories(updatedCategories);
          setSuccess(`Category "${response.data.displayName}" updated successfully!`);

          // Close the Edit Category Dialog
          setEditCategoryDialogOpen(false);
          setEditingCategory(null);
          setCategoryForm({
            name: '',
            displayName: '',
            description: '',
          });
        } else {
          throw new Error(response.error || 'Failed to update category');
        }
      } else {
        // Create new category
        const response = await ApiService.createFilterCategory({
          name: categoryForm.name.trim(),
          displayName: categoryForm.displayName.trim(),
          description: categoryForm.description.trim() || undefined,
        });

        if (response.success && response.data) {
          const newCategory = response.data;
          setCategories((prev) => [...prev, newCategory]);
          setAllValues((prev) => ({
            ...prev,
            [newCategory.id]: [],
          }));
          setSuccess(`Category "${newCategory.displayName}" created successfully!`);

          // Close the dialog and reset form
          setEditCategoryDialogOpen(false);
          setEditingCategory(null);
          setCategoryForm({
            name: '',
            displayName: '',
            description: '',
          });
        } else {
          throw new Error(response.error || 'Failed to create category');
        }
      }
    } catch (err) {
      const errorMessage =
        err instanceof Error ? err.message : 'Failed to save category. Please try again.';
      setError(errorMessage);
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
        setIsSubmitting(true);
        setError(null);
        setSuccess(null);

        console.log('Deleting category:', categoryId);
        const response = await ApiService.deleteFilterCategory(categoryId);

        if (response.success) {
          setCategories((prev) => prev.filter((cat) => cat.id !== categoryId));
          // Remove from allValues state as well
          setAllValues((prev) => {
            const newAllValues = { ...prev };
            delete newAllValues[categoryId];
            return newAllValues;
          });

          setSuccess('Category deleted successfully!');
          console.log('Category deleted successfully:', categoryId);
        } else {
          throw new Error(response.error || 'Failed to delete category');
        }
      } catch (err) {
        const errorMessage =
          err instanceof Error ? err.message : 'Failed to delete category. Please try again.';

        // Check if it's the "associated values" error
        if (
          errorMessage.includes('associated filter values') ||
          errorMessage.includes('associated filter value')
        ) {
          const category = categories.find((cat) => cat.id === categoryId);
          const categoryName = category?.displayName || 'this category';

          const shouldDeleteAll = window.confirm(
            `Cannot delete ${categoryName} because it has associated filter values.\n\n` +
              `Would you like to delete all values in ${categoryName} first, then delete the category?\n\n` +
              `This will permanently delete all filter values in ${categoryName}.`
          );

          if (shouldDeleteAll) {
            await handleDeleteCategoryWithValues(categoryId);
          }
        } else {
          setError(errorMessage);
        }
        console.error('Error deleting category:', err);
      } finally {
        setIsSubmitting(false);
      }
    }
  };

  // Function to delete all values in a category, then delete the category
  const handleDeleteCategoryWithValues = async (categoryId: string) => {
    try {
      setIsSubmitting(true);
      setError(null);
      setSuccess(null);

      console.log('Deleting category with all its values:', categoryId);

      // First, get all values in the category
      const category = categories.find((cat) => cat.id === categoryId);
      if (!category) {
        throw new Error('Category not found');
      }

      const valuesResponse = await ApiService.getCategoryValues(categoryId);
      if (valuesResponse.success && valuesResponse.data?.values) {
        const values = valuesResponse.data.values;
        console.log(`Found ${values.length} values to delete in category:`, category.displayName);

        // Delete all values one by one
        for (const value of values) {
          console.log('Deleting value:', value.displayName);
          const deleteValueResponse = await ApiService.deleteFilterValue(value.id);
          if (!deleteValueResponse.success) {
            throw new Error(
              `Failed to delete value "${value.displayName}": ${deleteValueResponse.error}`
            );
          }
        }

        // Now delete the category
        const deleteCategoryResponse = await ApiService.deleteFilterCategory(categoryId);
        if (deleteCategoryResponse.success) {
          setCategories((prev) => prev.filter((cat) => cat.id !== categoryId));
          // Remove from allValues state as well
          setAllValues((prev) => {
            const newAllValues = { ...prev };
            delete newAllValues[categoryId];
            return newAllValues;
          });

          setSuccess(
            `Category "${category.displayName}" and all its ${values.length} values deleted successfully!`
          );
          console.log('Category and all values deleted successfully:', categoryId);
        } else {
          throw new Error(
            deleteCategoryResponse.error || 'Failed to delete category after deleting values'
          );
        }
      } else {
        throw new Error('Failed to fetch category values');
      }
    } catch (err) {
      const errorMessage =
        err instanceof Error
          ? err.message
          : 'Failed to delete category with values. Please try again.';
      setError(errorMessage);
      console.error('Error deleting category with values:', err);
    } finally {
      setIsSubmitting(false);
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
      value: value.value,
      displayName: value.displayName,
      description: value.description || '',
      categoryId: value.categoryId,
    });
  };

  const handleChipEdit = (value: any) => {
    setEditingValueData(value);
    setEditValueForm({
      displayName: value.displayName,
      description: value.description || '',
    });
    setEditValueDialogOpen(true);
  };

  const handleEditValueDialogClose = () => {
    setEditValueDialogOpen(false);
    setEditingValueData(null);
    setEditValueForm({
      displayName: '',
      description: '',
    });
    setError(null);
    setSuccess(null);
  };

  const handleEditValueFormChange = (field: string, value: any) => {
    setEditValueForm((prev) => ({
      ...prev,
      [field]: value,
    }));
  };

  const handleSaveEditValue = async () => {
    if (!editValueForm.displayName.trim()) {
      setError('Display name is required');
      return;
    }

    setIsSubmitting(true);
    setError(null);
    setSuccess(null);

    try {
      console.log('Saving edit value:', { editingValueData, editValueForm });

      const response = await ApiService.updateFilterValue(editingValueData.id, {
        value: editingValueData.value,
        displayName: editValueForm.displayName.trim(),
        description: editValueForm.description.trim() || undefined,
      });

      if (response.success && response.data) {
        const updatedValue = response.data;

        // Update local state - categoryValues array
        setCategoryValues((prev) =>
          prev.map((val) => (val.id === editingValueData.id ? updatedValue : val))
        );

        // Update allValues state as well
        setAllValues((prev) => ({
          ...prev,
          [editingValueData.categoryId]:
            prev[editingValueData.categoryId]?.map((val) =>
              val.id === editingValueData.id ? updatedValue : val
            ) || [],
        }));

        setSuccess(`Value "${updatedValue.displayName}" updated successfully!`);
        setTimeout(() => handleEditValueDialogClose(), 1500);
      } else {
        throw new Error(response.error || 'Failed to update value');
      }
    } catch (err) {
      const errorMessage =
        err instanceof Error ? err.message : 'Failed to update value. Please try again.';
      setError(errorMessage);
      console.error('Error updating value:', err);
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleDeleteEditValue = async () => {
    if (
      window.confirm('Are you sure you want to delete this value? This action cannot be undone.')
    ) {
      try {
        setIsSubmitting(true);
        setError(null);
        setSuccess(null);

        console.log('Deleting edit value:', editingValueData);

        const response = await ApiService.deleteFilterValue(editingValueData.id);

        if (response.success) {
          // Update local state - remove from categoryValues
          setCategoryValues((prev) => prev.filter((val) => val.id !== editingValueData.id));

          // Update allValues state as well
          setAllValues((prev) => ({
            ...prev,
            [editingValueData.categoryId]:
              prev[editingValueData.categoryId]?.filter((val) => val.id !== editingValueData.id) ||
              [],
          }));

          setSuccess('Value deleted successfully!');
          setTimeout(() => handleEditValueDialogClose(), 1500);
        } else {
          throw new Error(response.error || 'Failed to delete value');
        }
      } catch (err) {
        const errorMessage =
          err instanceof Error ? err.message : 'Failed to delete value. Please try again.';
        setError(errorMessage);
        console.error('Error deleting value:', err);
      } finally {
        setIsSubmitting(false);
      }
    }
  };

  const handleValueFormChange = (field: string, value: any) => {
    setValueForm((prev) => ({
      ...prev,
      [field]: value,
    }));
  };

  const handleSaveValue = async () => {
    if (!valueForm.value.trim() || !valueForm.displayName.trim() || !valueForm.categoryId) {
      setError('Value, Display Name, and Category are required');
      return;
    }

    setIsSubmitting(true);
    setError(null);
    setSuccess(null);

    try {
      console.log('Saving value:', { valueForm, editingValue });

      if (editingValue) {
        // Update existing value
        const response = await ApiService.updateFilterValue(editingValue.id, {
          value: valueForm.value.trim(),
          displayName: valueForm.displayName.trim(),
          description: valueForm.description.trim() || undefined,
        });

        if (response.success && response.data) {
          // Update local state - categoryValues array
          const updatedValue = response.data;
          setCategoryValues((prev) =>
            prev.map((val) => (val.id === editingValue.id ? updatedValue : val))
          );

          // Update allValues state as well
          setAllValues((prev) => ({
            ...prev,
            [valueForm.categoryId]:
              prev[valueForm.categoryId]?.map((val) =>
                val.id === editingValue.id ? updatedValue : val
              ) || [],
          }));

          setSuccess(`Value "${updatedValue.displayName}" updated successfully!`);
          // Clear the editing state
          setEditingValue(null);
          setValueForm({
            value: '',
            displayName: '',
            description: '',
            categoryId: '',
          });
          setEditValueDialogOpen(false);
        } else {
          throw new Error(response.error || 'Failed to update value');
        }
      } else {
        // Add new value
        const response = await ApiService.createFilterValue({
          categoryId: valueForm.categoryId,
          value: valueForm.value.trim(),
          displayName: valueForm.displayName.trim(),
          description: valueForm.description.trim() || undefined,
        });

        if (response.success && response.data) {
          // Add to local state - categoryValues array
          const newValue = response.data;
          setCategoryValues((prev) => [...prev, newValue]);

          // Update allValues state as well
          setAllValues((prev) => ({
            ...prev,
            [valueForm.categoryId]: [...(prev[valueForm.categoryId] || []), newValue],
          }));

          setSuccess(`Value "${newValue.displayName}" created successfully!`);
          // Clear the form
          setValueForm({
            value: '',
            displayName: '',
            description: '',
            categoryId: editingCategory?.id || '',
          });
          // Close the add value dialog
          setAddValueDialogOpen(false);
        } else {
          throw new Error(response.error || 'Failed to create value');
        }
      }
    } catch (err) {
      const errorMessage =
        err instanceof Error ? err.message : 'Failed to save value. Please try again.';
      setError(errorMessage);
      console.error('Error saving value:', err);
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleDeleteValue = async (valueId: string, categoryId?: string) => {
    if (
      window.confirm('Are you sure you want to delete this value? This action cannot be undone.')
    ) {
      try {
        setIsSubmitting(true);
        setError(null);
        setSuccess(null);

        console.log('Deleting value:', { valueId, categoryId });

        // Make API call to delete the value
        const response = await ApiService.deleteFilterValue(valueId);

        if (response.success) {
          // Update local state - remove from categoryValues
          setCategoryValues((prev) => prev.filter((val) => val.id !== valueId));

          // Update allValues state as well
          if (categoryId) {
            setAllValues((prev) => ({
              ...prev,
              [categoryId]: prev[categoryId]?.filter((val) => val.id !== valueId) || [],
            }));
          }

          setSuccess('Value deleted successfully!');
          console.log('Value deleted successfully:', valueId);
        } else {
          throw new Error(response.error || 'Failed to delete value');
        }
      } catch (err) {
        const errorMessage =
          err instanceof Error ? err.message : 'Failed to delete value. Please try again.';
        setError(errorMessage);
        console.error('Error deleting value:', err);
      } finally {
        setIsSubmitting(false);
      }
    }
  };

  // Debug function to test all API endpoints
  const debugFilterManagement = async () => {
    console.log('=== DEBUGGING FILTER MANAGEMENT ===');

    try {
      // Test 1: Get all categories
      console.log('1. Testing getFilterCategories...');
      const categoriesResponse = await ApiService.getFilterCategories();
      console.log('Categories response:', categoriesResponse);

      if (
        categoriesResponse.success &&
        categoriesResponse.data?.categories &&
        categoriesResponse.data.categories.length > 0
      ) {
        const firstCategory = categoriesResponse.data.categories[0];

        // Test 2: Get category values
        console.log('2. Testing getCategoryValues...');
        const valuesResponse = await ApiService.getCategoryValues(firstCategory.id);
        console.log('Values response:', valuesResponse);

        if (
          valuesResponse.success &&
          valuesResponse.data?.values &&
          valuesResponse.data.values.length > 0
        ) {
          const firstValue = valuesResponse.data.values[0];

          // Test 3: Update a value
          console.log('3. Testing updateFilterValue...');
          const updateResponse = await ApiService.updateFilterValue(firstValue.id, {
            displayName: firstValue.displayName + ' (test)',
          });
          console.log('Update value response:', updateResponse);

          // Test 4: Update back to original
          if (updateResponse.success) {
            console.log('4. Testing updateFilterValue (revert)...');
            const revertResponse = await ApiService.updateFilterValue(firstValue.id, {
              displayName: firstValue.displayName,
            });
            console.log('Revert value response:', revertResponse);
          }

          // Test 5: Try to delete a value (but don't actually delete it)
          console.log('5. Testing deleteFilterValue (dry run)...');
          console.log('Note: Not actually deleting value to preserve data');
          console.log('Would delete value:', firstValue.displayName, 'with ID:', firstValue.id);
        }

        // Test 6: Update category
        console.log('6. Testing updateFilterCategory...');
        const updateCategoryResponse = await ApiService.updateFilterCategory(firstCategory.id, {
          displayName: firstCategory.displayName + ' (test)',
        });
        console.log('Update category response:', updateCategoryResponse);

        // Test 7: Revert category
        if (updateCategoryResponse.success) {
          console.log('7. Testing updateFilterCategory (revert)...');
          const revertCategoryResponse = await ApiService.updateFilterCategory(firstCategory.id, {
            displayName: firstCategory.displayName,
          });
          console.log('Revert category response:', revertCategoryResponse);
        }

        // Test 8: Try to delete a category (but don't actually delete it)
        console.log('8. Testing deleteFilterCategory (dry run)...');
        console.log('Note: Not actually deleting category to preserve data');
        console.log(
          'Would delete category:',
          firstCategory.displayName,
          'with ID:',
          firstCategory.id
        );
      } else {
        console.log('No categories found for testing');
      }

      console.log('=== DEBUG COMPLETE ===');
    } catch (error) {
      console.error('Debug error:', error);
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
            <Tab label="System Settings" iconPosition="start" />
          </Tabs>
        </Box>

        <TabPanel value={tabValue} index={0}>
          <Box>
            <Box
              sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}
            >
              <Typography variant="h6">Filter Categories & Values</Typography>
              <Box sx={{ display: 'flex', gap: 1 }}>
                <Button
                  variant="outlined"
                  size="small"
                  onClick={debugFilterManagement}
                  sx={{ fontSize: '0.75rem' }}
                >
                  Debug API
                </Button>
                <Button variant="contained" startIcon={<AddIcon />} onClick={handleAddFilterClick}>
                  Add Category
                </Button>
              </Box>
            </Box>

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

            {loading ? (
              <Box display="flex" justifyContent="center" p={3}>
                <CircularProgress />
              </Box>
            ) : (
              <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
                {categories.map((category) => {
                  const categoryValues = allValues[category.id] || [];

                  return (
                    <Paper key={category.id} sx={{ p: 3 }}>
                      {/* Category Header */}
                      <Box
                        sx={{
                          display: 'flex',
                          justifyContent: 'space-between',
                          alignItems: 'flex-start',
                          mb: 2,
                        }}
                      >
                        <Box sx={{ flex: 1 }}>
                          <Typography variant="h6" component="div" sx={{ mb: 1 }}>
                            {category.displayName}
                          </Typography>
                          <Typography variant="body2" color="text.secondary" sx={{ mb: 1 }}>
                            {category.description || 'No description'}
                          </Typography>
                        </Box>
                        <Box sx={{ display: 'flex', gap: 1 }}>
                          <Button
                            size="small"
                            variant="contained"
                            onClick={() => {
                              setEditingCategory(category);
                              setEditingValue(null);
                              setValueForm({
                                value: '',
                                displayName: '',
                                description: '',
                                categoryId: category.id,
                              });
                              setAddValueDialogOpen(true);
                            }}
                          >
                            Add Value
                          </Button>
                          <Button
                            size="small"
                            variant="outlined"
                            onClick={() => handleEditCategory(category)}
                            startIcon={<EditIcon />}
                          >
                            Edit
                          </Button>
                        </Box>
                      </Box>

                      {/* Values Section - Always Visible */}
                      <Box sx={{ pt: 2 }}>
                        {categoryValues.length > 0 ? (
                          <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
                            {categoryValues.map((value) => (
                              <EditableChip
                                key={value.id}
                                value={value.value}
                                displayName={value.displayName}
                                description={value.description}
                                onEdit={() => handleChipEdit(value)}
                                color="primary"
                                variant="outlined"
                                size="small"
                              />
                            ))}
                          </Box>
                        ) : (
                          <Paper
                            sx={{
                              p: 2,
                              textAlign: 'center',
                              backgroundColor: 'background.default',
                            }}
                          >
                            <Typography variant="body2" color="text.secondary">
                              No values found for this category. Add your first value to get
                              started.
                            </Typography>
                          </Paper>
                        )}
                      </Box>
                    </Paper>
                  );
                })}
                {categories.length === 0 && (
                  <Paper sx={{ p: 3, textAlign: 'center' }}>
                    <Typography variant="body1" color="text.secondary">
                      No filter categories found. Create your first category to get started.
                    </Typography>
                  </Paper>
                )}
              </Box>
            )}
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

      {/* Add Filter Dialog */}
      <Dialog open={addFilterDialogOpen} onClose={handleAddFilterClose} maxWidth="sm" fullWidth>
        <DialogTitle>Add New Category</DialogTitle>
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

            <FormControl fullWidth sx={{ mb: 2 }}>
              <FormLabel>Name *</FormLabel>
              <TextField
                value={filterForm.name}
                onChange={(e) => handleFilterFormChange('name', e.target.value)}
                placeholder="e.g., use_cases"
                size="small"
                error={!!validateName(filterForm.name)}
                helperText={validateName(filterForm.name) || 'Internal name (no spaces, lowercase)'}
              />
            </FormControl>

            <FormControl fullWidth sx={{ mb: 2 }}>
              <FormLabel>Display Name *</FormLabel>
              <TextField
                value={filterForm.displayName}
                onChange={(e) => handleFilterFormChange('displayName', e.target.value)}
                placeholder="e.g., Use Cases"
                size="small"
                error={!!validateDisplayName(filterForm.displayName)}
                helperText={
                  validateDisplayName(filterForm.displayName) ||
                  'User-friendly name shown in the UI'
                }
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
                error={!!validateDescription(filterForm.description)}
                helperText={
                  validateDescription(filterForm.description) ||
                  'Optional description (max 500 characters)'
                }
              />
            </FormControl>
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleAddFilterClose} disabled={isSubmitting}>
            Cancel
          </Button>
          <Button
            onClick={handleAddFilterSubmit}
            variant="contained"
            disabled={
              isSubmitting ||
              !!validateName(filterForm.name) ||
              !!validateDisplayName(filterForm.displayName) ||
              !!validateDescription(filterForm.description)
            }
          >
            {isSubmitting ? 'Adding...' : 'Add Category'}
          </Button>
        </DialogActions>
      </Dialog>

      {/* Manage Categories Dialog */}
      <Dialog
        open={editCategoryDialogOpen}
        onClose={() => setEditCategoryDialogOpen(false)}
        maxWidth="sm"
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
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setEditCategoryDialogOpen(false)} disabled={isSubmitting}>
            Cancel
          </Button>
          <Button
            onClick={handleSaveCategory}
            variant="contained"
            disabled={isSubmitting || !categoryForm.name.trim() || !categoryForm.displayName.trim()}
          >
            {isSubmitting ? 'Saving...' : editingCategory ? 'Update Category' : 'Add Category'}
          </Button>
        </DialogActions>
      </Dialog>

      {/* Add Value Dialog */}
      <Dialog
        open={addValueDialogOpen}
        onClose={() => setAddValueDialogOpen(false)}
        maxWidth="sm"
        fullWidth
      >
        <DialogTitle>Add New Value</DialogTitle>
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

            <FormControl fullWidth sx={{ mb: 2 }}>
              <FormLabel>Value *</FormLabel>
              <TextField
                value={valueForm.value}
                onChange={(e) => handleValueFormChange('value', e.target.value)}
                placeholder="e.g., navigation"
                size="small"
                error={!valueForm.value.trim()}
                helperText={
                  !valueForm.value.trim()
                    ? 'Value is required'
                    : 'Internal value (no spaces, lowercase)'
                }
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
                helperText={
                  !valueForm.displayName.trim()
                    ? 'Display name is required'
                    : 'User-friendly name shown in the UI'
                }
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
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setAddValueDialogOpen(false)} disabled={isSubmitting}>
            Cancel
          </Button>
          <Button
            onClick={handleSaveValue}
            variant="contained"
            disabled={isSubmitting || !valueForm.value.trim() || !valueForm.displayName.trim()}
          >
            {isSubmitting ? 'Saving...' : 'Add Value'}
          </Button>
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

      {/* Edit Category Dialog */}
      <Dialog
        open={editCategoryDialogOpen}
        onClose={() => setEditCategoryDialogOpen(false)}
        maxWidth="sm"
        fullWidth
      >
        <DialogTitle>Edit Category: {editingCategory?.displayName}</DialogTitle>
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
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setEditCategoryDialogOpen(false)} disabled={isSubmitting}>
            Cancel
          </Button>
          <Button
            onClick={handleSaveCategory}
            variant="contained"
            disabled={isSubmitting || !categoryForm.name.trim() || !categoryForm.displayName.trim()}
          >
            {isSubmitting ? 'Saving...' : 'Update Category'}
          </Button>
        </DialogActions>
      </Dialog>

      {/* Edit Value Dialog */}
      <Dialog
        open={editValueDialogOpen}
        onClose={handleEditValueDialogClose}
        maxWidth="sm"
        fullWidth
      >
        <DialogTitle>Edit Filter Value: {editingValueData?.displayName}</DialogTitle>
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

            <FormControl fullWidth sx={{ mb: 2 }}>
              <FormLabel>Internal Value</FormLabel>
              <TextField
                value={editingValueData?.value || ''}
                size="small"
                disabled
                helperText="Internal value cannot be changed"
              />
            </FormControl>

            <FormControl fullWidth sx={{ mb: 2 }}>
              <FormLabel>Display Name *</FormLabel>
              <TextField
                value={editValueForm.displayName}
                onChange={(e) => handleEditValueFormChange('displayName', e.target.value)}
                placeholder="e.g., Navigation"
                size="small"
                error={!editValueForm.displayName.trim()}
                helperText={!editValueForm.displayName.trim() ? 'Display name is required' : ''}
              />
            </FormControl>

            <FormControl fullWidth sx={{ mb: 2 }}>
              <FormLabel>Description</FormLabel>
              <TextField
                value={editValueForm.description}
                onChange={(e) => handleEditValueFormChange('description', e.target.value)}
                placeholder="Optional description"
                multiline
                rows={2}
                size="small"
              />
            </FormControl>
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleEditValueDialogClose} disabled={isSubmitting}>
            Cancel
          </Button>
          <Button onClick={handleDeleteEditValue} color="error" disabled={isSubmitting}>
            Delete
          </Button>
          <Button
            onClick={handleSaveEditValue}
            variant="contained"
            disabled={isSubmitting || !editValueForm.displayName.trim()}
          >
            {isSubmitting ? 'Saving...' : 'Save Changes'}
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default AdminPage;
