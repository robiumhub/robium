import React, { ReactNode, useState } from 'react';
import {
  Box,
  CssBaseline,
  Drawer,
  List,
  ListItem,
  ListItemButton,
  ListItemIcon,
  ListItemText,
  Toolbar,
  Typography,
  Divider,
  AppBar,
  IconButton,
  Menu,
  MenuItem,
  Avatar,
  Chip,
} from '@mui/material';
import {
  Settings as SettingsIcon,
  Folder as FolderIcon,
  AdminPanelSettings as AdminIcon,
  Extension as ModuleIcon,
  Article as TemplateIcon,
  Storage as StorageIcon,
  Home as HomeIcon,
  Code as CodeIcon,
  SmartToy as RobotIcon,
  AccountCircle as AccountIcon,
  Logout as LogoutIcon,
  Person as PersonIcon,
} from '@mui/icons-material';
import { useAuth } from '../contexts/AuthContext';
import { useNavigate } from 'react-router-dom';
import { useNavigation } from '../contexts/NavigationContext';

const drawerWidth = 240;
const mobileDrawerWidth = 280;

interface LayoutProps {
  children: ReactNode;
}

const Layout: React.FC<LayoutProps> = ({ children }) => {
  const [mobileOpen, setMobileOpen] = useState(false);
  const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
  const { user, logout } = useAuth();
  const navigate = useNavigate();
  const { activeMenuItem, setActiveMenuItem } = useNavigation();

  const handleDrawerToggle = () => {
    setMobileOpen(!mobileOpen);
  };

  const handleNavigation = (path: string) => {
    setActiveMenuItem(path);
    navigate(path);
    setMobileOpen(false);
  };

  const handleUserMenuOpen = (event: React.MouseEvent<HTMLElement>) => {
    setAnchorEl(event.currentTarget);
  };

  const handleUserMenuClose = () => {
    setAnchorEl(null);
  };

  const handleLogout = () => {
    logout();
    handleUserMenuClose();
    navigate('/login');
  };

  const handleProfile = () => {
    handleUserMenuClose();
    navigate('/profile');
  };

  const menuItems = [
    { text: 'Home', icon: <HomeIcon />, path: '/' },
    { text: 'Projects', icon: <FolderIcon />, path: '/projects' },
    { text: 'Modules', icon: <ModuleIcon />, path: '/modules' },
    { text: 'Templates', icon: <TemplateIcon />, path: '/templates' },
    { text: 'Robots', icon: <RobotIcon />, path: '/robots' },
    { text: 'Datasets', icon: <StorageIcon />, path: '/datasets' },
    { text: 'Settings', icon: <SettingsIcon />, path: '/settings' },
  ];

  // Add admin menu item if user has admin role
  if (user?.role === 'admin') {
    menuItems.push({ text: 'Admin', icon: <AdminIcon />, path: '/admin' });
  }

  const drawer = (
    <div>
      <Toolbar>
        <Typography variant="h6" noWrap component="div">
          Robium
        </Typography>
      </Toolbar>
      <Divider />
      <List>
        {menuItems.map((item) => (
          <ListItem key={item.text} disablePadding>
            <ListItemButton
              onClick={() => handleNavigation(item.path)}
              selected={activeMenuItem === item.path}
              sx={{
                '&.Mui-selected': {
                  backgroundColor: 'primary.light',
                  '&:hover': {
                    backgroundColor: 'primary.light',
                  },
                },
              }}
            >
              <ListItemIcon>{item.icon}</ListItemIcon>
              <ListItemText primary={item.text} />
            </ListItemButton>
          </ListItem>
        ))}
      </List>
    </div>
  );

  return (
    <Box sx={{ display: 'flex', height: '100vh' }}>
      <CssBaseline />

      {/* Top App Bar */}
      <AppBar
        position="fixed"
        sx={{
          width: { sm: `calc(100% - ${drawerWidth}px)` },
          ml: { sm: `${drawerWidth}px` },
          zIndex: (theme) => theme.zIndex.drawer + 1,
          backgroundColor: 'white',
          color: 'text.primary',
          boxShadow: 1,
        }}
      >
        <Toolbar sx={{ justifyContent: 'space-between' }}>
          <Box sx={{ display: 'flex', alignItems: 'center' }}>
            <Typography
              variant="h6"
              noWrap
              component="div"
              sx={{ color: 'primary.main' }}
            >
              Robium Platform
            </Typography>
          </Box>

          {/* User Menu */}
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
            {user && (
              <>
                <Chip
                  label={user.role === 'admin' ? 'Admin' : 'User'}
                  size="small"
                  color={user.role === 'admin' ? 'error' : 'default'}
                  variant="outlined"
                />
                <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                  <Typography variant="body2" sx={{ color: 'text.secondary' }}>
                    {user.username}
                  </Typography>
                  <IconButton
                    onClick={handleUserMenuOpen}
                    sx={{ color: 'primary.main' }}
                    aria-label="user menu"
                  >
                    <Avatar
                      sx={{ width: 32, height: 32, bgcolor: 'primary.main' }}
                    >
                      <AccountIcon />
                    </Avatar>
                  </IconButton>
                </Box>
              </>
            )}
          </Box>
        </Toolbar>
      </AppBar>

      {/* User Menu Dropdown */}
      <Menu
        anchorEl={anchorEl}
        open={Boolean(anchorEl)}
        onClose={handleUserMenuClose}
        anchorOrigin={{
          vertical: 'bottom',
          horizontal: 'right',
        }}
        transformOrigin={{
          vertical: 'top',
          horizontal: 'right',
        }}
      >
        <MenuItem onClick={handleProfile}>
          <ListItemIcon>
            <PersonIcon fontSize="small" />
          </ListItemIcon>
          Profile
        </MenuItem>
        <Divider />
        <MenuItem onClick={handleLogout}>
          <ListItemIcon>
            <LogoutIcon fontSize="small" />
          </ListItemIcon>
          Logout
        </MenuItem>
      </Menu>

      <Box
        component="nav"
        id="main-navigation"
        role="navigation"
        aria-label="Main navigation"
        sx={{ width: { sm: drawerWidth }, flexShrink: { sm: 0 } }}
      >
        <Drawer
          variant="temporary"
          open={mobileOpen}
          onClose={handleDrawerToggle}
          ModalProps={{
            keepMounted: true, // Better open performance on mobile.
          }}
          sx={{
            display: { xs: 'block', sm: 'none' },
            '& .MuiDrawer-paper': {
              boxSizing: 'border-box',
              width: mobileDrawerWidth,
            },
          }}
        >
          {drawer}
        </Drawer>
        <Drawer
          variant="permanent"
          sx={{
            display: { xs: 'none', sm: 'block' },
            '& .MuiDrawer-paper': {
              boxSizing: 'border-box',
              width: drawerWidth,
              borderRight: 'none',
              bgcolor: 'grey.50',
            },
          }}
          open
        >
          {drawer}
        </Drawer>
      </Box>
      <Box
        component="main"
        id="main-content"
        role="main"
        aria-label="Main content"
        sx={{
          flexGrow: 1,
          p: 0, // Remove padding from main container for all pages
          width: { sm: `calc(100% - ${drawerWidth}px)` },
          minHeight: '100vh',
          background: 'linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%)', // Same gradient as home page
          pt: '64px', // Add top padding to account for the AppBar
        }}
      >
        {/* Breadcrumb Section - Unified for all pages */}
        <Box sx={{ py: 2, px: { xs: 2, md: 4 } }}>
          <Box
            sx={{
              display: 'flex',
              alignItems: 'center',
              gap: 1,
              color: 'text.secondary',
            }}
          >
            {(activeMenuItem === '/projects' ||
              activeMenuItem.startsWith('/projects/')) && (
              <FolderIcon sx={{ fontSize: 20 }} />
            )}
            {activeMenuItem === '/modules' && (
              <ModuleIcon sx={{ fontSize: 20 }} />
            )}
            {activeMenuItem === '/templates' && (
              <TemplateIcon sx={{ fontSize: 20 }} />
            )}
            {activeMenuItem === '/robots' && (
              <RobotIcon sx={{ fontSize: 20 }} />
            )}
            {activeMenuItem === '/datasets' && (
              <StorageIcon sx={{ fontSize: 20 }} />
            )}
            {activeMenuItem === '/settings' && (
              <SettingsIcon sx={{ fontSize: 20 }} />
            )}
            {activeMenuItem === '/admin' && <AdminIcon sx={{ fontSize: 20 }} />}
            {activeMenuItem.startsWith('/workspace/') && (
              <CodeIcon sx={{ fontSize: 20 }} />
            )}
            {(activeMenuItem === '/' || activeMenuItem === '/home') && (
              <HomeIcon sx={{ fontSize: 20 }} />
            )}
            <Typography variant="body2" sx={{ fontWeight: 500 }}>
              {activeMenuItem === '/projects' && 'Projects'}
              {activeMenuItem.startsWith('/projects/') &&
                activeMenuItem !== '/projects' && (
                  <>
                    <span
                      style={{ cursor: 'pointer' }}
                      onClick={() => navigate('/projects')}
                    >
                      Projects
                    </span>
                    <span style={{ margin: '0 8px' }}>/</span>
                    {activeMenuItem === '/projects/new' && 'Create Project'}
                    {activeMenuItem.match(/^\/projects\/[^\/]+$/) &&
                      activeMenuItem !== '/projects/new' &&
                      'Project Details'}
                  </>
                )}
              {activeMenuItem === '/modules' && 'Modules'}
              {activeMenuItem === '/templates' && 'Templates'}
              {activeMenuItem === '/datasets' && 'Datasets'}
              {activeMenuItem === '/settings' && 'Settings'}
              {activeMenuItem === '/admin' && 'Admin'}
              {activeMenuItem.startsWith('/workspace/') && 'Workspace'}
              {(activeMenuItem === '/' || activeMenuItem === '/home') && 'Home'}
            </Typography>
          </Box>
        </Box>

        <Box
          sx={{
            width: '100%',
            px: { xs: 2, md: 4 }, // Add consistent horizontal padding for all pages
            mt: { xs: 1, sm: 2 }, // Consistent top margin for all pages
          }}
        >
          {children}
        </Box>
      </Box>
    </Box>
  );
};

export default Layout;
