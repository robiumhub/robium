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
  const { user } = useAuth();
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
  if (user?.role === 'ADMIN') {
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
