import React, { useState } from 'react';
import {
  AppBar,
  Toolbar,
  Typography,
  Box,
  Drawer,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  IconButton,
  Button,
  Avatar,
  Menu,
  MenuItem,
  Divider,
  useTheme,
  useMediaQuery,
} from '@mui/material';
import {
  Menu as MenuIcon,
  Dashboard as DashboardIcon,
  LibraryBooks as TemplatesIcon,
  Settings as SettingsIcon,
  Person as PersonIcon,
  AdminPanelSettings as AdminIcon,
  Logout as LogoutIcon,
  AccountCircle as AccountIcon,
} from '@mui/icons-material';
import { useNavigate, useLocation } from 'react-router-dom';
import { useAuth } from '../contexts/AuthContext';

const drawerWidth = 240;

interface AppShellProps {
  children: React.ReactNode;
}

const AppShell: React.FC<AppShellProps> = ({ children }) => {
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('md'));
  const [mobileOpen, setMobileOpen] = useState(false);
  const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
  const navigate = useNavigate();
  const location = useLocation();
  const { user, logout } = useAuth();

  const menuItems = [
    { text: 'Projects', icon: <DashboardIcon />, path: '/projects' },
    { text: 'Templates', icon: <TemplatesIcon />, path: '/templates' },
  ];

  const handleDrawerToggle = () => {
    setMobileOpen(!mobileOpen);
  };

  const handleNavigation = (path: string) => {
    navigate(path);
    if (isMobile) {
      setMobileOpen(false);
    }
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

  const drawer = (
    <Box>
      <Toolbar>
        <Typography variant="h6" noWrap component="div">
          Robium
        </Typography>
      </Toolbar>
      <List>
        {menuItems.map((item) => (
          <ListItem
            button
            key={item.text}
            onClick={() => handleNavigation(item.path)}
            selected={location.pathname === item.path}
          >
            <ListItemIcon>{item.icon}</ListItemIcon>
            <ListItemText primary={item.text} />
          </ListItem>
        ))}
        <Divider />
        <ListItem button onClick={() => navigate('/settings')}>
          <ListItemIcon>
            <SettingsIcon />
          </ListItemIcon>
          <ListItemText primary="Settings" />
        </ListItem>
        <ListItem button onClick={handleProfile}>
          <ListItemIcon>
            <PersonIcon />
          </ListItemIcon>
          <ListItemText primary="Profile" />
        </ListItem>
        {user?.role === 'admin' && (
          <ListItem button onClick={() => navigate('/admin')}>
            <ListItemIcon>
              <AdminIcon />
            </ListItemIcon>
            <ListItemText primary="Admin" />
          </ListItem>
        )}
        <ListItem button onClick={handleLogout}>
          <ListItemIcon>
            <LogoutIcon />
          </ListItemIcon>
          <ListItemText primary="Logout" />
        </ListItem>
      </List>
    </Box>
  );

  return (
    <Box sx={{ display: 'flex', height: '100vh' }}>
      {/* Top App Bar with Navigation */}
      <AppBar
        position="fixed"
        sx={{
          width: '100%',
          zIndex: (theme) => theme.zIndex.drawer + 1,
          backgroundColor: 'white',
          color: 'text.primary',
          boxShadow: 1,
        }}
      >
        <Toolbar sx={{ justifyContent: 'space-between', px: { xs: 1, md: 3 } }}>
          {/* Left side - Logo and Mobile Menu */}
          <Box sx={{ display: 'flex', alignItems: 'center', gap: { xs: 1, md: 3 } }}>
            {/* Mobile Menu Button */}
            <IconButton
              color="inherit"
              aria-label="open drawer"
              edge="start"
              onClick={handleDrawerToggle}
              sx={{ mr: 2, display: { sm: 'none' } }}
            >
              <MenuIcon />
            </IconButton>

            {/* Logo */}
            <Typography
              variant="h6"
              noWrap
              component="div"
              sx={{
                color: 'primary.main',
                fontWeight: 600,
                cursor: 'pointer',
                '&:hover': { opacity: 0.8 },
              }}
              onClick={() => handleNavigation('/projects')}
            >
              Robium
            </Typography>
            
            {/* Testing Indicator */}
            <Typography
              variant="caption"
              sx={{
                color: 'warning.main',
                fontWeight: 600,
                ml: 1,
                px: 1,
                py: 0.5,
                bgcolor: 'warning.light',
                borderRadius: 1,
                fontSize: '0.7rem',
              }}
            >
              TESTING
            </Typography>
          </Box>

          {/* Center - Desktop Navigation */}
          {!isMobile && (
            <Box sx={{ display: 'flex', gap: 1 }}>
              {menuItems.map((item) => (
                <Button
                  key={item.path}
                  startIcon={item.icon}
                  onClick={() => handleNavigation(item.path)}
                  sx={{
                    color: 'text.primary',
                    transition: 'opacity 0.2s ease',
                    '&:hover': {
                      backgroundColor: 'transparent',
                      opacity: 0.7,
                    },
                    ...(location.pathname === item.path && {
                      color: 'primary.main',
                      fontWeight: 600,
                      '&:hover': {
                        backgroundColor: 'transparent',
                        opacity: 0.8,
                      },
                    }),
                  }}
                >
                  {item.text}
                </Button>
              ))}
            </Box>
          )}

          {/* Right side - User Menu */}
          <Box sx={{ display: 'flex', alignItems: 'center', ml: 2 }}>
            {user ? (
              <>
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
                  <MenuItem onClick={() => navigate('/settings')}>
                    <ListItemIcon>
                      <SettingsIcon fontSize="small" />
                    </ListItemIcon>
                    Settings
                  </MenuItem>
                  {user.role === 'admin' && (
                    <MenuItem onClick={() => navigate('/admin')}>
                      <ListItemIcon>
                        <AdminIcon fontSize="small" />
                      </ListItemIcon>
                      Admin
                    </MenuItem>
                  )}
                  <MenuItem onClick={handleLogout}>
                    <ListItemIcon>
                      <LogoutIcon fontSize="small" />
                    </ListItemIcon>
                    Logout
                  </MenuItem>
                </Menu>
              </>
            ) : (
              <Button
                color="inherit"
                onClick={() => navigate('/login')}
                sx={{ color: 'primary.main' }}
              >
                Login
              </Button>
            )}
          </Box>
        </Toolbar>
      </AppBar>

      {/* Mobile Drawer */}
      <Box
        component="nav"
        sx={{ display: { sm: 'none' } }}
      >
        <Drawer
          variant="temporary"
          open={mobileOpen}
          onClose={handleDrawerToggle}
          ModalProps={{
            keepMounted: true,
          }}
          sx={{
            display: { xs: 'block', sm: 'none' },
            '& .MuiDrawer-paper': {
              boxSizing: 'border-box',
              width: drawerWidth,
            },
          }}
        >
          {drawer}
        </Drawer>
      </Box>

      {/* Main Content */}
      <Box
        component="main"
        sx={{
          flexGrow: 1,
          p: 0,
          width: '100%',
          minHeight: '100vh',
          pt: '64px', // Account for top bar
        }}
      >
        <Box
          sx={{
            width: '100%',
            px: { xs: 2, md: 4 },
            mt: { xs: 1, sm: 2 },
          }}
        >
          {children}
        </Box>
      </Box>
    </Box>
  );
};

export default AppShell;
