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
  Button,
  useTheme,
  useMediaQuery,
  TextField,
  InputAdornment,
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
  Menu as MenuIcon,
  Search as SearchIcon,
  Clear as ClearIcon,
} from '@mui/icons-material';
import { useAuth } from '../contexts/AuthContext';
import { useNavigate, useLocation, Link } from 'react-router-dom';
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
  const location = useLocation();
  const { activeMenuItem, setActiveMenuItem } = useNavigation();
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('md'));
  const [searchQuery, setSearchQuery] = useState('');

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

  const handleSearch = (e: React.FormEvent) => {
    e.preventDefault();
    if (searchQuery.trim()) {
      // Navigate to search results or templates page with search query
      navigate(`/templates?search=${encodeURIComponent(searchQuery.trim())}`);
    }
  };

  const handleClearSearch = () => {
    setSearchQuery('');
  };

  const menuItems = [
    { text: 'Home', icon: <HomeIcon />, path: '/' },
    { text: 'My Projects', icon: <FolderIcon />, path: '/projects' },
    { text: 'Templates', icon: <TemplateIcon />, path: '/templates' },
    { text: 'Modules', icon: <ModuleIcon />, path: '/modules' },
    { text: 'Datasets', icon: <StorageIcon />, path: '/datasets' },
  ];

  // Don't add admin to main menu - it will be in user dropdown only

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
        <Divider />
        <ListItem disablePadding>
          <ListItemButton onClick={() => handleNavigation('/settings')}>
            <ListItemIcon>
              <SettingsIcon />
            </ListItemIcon>
            <ListItemText primary="Settings" />
          </ListItemButton>
        </ListItem>
      </List>
    </div>
  );

  return (
    <Box sx={{ display: 'flex', height: '100vh' }}>
      <CssBaseline />

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
          {/* Left side - Logo, Search, and Navigation */}
          <Box
            sx={{
              display: 'flex',
              alignItems: 'center',
              gap: { xs: 1, md: 3 },
              flex: 1,
            }}
          >
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
              onClick={() => handleNavigation('/')}
            >
              Robium
            </Typography>

            {/* Search Bar - Desktop */}
            {!isMobile && (
              <Box
                component="form"
                onSubmit={handleSearch}
                sx={{
                  flexGrow: 1,
                  maxWidth: 600,
                  mx: 3,
                }}
              >
                <TextField
                  fullWidth
                  placeholder="Search templates, modules, robots..."
                  value={searchQuery}
                  onChange={(e) => setSearchQuery(e.target.value)}
                  InputProps={{
                    startAdornment: (
                      <InputAdornment position="start">
                        <SearchIcon sx={{ color: 'text.secondary' }} />
                      </InputAdornment>
                    ),
                    endAdornment: searchQuery && (
                      <InputAdornment position="end">
                        <IconButton size="small" onClick={handleClearSearch}>
                          <ClearIcon fontSize="small" />
                        </IconButton>
                      </InputAdornment>
                    ),
                    sx: {
                      backgroundColor: 'white',
                      borderRadius: 2,
                      '& .MuiOutlinedInput-root': {
                        '& fieldset': {
                          borderColor: 'transparent',
                        },
                        '&:hover fieldset': {
                          borderColor: 'transparent',
                        },
                        '&.Mui-focused fieldset': {
                          borderColor: 'transparent',
                        },
                      },
                    },
                  }}
                  size="small"
                />
              </Box>
            )}

            {/* Desktop Navigation */}
            {!isMobile && (
              <Box sx={{ display: 'flex', gap: 1, ml: 'auto' }}>
                {menuItems.map((item) => (
                  <Button
                    key={item.path}
                    component={Link}
                    to={item.path}
                    startIcon={item.icon}
                    sx={{
                      color: 'text.primary',
                      transition: 'opacity 0.2s ease',
                      '&:hover': {
                        backgroundColor: 'transparent',
                        opacity: 0.7,
                      },
                      '&:focus': {
                        outline: 'none',
                      },
                      '&:focus-visible': {
                        outline: 'none',
                      },
                      ...(activeMenuItem === item.path && {
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
          </Box>

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

      {/* Mobile Search Bar */}
      {isMobile && (
        <Box
          component="form"
          onSubmit={handleSearch}
          sx={{
            position: 'fixed',
            top: 64,
            left: 0,
            right: 0,
            zIndex: theme.zIndex.drawer,
            p: 1,
            backgroundColor: 'background.paper',
            borderBottom: 1,
            borderColor: 'divider',
          }}
        >
          <TextField
            fullWidth
            placeholder="Search templates, modules, robots..."
            value={searchQuery}
            onChange={(e) => setSearchQuery(e.target.value)}
            InputProps={{
              startAdornment: (
                <InputAdornment position="start">
                  <SearchIcon />
                </InputAdornment>
              ),
              endAdornment: searchQuery && (
                <InputAdornment position="end">
                  <IconButton size="small" onClick={handleClearSearch}>
                    <ClearIcon fontSize="small" />
                  </IconButton>
                </InputAdornment>
              ),
            }}
            size="small"
          />
        </Box>
      )}

      {/* Mobile Drawer */}
      <Box
        component="nav"
        id="main-navigation"
        role="navigation"
        aria-label="Main navigation"
        sx={{ display: { sm: 'none' } }}
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
      </Box>

      {/* Main Content */}
      <Box
        component="main"
        id="main-content"
        role="main"
        aria-label="Main content"
        sx={{
          flexGrow: 1,
          p: 0,
          width: '100%',
          minHeight: '100vh',
          background: 'linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%)',
          pt: isMobile ? '120px' : '64px', // Account for search bar on mobile
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

export default Layout;
