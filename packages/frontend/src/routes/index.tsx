import React from 'react';
import { Routes, Route, Navigate, Outlet } from 'react-router-dom';
import { ProtectedRoute } from '../components/ProtectedRoute';
import { RoleBasedRoute } from '../components/RoleBasedRoute';
import Layout from '../components/Layout';
import Login from '../pages/Login';
import Register from '../pages/Register';
import Dashboard from '../pages/Dashboard';
import DesignSystemShowcase from '../pages/DesignSystemShowcase';

// Lazy load components for better performance
const Projects = React.lazy(() => import('../pages/Projects'));
const ProjectDetails = React.lazy(() => import('../pages/ProjectDetails'));
const Robots = React.lazy(() => import('../pages/Robots'));
const RobotDetails = React.lazy(() => import('../pages/RobotDetails'));
const Settings = React.lazy(() => import('../pages/Settings'));
const Profile = React.lazy(() => import('../pages/Profile'));
const AdminDashboard = React.lazy(() => import('../pages/AdminDashboard'));
const NotFound = React.lazy(() => import('../pages/NotFound'));

// Route configuration interface
export interface RouteConfig {
  path: string;
  element: React.ReactNode;
  children?: RouteConfig[];
  meta?: {
    title: string;
    description?: string;
    requiresAuth?: boolean;
    allowedRoles?: string[];
    breadcrumb?: string;
    icon?: string;
  };
}

// Main route configuration
export const routes: RouteConfig[] = [
  {
    path: '/login',
    element: <Login />,
    meta: {
      title: 'Login',
      description: 'Sign in to your account',
    },
  },
  {
    path: '/register',
    element: <Register />,
    meta: {
      title: 'Register',
      description: 'Create a new account',
    },
  },
  {
    path: '/',
    element: <Navigate to="/dashboard" replace />,
  },
  {
    path: '/dashboard',
    element: (
      <ProtectedRoute>
        <Layout>
          <Dashboard />
        </Layout>
      </ProtectedRoute>
    ),
    meta: {
      title: 'Dashboard',
      description: 'Overview of your projects and robots',
      requiresAuth: true,
      breadcrumb: 'Dashboard',
      icon: 'dashboard',
    },
  },
  {
    path: '/projects',
    element: (
      <ProtectedRoute>
        <Layout>
          <React.Suspense fallback={<div>Loading...</div>}>
            <Projects />
          </React.Suspense>
        </Layout>
      </ProtectedRoute>
    ),
    meta: {
      title: 'Projects',
      description: 'Manage your robot projects',
      requiresAuth: true,
      breadcrumb: 'Projects',
      icon: 'folder',
    },
    children: [
      {
        path: ':projectId',
        element: (
          <React.Suspense fallback={<div>Loading...</div>}>
            <ProjectDetails />
          </React.Suspense>
        ),
        meta: {
          title: 'Project Details',
          description: 'View and edit project details',
          requiresAuth: true,
          breadcrumb: 'Project Details',
        },
      },
    ],
  },
  {
    path: '/robots',
    element: (
      <ProtectedRoute>
        <Layout>
          <React.Suspense fallback={<div>Loading...</div>}>
            <Robots />
          </React.Suspense>
        </Layout>
      </ProtectedRoute>
    ),
    meta: {
      title: 'Robots',
      description: 'Manage your robots',
      requiresAuth: true,
      breadcrumb: 'Robots',
      icon: 'smart_toy',
    },
    children: [
      {
        path: ':robotId',
        element: (
          <React.Suspense fallback={<div>Loading...</div>}>
            <RobotDetails />
          </React.Suspense>
        ),
        meta: {
          title: 'Robot Details',
          description: 'View and control robot',
          requiresAuth: true,
          breadcrumb: 'Robot Details',
        },
      },
    ],
  },
  {
    path: '/settings',
    element: (
      <ProtectedRoute>
        <Layout>
          <React.Suspense fallback={<div>Loading...</div>}>
            <Settings />
          </React.Suspense>
        </Layout>
      </ProtectedRoute>
    ),
    meta: {
      title: 'Settings',
      description: 'Application settings and preferences',
      requiresAuth: true,
      breadcrumb: 'Settings',
      icon: 'settings',
    },
  },
  {
    path: '/profile',
    element: (
      <ProtectedRoute>
        <Layout>
          <React.Suspense fallback={<div>Loading...</div>}>
            <Profile />
          </React.Suspense>
        </Layout>
      </ProtectedRoute>
    ),
    meta: {
      title: 'Profile',
      description: 'User profile and account settings',
      requiresAuth: true,
      breadcrumb: 'Profile',
      icon: 'person',
    },
  },
  {
    path: '/admin',
    element: (
      <RoleBasedRoute allowedRoles={['ADMIN']}>
        <Layout>
          <React.Suspense fallback={<div>Loading...</div>}>
            <AdminDashboard />
          </React.Suspense>
        </Layout>
      </RoleBasedRoute>
    ),
    meta: {
      title: 'Admin Dashboard',
      description: 'Administrative controls and system management',
      requiresAuth: true,
      allowedRoles: ['ADMIN'],
      breadcrumb: 'Admin',
      icon: 'admin_panel_settings',
    },
  },
  {
    path: '/design-system',
    element: (
      <ProtectedRoute>
        <Layout>
          <DesignSystemShowcase />
        </Layout>
      </ProtectedRoute>
    ),
    meta: {
      title: 'Design System',
      description: 'Component library and design patterns',
      requiresAuth: true,
      breadcrumb: 'Design System',
      icon: 'palette',
    },
  },
  {
    path: '*',
    element: (
      <React.Suspense fallback={<div>Loading...</div>}>
        <NotFound />
      </React.Suspense>
    ),
    meta: {
      title: 'Page Not Found',
      description: 'The requested page could not be found',
    },
  },
];

// Helper function to render routes recursively
const renderRoutes = (routes: RouteConfig[]): React.ReactNode[] => {
  return routes.map((route) => {
    if (route.children && route.children.length > 0) {
      return (
        <Route key={route.path} path={route.path} element={route.element}>
          {renderRoutes(route.children)}
        </Route>
      );
    }
    return <Route key={route.path} path={route.path} element={route.element} />;
  });
};

// Main AppRoutes component
export const AppRoutes: React.FC = () => {
  return <Routes>{renderRoutes(routes)}</Routes>;
};

export default AppRoutes;
