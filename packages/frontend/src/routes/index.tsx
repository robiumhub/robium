import React from 'react';
import { Routes, Route, Navigate, Outlet } from 'react-router-dom';
import { ProtectedRoute } from '../components/ProtectedRoute';
import { RoleBasedRoute } from '../components/RoleBasedRoute';
import Layout from '../components/Layout';
import Login from '../pages/Login';
import Register from '../pages/Register';

import DesignSystemShowcase from '../pages/DesignSystemShowcase';

// Lazy load components for better performance
const Home = React.lazy(() => import('../pages/Home'));
const Projects = React.lazy(() => import('../pages/Projects'));
const ProjectDetails = React.lazy(() => import('../pages/ProjectDetails'));
const ProjectWorkspace = React.lazy(() => import('../pages/ProjectWorkspace'));
const ProjectCreationWizard = React.lazy(
  () => import('../pages/ProjectCreationWizard')
);
const Templates = React.lazy(() => import('../pages/Templates'));
const Datasets = React.lazy(() => import('../pages/Datasets'));
const Modules = React.lazy(() => import('../pages/Modules'));
const Robots = React.lazy(() => import('../pages/Robots'));
const Settings = React.lazy(() => import('../pages/Settings'));
const Profile = React.lazy(() => import('../pages/Profile'));
const AdminDashboard = React.lazy(() => import('../pages/AdminDashboard'));
const ExecutionEnvironment = React.lazy(
  () => import('../pages/ExecutionEnvironment')
);
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
    element: <Navigate to="/home" replace />,
  },
  {
    path: '/home',
    element: (
      <ProtectedRoute>
        <Layout>
          <React.Suspense fallback={<div>Loading...</div>}>
            <Home />
          </React.Suspense>
        </Layout>
      </ProtectedRoute>
    ),
    meta: {
      title: 'Home',
      description: 'AI-powered project assistant and overview',
      requiresAuth: true,
      breadcrumb: 'Home',
      icon: 'home',
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
      description: 'Manage your robotics projects',
      requiresAuth: true,
      breadcrumb: 'Projects',
      icon: 'folder',
    },
    children: [
      {
        path: 'new',
        element: (
          <React.Suspense fallback={<div>Loading...</div>}>
            <ProjectCreationWizard />
          </React.Suspense>
        ),
        meta: {
          title: 'Create New Project',
          description: 'Create a new robotics project',
          requiresAuth: true,
          breadcrumb: 'Create Project',
        },
      },
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
    path: '/workspace/:projectId',
    element: (
      <ProtectedRoute>
        <React.Suspense fallback={<div>Loading...</div>}>
          <ProjectWorkspace />
        </React.Suspense>
      </ProtectedRoute>
    ),
    meta: {
      title: 'Project Workspace',
      description: 'IDE-like workspace for project development',
      requiresAuth: true,
      breadcrumb: 'Workspace',
    },
  },
  {
    path: '/templates',
    element: (
      <ProtectedRoute>
        <Layout>
          <React.Suspense fallback={<div>Loading...</div>}>
            <Templates />
          </React.Suspense>
        </Layout>
      </ProtectedRoute>
    ),
    meta: {
      title: 'Templates',
      description: 'Browse and clone project templates',
      requiresAuth: true,
      breadcrumb: 'Templates',
      icon: 'article',
    },
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
      description: 'Supported robots and compatible modules',
      requiresAuth: true,
      breadcrumb: 'Robots',
      icon: 'smart_toy',
    },
  },
  {
    path: '/datasets',
    element: (
      <ProtectedRoute>
        <Layout>
          <React.Suspense fallback={<div>Loading...</div>}>
            <Datasets />
          </React.Suspense>
        </Layout>
      </ProtectedRoute>
    ),
    meta: {
      title: 'Datasets',
      description: 'Browse and download datasets for robotics projects',
      requiresAuth: true,
      breadcrumb: 'Datasets',
      icon: 'storage',
    },
  },
  {
    path: '/modules',
    element: (
      <ProtectedRoute>
        <Layout>
          <React.Suspense fallback={<div>Loading...</div>}>
            <Modules />
          </React.Suspense>
        </Layout>
      </ProtectedRoute>
    ),
    meta: {
      title: 'Modules',
      description: 'Browse and manage available robotics modules',
      requiresAuth: true,
      breadcrumb: 'Modules',
      icon: 'extension',
    },
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
    path: '/execution',
    element: (
      <ProtectedRoute>
        <Layout>
          <React.Suspense fallback={<div>Loading...</div>}>
            <ExecutionEnvironment />
          </React.Suspense>
        </Layout>
      </ProtectedRoute>
    ),
    meta: {
      title: 'Execution Environment',
      description:
        'Manage containers, monitor resources, and debug applications',
      requiresAuth: true,
      breadcrumb: 'Execution',
      icon: 'terminal',
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
