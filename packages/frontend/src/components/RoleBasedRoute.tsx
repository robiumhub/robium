import React, { ReactNode } from 'react';
import { Navigate } from 'react-router-dom';
import { useAuth } from '../contexts/AuthContext';
import LoadingSpinner from './LoadingSpinner';

interface RoleBasedRouteProps {
  children: React.ReactNode;
  allowedRoles: ('user' | 'admin')[];
  fallbackPath?: string;
}

export const RoleBasedRoute: React.FC<RoleBasedRouteProps> = ({
  children,
  allowedRoles,
  fallbackPath = '/dashboard',
}) => {
  const { user, isLoading } = useAuth();

  if (isLoading) {
    return <LoadingSpinner message="Loading..." fullScreen />;
  }

  if (!user) {
    return <Navigate to="/login" replace />;
  }

  if (!allowedRoles.includes(user.role)) {
    return <Navigate to={fallbackPath} replace />;
  }

  return <>{children}</>;
};

// Higher-order component for role-based protection
export const withRoleProtection = <P extends object>(
  Component: React.ComponentType<P>,
  allowedRoles: ('USER' | 'ADMIN')[],
  fallbackPath?: string
) => {
  return (props: P) => (
    <RoleBasedRoute allowedRoles={allowedRoles} fallbackPath={fallbackPath}>
      <Component {...props} />
    </RoleBasedRoute>
  );
};
