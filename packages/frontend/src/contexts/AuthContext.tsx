import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { User, LoginInput, CreateUserInput, AuthResponse } from '@robium/shared';
import { ApiService } from '../services/api';

interface AuthContextType {
  user: User | null;
  loading: boolean;
  login: (credentials: LoginInput) => Promise<void>;
  register: (userData: CreateUserInput) => Promise<void>;
  logout: () => void;
  updateUser: (updatedUser: User) => void;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const token = localStorage.getItem('authToken');
    if (token) {
      ApiService.getCurrentUser()
        .then((response) => {
          if (response.success && response.data) {
            setUser(response.data);
          }
        })
        .catch(() => {
          localStorage.removeItem('authToken');
        })
        .finally(() => {
          setLoading(false);
        });
    } else {
      setLoading(false);
    }
  }, []);

  const login = async (credentials: LoginInput) => {
    const response = await ApiService.login(credentials);
    if (response.success && response.data) {
      const { user, token } = response.data;
      localStorage.setItem('authToken', token);
      setUser(user);
    } else {
      throw new Error(response.error || 'Login failed');
    }
  };

  const register = async (userData: CreateUserInput) => {
    const response = await ApiService.signup(userData);
    if (response.success && response.data) {
      const { user, token } = response.data;
      localStorage.setItem('authToken', token);
      setUser(user);
    } else {
      throw new Error(response.error || 'Registration failed');
    }
  };

  const logout = () => {
    localStorage.removeItem('authToken');
    setUser(null);
  };

  const updateUser = (updatedUser: User) => {
    setUser(updatedUser);
  };

  const value: AuthContextType = {
    user,
    loading,
    login,
    register,
    logout,
    updateUser,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};
