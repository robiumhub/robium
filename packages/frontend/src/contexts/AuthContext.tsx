import React, {
  createContext,
  useContext,
  useState,
  useEffect,
  ReactNode,
} from 'react';
import ApiService, {
  User,
  LoginRequest,
  RegisterRequest,
} from '../services/api';

interface AuthContextType {
  user: User | null;
  token: string | null;
  login: (email: string, password: string) => Promise<void>;
  register: (
    username: string,
    email: string,
    password: string
  ) => Promise<void>;
  logout: () => void;
  refreshToken: () => Promise<void>;
  isLoading: boolean;
  error: string | null;
  clearError: () => void;
}

// Create context
const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [token, setToken] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(() => {
    // Initialize error from localStorage to persist across re-mounts
    const storedError = localStorage.getItem('auth_error');
    return storedError || null;
  });

  // Clear error function
  const clearError = () => {
    setError(null);
    localStorage.removeItem('auth_error');
  };

  // Set error with localStorage persistence
  const setErrorWithLog = (error: string | null) => {
    setError(error);

    // Persist error in localStorage to survive re-mounts
    if (error) {
      localStorage.setItem('auth_error', error);
    } else {
      localStorage.removeItem('auth_error');
    }
  };

  // Check for existing token on mount and validate it
  useEffect(() => {
    const initializeAuth = async () => {
      try {
        const storedToken = localStorage.getItem('token');
        const storedUser = localStorage.getItem('user');

        if (storedToken && storedUser) {
          try {
            // Validate token by getting current user
            const currentUser = await ApiService.getCurrentUser();
            setToken(storedToken);
            setUser(currentUser);
          } catch (error) {
            // Token is invalid, clear storage silently
            localStorage.removeItem('token');
            localStorage.removeItem('user');
            console.log('Invalid token cleared, redirecting to login');
          }
        }
      } catch (error) {
        // Network error or other issues - clear storage and continue
        console.log('Auth initialization error, clearing storage:', error);
        localStorage.removeItem('token');
        localStorage.removeItem('user');
      } finally {
        setIsLoading(false);
      }
    };

    initializeAuth();
  }, []);

  const login = async (email: string, password: string) => {
    try {
      setIsLoading(true);
      setErrorWithLog(null);

      const response = await ApiService.login({ email, password });

      setUser(response.user);
      setToken(response.token);
      localStorage.setItem('token', response.token);
      localStorage.setItem('user', JSON.stringify(response.user));
    } catch (error) {
      const errorMessage =
        error instanceof Error ? error.message : 'Login failed';
      setErrorWithLog(errorMessage);
      throw error;
    } finally {
      setIsLoading(false);
    }
  };

  const register = async (
    username: string,
    email: string,
    password: string
  ) => {
    try {
      setIsLoading(true);
      setErrorWithLog(null);

      const response = await ApiService.register({
        username,
        email,
        password,
      });

      setUser(response.user);
      setToken(response.token);
      localStorage.setItem('token', response.token);
      localStorage.setItem('user', JSON.stringify(response.user));
    } catch (error) {
      setErrorWithLog(
        error instanceof Error ? error.message : 'Registration failed'
      );
      throw error;
    } finally {
      setIsLoading(false);
    }
  };

  const refreshToken = async () => {
    try {
      const response = await ApiService.refreshToken();

      localStorage.setItem('token', response.token);
      setToken(response.token);
    } catch (error) {
      console.error('Token refresh failed:', error);
      logout();
      throw error;
    }
  };

  // Automatic token refresh
  useEffect(() => {
    if (!token) return;

    // Refresh token 5 minutes before expiration (assuming 1 hour tokens)
    const refreshInterval = setInterval(
      async () => {
        try {
          await refreshToken();
        } catch (error) {
          console.error('Automatic token refresh failed:', error);
          // If refresh fails, logout user
          logout();
        }
      },
      55 * 60 * 1000
    ); // 55 minutes

    return () => clearInterval(refreshInterval);
  }, [token, refreshToken]);

  const logout = async () => {
    try {
      // Call logout endpoint
      await ApiService.logout();
    } catch (error) {
      console.warn('Logout server call failed:', error);
    } finally {
      // Always clear local storage and state
      localStorage.removeItem('token');
      localStorage.removeItem('user');
      localStorage.removeItem('auth_error');
      setToken(null);
      setUser(null);
      setError(null);
    }
  };

  const value: AuthContextType = {
    user,
    token,
    login,
    register,
    logout,
    refreshToken,
    isLoading,
    error,
    clearError,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

// Custom hook to use auth context
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
