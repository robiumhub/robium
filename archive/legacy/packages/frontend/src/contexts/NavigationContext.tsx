import React, {
  createContext,
  useContext,
  useState,
  useEffect,
  ReactNode,
} from 'react';
import { useLocation } from 'react-router-dom';
import { routes, RouteConfig } from '../routes';

interface BreadcrumbItem {
  label: string;
  path?: string;
  icon?: string;
}

interface NavigationContextType {
  currentRoute: RouteConfig | null;
  breadcrumbs: BreadcrumbItem[];
  activeMenuItem: string;
  setActiveMenuItem: (path: string) => void;
  getRouteByPath: (path: string) => RouteConfig | null;
}

const NavigationContext = createContext<NavigationContextType | undefined>(
  undefined
);

interface NavigationProviderProps {
  children: ReactNode;
}

export const NavigationProvider: React.FC<NavigationProviderProps> = ({
  children,
}) => {
  const location = useLocation();
  const [activeMenuItem, setActiveMenuItem] = useState<string>('/dashboard');

  // Find route by path recursively
  const findRouteByPath = (
    path: string,
    routeList: RouteConfig[]
  ): RouteConfig | null => {
    for (const route of routeList) {
      if (route.path === path) {
        return route;
      }
      if (route.children) {
        const childRoute = findRouteByPath(path, route.children);
        if (childRoute) {
          return childRoute;
        }
      }
    }
    return null;
  };

  // Generate breadcrumbs from current path
  const generateBreadcrumbs = (pathname: string): BreadcrumbItem[] => {
    const pathnames = pathname.split('/').filter((x) => x);
    const breadcrumbs: BreadcrumbItem[] = [
      { label: 'Home', path: '/dashboard', icon: 'home' },
    ];

    let currentPath = '';
    pathnames.forEach((name, index) => {
      currentPath += `/${name}`;
      const route = findRouteByPath(currentPath, routes);

      if (route?.meta?.breadcrumb) {
        const label = route.meta.breadcrumb;
        const icon = route.meta.icon;

        if (index === pathnames.length - 1) {
          breadcrumbs.push({ label, icon });
        } else {
          breadcrumbs.push({ label, path: currentPath, icon });
        }
      } else {
        // Fallback for routes without explicit breadcrumb config
        const label = name.charAt(0).toUpperCase() + name.slice(1);
        if (index === pathnames.length - 1) {
          breadcrumbs.push({ label });
        } else {
          breadcrumbs.push({ label, path: currentPath });
        }
      }
    });

    return breadcrumbs;
  };

  // Get current route based on location
  const currentRoute = findRouteByPath(location.pathname, routes);

  // Generate breadcrumbs
  const breadcrumbs = generateBreadcrumbs(location.pathname);

  // Update active menu item when location changes
  useEffect(() => {
    setActiveMenuItem(location.pathname);
  }, [location.pathname]);

  const getRouteByPath = (path: string): RouteConfig | null => {
    return findRouteByPath(path, routes);
  };

  const value: NavigationContextType = {
    currentRoute,
    breadcrumbs,
    activeMenuItem,
    setActiveMenuItem,
    getRouteByPath,
  };

  return (
    <NavigationContext.Provider value={value}>
      {children}
    </NavigationContext.Provider>
  );
};

export const useNavigation = (): NavigationContextType => {
  const context = useContext(NavigationContext);
  if (context === undefined) {
    throw new Error('useNavigation must be used within a NavigationProvider');
  }
  return context;
};
