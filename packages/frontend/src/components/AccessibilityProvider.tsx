import React, {
  createContext,
  useContext,
  useEffect,
  useRef,
  useState,
} from 'react';
import { Box, Button, Snackbar, Alert } from '@mui/material';
import { styled } from '@mui/material/styles';

// Accessibility Context
interface AccessibilityContextType {
  announceToScreenReader: (
    message: string,
    priority?: 'polite' | 'assertive'
  ) => void;
  setFocus: (elementId: string) => void;
  isHighContrastMode: boolean;
  toggleHighContrastMode: () => void;
}

const AccessibilityContext = createContext<
  AccessibilityContextType | undefined
>(undefined);

export const useAccessibility = () => {
  const context = useContext(AccessibilityContext);
  if (!context) {
    throw new Error(
      'useAccessibility must be used within an AccessibilityProvider'
    );
  }
  return context;
};

// Styled Components for Accessibility
const SkipLink = styled(Button)(({ theme }) => ({
  position: 'absolute',
  top: '-40px',
  left: '6px',
  zIndex: 9999,
  backgroundColor: theme.palette.primary.main,
  color: theme.palette.primary.contrastText,
  padding: theme.spacing(1, 2),
  borderRadius: theme.shape.borderRadius,
  textDecoration: 'none',
  fontSize: '14px',
  fontWeight: 500,
  transition: 'top 0.3s ease',
  '&:focus': {
    top: '6px',
    outline: `2px solid ${theme.palette.primary.main}`,
    outlineOffset: '2px',
  },
  '&:hover': {
    backgroundColor: theme.palette.primary.dark,
  },
}));

const SkipToContent = styled(Button)(({ theme }) => ({
  position: 'absolute',
  top: '-40px',
  left: '6px',
  zIndex: 9999,
  backgroundColor: theme.palette.secondary.main,
  color: theme.palette.secondary.contrastText,
  padding: theme.spacing(1, 2),
  borderRadius: theme.shape.borderRadius,
  textDecoration: 'none',
  fontSize: '14px',
  fontWeight: 500,
  transition: 'top 0.3s ease',
  '&:focus': {
    top: '6px',
    outline: `2px solid ${theme.palette.secondary.main}`,
    outlineOffset: '2px',
  },
  '&:hover': {
    backgroundColor: theme.palette.secondary.dark,
  },
}));

const LiveRegion = styled('div')({
  position: 'absolute',
  left: '-10000px',
  width: '1px',
  height: '1px',
  overflow: 'hidden',
});

interface AccessibilityProviderProps {
  children: React.ReactNode;
}

export const AccessibilityProvider: React.FC<AccessibilityProviderProps> = ({
  children,
}) => {
  const [announcements, setAnnouncements] = useState<
    Array<{ id: string; message: string; priority: 'polite' | 'assertive' }>
  >([]);
  const [isHighContrastMode, setIsHighContrastMode] = useState(false);
  const mainContentRef = useRef<HTMLElement>(null);
  const navigationRef = useRef<HTMLElement>(null);

  // Handle keyboard shortcuts
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      // Alt + H for high contrast toggle
      if (event.altKey && event.key === 'h') {
        event.preventDefault();
        toggleHighContrastMode();
      }

      // Alt + S for skip to content
      if (event.altKey && event.key === 's') {
        event.preventDefault();
        setFocus('main-content');
      }

      // Alt + N for skip to navigation
      if (event.altKey && event.key === 'n') {
        event.preventDefault();
        setFocus('main-navigation');
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, []);

  // Apply high contrast mode
  useEffect(() => {
    if (isHighContrastMode) {
      document.documentElement.setAttribute('data-high-contrast', 'true');
    } else {
      document.documentElement.removeAttribute('data-high-contrast');
    }
  }, [isHighContrastMode]);

  const announceToScreenReader = (
    message: string,
    priority: 'polite' | 'assertive' = 'polite'
  ) => {
    const id = `announcement-${Date.now()}`;
    setAnnouncements((prev) => [...prev, { id, message, priority }]);

    // Remove announcement after 5 seconds
    setTimeout(() => {
      setAnnouncements((prev) =>
        prev.filter((announcement) => announcement.id !== id)
      );
    }, 5000);
  };

  const setFocus = (elementId: string) => {
    const element = document.getElementById(elementId);
    if (element) {
      element.focus();
      element.scrollIntoView({ behavior: 'smooth', block: 'start' });
    }
  };

  const toggleHighContrastMode = () => {
    setIsHighContrastMode((prev) => !prev);
    announceToScreenReader(
      `High contrast mode ${!isHighContrastMode ? 'enabled' : 'disabled'}`,
      'polite'
    );
  };

  const handleSkipToContent = () => {
    setFocus('main-content');
    announceToScreenReader('Skipped to main content', 'polite');
  };

  const handleSkipToNavigation = () => {
    setFocus('main-navigation');
    announceToScreenReader('Skipped to navigation', 'polite');
  };

  const contextValue: AccessibilityContextType = {
    announceToScreenReader,
    setFocus,
    isHighContrastMode,
    toggleHighContrastMode,
  };

  return (
    <AccessibilityContext.Provider value={contextValue}>
      {/* Skip Links */}
      <SkipLink
        onClick={handleSkipToNavigation}
        onFocus={(e) => (e.currentTarget.style.top = '6px')}
        onBlur={(e) => (e.currentTarget.style.top = '-40px')}
      >
        Skip to Navigation
      </SkipLink>

      <SkipToContent
        onClick={handleSkipToContent}
        onFocus={(e) => (e.currentTarget.style.top = '6px')}
        onBlur={(e) => (e.currentTarget.style.top = '-40px')}
      >
        Skip to Content
      </SkipToContent>

      {/* ARIA Live Regions */}
      <LiveRegion
        aria-live="polite"
        aria-atomic="true"
        role="status"
        aria-label="Screen reader announcements"
      >
        {announcements
          .filter((announcement) => announcement.priority === 'polite')
          .map((announcement) => (
            <div key={announcement.id}>{announcement.message}</div>
          ))}
      </LiveRegion>

      <LiveRegion
        aria-live="assertive"
        aria-atomic="true"
        role="alert"
        aria-label="Important screen reader announcements"
      >
        {announcements
          .filter((announcement) => announcement.priority === 'assertive')
          .map((announcement) => (
            <div key={announcement.id}>{announcement.message}</div>
          ))}
      </LiveRegion>

      {/* High Contrast Mode Toggle */}
      <Box
        sx={{
          position: 'fixed',
          top: '10px',
          right: '10px',
          zIndex: 1000,
        }}
      >
        <Button
          variant="outlined"
          size="small"
          onClick={toggleHighContrastMode}
          aria-label={`${isHighContrastMode ? 'Disable' : 'Enable'} high contrast mode`}
          sx={{
            backgroundColor: isHighContrastMode
              ? 'primary.main'
              : 'transparent',
            color: isHighContrastMode ? 'primary.contrastText' : 'primary.main',
            '&:focus': {
              outline: '2px solid',
              outlineOffset: '2px',
            },
          }}
        >
          {isHighContrastMode ? 'HC On' : 'HC Off'}
        </Button>
      </Box>

      {children}
    </AccessibilityContext.Provider>
  );
};

export default AccessibilityProvider;
