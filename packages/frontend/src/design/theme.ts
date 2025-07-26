import { createTheme, ThemeOptions } from '@mui/material/styles';
import {
  colors,
  typography,
  spacing,
  borderRadius,
  shadows,
  breakpoints,
  transitions,
} from './designTokens';

// Create a comprehensive theme configuration
const themeOptions: ThemeOptions = {
  palette: {
    mode: 'light',
    primary: {
      main: colors.primary[500],
      light: colors.primary[300],
      dark: colors.primary[700],
      contrastText: colors.text.inverse,
      50: colors.primary[50],
      100: colors.primary[100],
      200: colors.primary[200],
      300: colors.primary[300],
      400: colors.primary[400],
      500: colors.primary[500],
      600: colors.primary[600],
      700: colors.primary[700],
      800: colors.primary[800],
      900: colors.primary[900],
    },
    secondary: {
      main: colors.secondary[500],
      light: colors.secondary[300],
      dark: colors.secondary[700],
      contrastText: colors.text.inverse,
      50: colors.secondary[50],
      100: colors.secondary[100],
      200: colors.secondary[200],
      300: colors.secondary[300],
      400: colors.secondary[400],
      500: colors.secondary[500],
      600: colors.secondary[600],
      700: colors.secondary[700],
      800: colors.secondary[800],
      900: colors.secondary[900],
    },
    success: {
      main: colors.success[500],
      light: colors.success[300],
      dark: colors.success[700],
      contrastText: colors.text.inverse,
    },
    warning: {
      main: colors.warning[500],
      light: colors.warning[300],
      dark: colors.warning[700],
      contrastText: colors.text.primary,
    },
    error: {
      main: colors.error[500],
      light: colors.error[300],
      dark: colors.error[700],
      contrastText: colors.text.inverse,
    },
    info: {
      main: colors.primary[500],
      light: colors.primary[300],
      dark: colors.primary[700],
      contrastText: colors.text.inverse,
    },
    grey: {
      50: colors.neutral[50],
      100: colors.neutral[100],
      200: colors.neutral[200],
      300: colors.neutral[300],
      400: colors.neutral[400],
      500: colors.neutral[500],
      600: colors.neutral[600],
      700: colors.neutral[700],
      800: colors.neutral[800],
      900: colors.neutral[900],
    },
    text: {
      primary: colors.text.primary,
      secondary: colors.text.secondary,
      disabled: colors.text.disabled,
    },
    background: {
      default: colors.background.default,
      paper: colors.background.paper,
    },
    divider: colors.neutral[200],
  },

  typography: {
    fontFamily: typography.fontFamily.primary,
    h1: {
      fontSize: typography.fontSize['4xl'],
      fontWeight: typography.fontWeight.bold,
      lineHeight: typography.lineHeight.tight,
    },
    h2: {
      fontSize: typography.fontSize['3xl'],
      fontWeight: typography.fontWeight.bold,
      lineHeight: typography.lineHeight.tight,
    },
    h3: {
      fontSize: typography.fontSize['2xl'],
      fontWeight: typography.fontWeight.semibold,
      lineHeight: typography.lineHeight.relaxed,
    },
    h4: {
      fontSize: typography.fontSize.xl,
      fontWeight: typography.fontWeight.semibold,
      lineHeight: typography.lineHeight.relaxed,
    },
    h5: {
      fontSize: typography.fontSize.lg,
      fontWeight: typography.fontWeight.medium,
      lineHeight: typography.lineHeight.relaxed,
    },
    h6: {
      fontSize: typography.fontSize.lg,
      fontWeight: typography.fontWeight.medium,
      lineHeight: typography.lineHeight.relaxed,
    },
    body1: {
      fontSize: typography.fontSize.lg,
      fontWeight: typography.fontWeight.normal,
      lineHeight: typography.lineHeight.relaxed,
    },
    body2: {
      fontSize: typography.fontSize.sm,
      fontWeight: typography.fontWeight.normal,
      lineHeight: typography.lineHeight.relaxed,
    },
    button: {
      fontSize: typography.fontSize.sm,
      fontWeight: typography.fontWeight.medium,
      lineHeight: typography.lineHeight.tight,
      textTransform: 'none',
    },
    caption: {
      fontSize: typography.fontSize.xs,
      fontWeight: typography.fontWeight.normal,
      lineHeight: typography.lineHeight.relaxed,
    },
    overline: {
      fontSize: typography.fontSize.xs,
      fontWeight: typography.fontWeight.medium,
      lineHeight: typography.lineHeight.tight,
      textTransform: 'uppercase',
    },
  },

  spacing: 8, // Material-UI spacing unit (8px)

  shape: {
    borderRadius: borderRadius.md,
  },

  shadows: [
    'none',
    shadows.sm,
    shadows.md,
    shadows.lg,
    shadows.xl,
    ...Array(19).fill(shadows.xl), // Material-UI expects 25 shadows
  ],

  breakpoints: {
    values: {
      xs: breakpoints.xs,
      sm: breakpoints.sm,
      md: breakpoints.md,
      lg: breakpoints.lg,
      xl: breakpoints.xl,
    },
  },

  transitions: {
    easing: {
      easeInOut: transitions.easing.easeInOut,
      easeOut: transitions.easing.easeOut,
      easeIn: transitions.easing.easeIn,
      sharp: 'cubic-bezier(0.4, 0, 0.6, 1)',
    },
    duration: {
      shortest: 150,
      shorter: 200,
      short: 250,
      standard: 300,
      complex: 375,
      enteringScreen: 225,
      leavingScreen: 195,
    },
  },

  components: {
    MuiButton: {
      styleOverrides: {
        root: {
          borderRadius: borderRadius.md,
          textTransform: 'none',
          fontWeight: typography.fontWeight.medium,
          '&:focus': {
            outline: '2px solid',
            outlineOffset: '2px',
          },
          '&:focus-visible': {
            outline: '2px solid',
            outlineOffset: '2px',
          },
        },
      },
    },
    MuiTextField: {
      styleOverrides: {
        root: {
          '& .MuiOutlinedInput-root': {
            '&:focus-within': {
              '& .MuiOutlinedInput-notchedOutline': {
                borderWidth: '2px',
              },
            },
          },
        },
      },
    },
    MuiIconButton: {
      styleOverrides: {
        root: {
          '&:focus': {
            outline: '2px solid',
            outlineOffset: '2px',
          },
          '&:focus-visible': {
            outline: '2px solid',
            outlineOffset: '2px',
          },
        },
      },
    },
    MuiLink: {
      styleOverrides: {
        root: {
          '&:focus': {
            outline: '2px solid',
            outlineOffset: '2px',
          },
          '&:focus-visible': {
            outline: '2px solid',
            outlineOffset: '2px',
          },
        },
      },
    },
    MuiCard: {
      styleOverrides: {
        root: {
          '&:focus-within': {
            outline: '2px solid',
            outlineOffset: '2px',
          },
        },
      },
    },
    MuiChip: {
      styleOverrides: {
        root: {
          '&:focus': {
            outline: '2px solid',
            outlineOffset: '2px',
          },
          '&:focus-visible': {
            outline: '2px solid',
            outlineOffset: '2px',
          },
        },
      },
    },
    MuiAlert: {
      styleOverrides: {
        root: {
          '&:focus': {
            outline: '2px solid',
            outlineOffset: '2px',
          },
          '&:focus-visible': {
            outline: '2px solid',
            outlineOffset: '2px',
          },
        },
      },
    },
  },
};

// Create the base theme
const baseTheme = createTheme(themeOptions);

// Create high contrast theme variant
const highContrastTheme = createTheme({
  ...themeOptions,
  palette: {
    ...themeOptions.palette,
    mode: 'light',
    primary: {
      ...themeOptions.palette!.primary!,
      main: '#000000',
      contrastText: '#ffffff',
    },
    secondary: {
      ...themeOptions.palette!.secondary!,
      main: '#ffffff',
      contrastText: '#000000',
    },
    background: {
      default: '#ffffff',
      paper: '#ffffff',
    },
    text: {
      primary: '#000000',
      secondary: '#000000',
      disabled: '#666666',
    },
    divider: '#000000',
  },
  components: {
    ...themeOptions.components,
    MuiButton: {
      styleOverrides: {
        root: {
          ...themeOptions.components?.MuiButton?.styleOverrides?.root,
          border: '2px solid #000000',
          '&:focus': {
            outline: '3px solid #000000',
            outlineOffset: '3px',
          },
          '&:focus-visible': {
            outline: '3px solid #000000',
            outlineOffset: '3px',
          },
        },
      },
    },
    MuiTextField: {
      styleOverrides: {
        root: {
          '& .MuiOutlinedInput-root': {
            border: '2px solid #000000',
            '&:focus-within': {
              '& .MuiOutlinedInput-notchedOutline': {
                borderWidth: '3px',
                borderColor: '#000000',
              },
            },
          },
        },
      },
    },
    MuiIconButton: {
      styleOverrides: {
        root: {
          border: '2px solid #000000',
          '&:focus': {
            outline: '3px solid #000000',
            outlineOffset: '3px',
          },
          '&:focus-visible': {
            outline: '3px solid #000000',
            outlineOffset: '3px',
          },
        },
      },
    },
  },
});

// Export both themes
export { baseTheme as theme, highContrastTheme };
export default baseTheme;
