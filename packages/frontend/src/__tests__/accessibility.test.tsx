import React from 'react';
import { render, screen } from '@testing-library/react';
import { axe, toHaveNoViolations } from 'jest-axe';
import { ThemeProvider } from '@mui/material/styles';
import theme from '../design/theme';
import AccessibilityProvider from '../components/AccessibilityProvider';
import AccessibleForm from '../components/AccessibleForm';
import Layout from '../components/Layout';
import Login from '../pages/Login';
import Register from '../pages/Register';
import Dashboard from '../pages/Dashboard';

// Extend Jest matchers
expect.extend(toHaveNoViolations);

// Test wrapper with providers
const TestWrapper: React.FC<{ children: React.ReactNode }> = ({ children }) => (
  <ThemeProvider theme={theme}>
    <AccessibilityProvider>{children}</AccessibilityProvider>
  </ThemeProvider>
);

describe('Accessibility Tests', () => {
  describe('AccessibilityProvider', () => {
    it('should not have accessibility violations', async () => {
      const { container } = render(
        <TestWrapper>
          <div>Test content</div>
        </TestWrapper>
      );

      const results = await axe(container);
      expect(results).toHaveNoViolations();
    });

    it('should render skip navigation links', () => {
      render(
        <TestWrapper>
          <div>Test content</div>
        </TestWrapper>
      );

      expect(screen.getByText('Skip to Navigation')).toBeInTheDocument();
      expect(screen.getByText('Skip to Content')).toBeInTheDocument();
    });

    it('should render high contrast toggle button', () => {
      render(
        <TestWrapper>
          <div>Test content</div>
        </TestWrapper>
      );

      expect(screen.getByLabelText(/high contrast mode/i)).toBeInTheDocument();
    });
  });

  describe('AccessibleForm', () => {
    const mockFields = [
      {
        name: 'email',
        label: 'Email Address',
        type: 'email' as const,
        required: true,
        validation: {
          pattern: /^[^\s@]+@[^\s@]+\.[^\s@]+$/,
        },
      },
      {
        name: 'password',
        label: 'Password',
        type: 'password' as const,
        required: true,
        validation: {
          minLength: 8,
        },
      },
      {
        name: 'role',
        label: 'Role',
        type: 'select' as const,
        options: [
          { value: 'user', label: 'User' },
          { value: 'admin', label: 'Administrator' },
        ],
      },
    ];

    it('should not have accessibility violations', async () => {
      const { container } = render(
        <TestWrapper>
          <AccessibleForm
            fields={mockFields}
            onSubmit={jest.fn()}
            title="Test Form"
            description="This is a test form"
          />
        </TestWrapper>
      );

      const results = await axe(container);
      expect(results).toHaveNoViolations();
    });

    it('should have proper form labels and descriptions', () => {
      render(
        <TestWrapper>
          <AccessibleForm
            fields={mockFields}
            onSubmit={jest.fn()}
            title="Test Form"
            description="This is a test form"
          />
        </TestWrapper>
      );

      expect(screen.getByText('Test Form')).toBeInTheDocument();
      expect(screen.getByText('This is a test form')).toBeInTheDocument();
      expect(screen.getByLabelText('Email Address')).toBeInTheDocument();
      expect(screen.getByLabelText('Password')).toBeInTheDocument();
      expect(screen.getByLabelText('Role')).toBeInTheDocument();
    });

    it('should announce validation errors to screen readers', async () => {
      const mockAnnounce = jest.fn();
      jest
        .spyOn(
          require('../components/AccessibilityProvider'),
          'useAccessibility'
        )
        .mockReturnValue({
          announceToScreenReader: mockAnnounce,
        });

      render(
        <TestWrapper>
          <AccessibleForm fields={mockFields} onSubmit={jest.fn()} />
        </TestWrapper>
      );

      const submitButton = screen.getByText('Submit');
      submitButton.click();

      // Wait for validation to complete
      await screen.findByText(/Email Address is required/);

      expect(mockAnnounce).toHaveBeenCalledWith(
        expect.stringContaining('Form has'),
        'assertive'
      );
    });
  });

  describe('Layout Component', () => {
    it('should not have accessibility violations', async () => {
      const { container } = render(
        <TestWrapper>
          <Layout>
            <div>Test content</div>
          </Layout>
        </TestWrapper>
      );

      const results = await axe(container);
      expect(results).toHaveNoViolations();
    });

    it('should have proper navigation landmarks', () => {
      render(
        <TestWrapper>
          <Layout>
            <div>Test content</div>
          </Layout>
        </TestWrapper>
      );

      expect(screen.getByRole('navigation')).toBeInTheDocument();
      expect(screen.getByRole('main')).toBeInTheDocument();
    });

    it('should have proper navigation labels', () => {
      render(
        <TestWrapper>
          <Layout>
            <div>Test content</div>
          </Layout>
        </TestWrapper>
      );

      expect(screen.getByLabelText('Main navigation')).toBeInTheDocument();
      expect(screen.getByLabelText('Main content')).toBeInTheDocument();
    });
  });

  describe('Login Page', () => {
    it('should not have accessibility violations', async () => {
      const { container } = render(
        <TestWrapper>
          <Login />
        </TestWrapper>
      );

      const results = await axe(container);
      expect(results).toHaveNoViolations();
    });

    it('should have proper form structure', () => {
      render(
        <TestWrapper>
          <Login />
        </TestWrapper>
      );

      expect(screen.getByRole('heading', { level: 1 })).toBeInTheDocument();
      expect(screen.getByLabelText('Email Address')).toBeInTheDocument();
      expect(screen.getByLabelText('Password')).toBeInTheDocument();
      expect(
        screen.getByRole('button', { name: /sign in/i })
      ).toBeInTheDocument();
    });

    it('should have proper focus management', () => {
      render(
        <TestWrapper>
          <Login />
        </TestWrapper>
      );

      const emailInput = screen.getByLabelText('Email Address');
      expect(emailInput).toHaveAttribute('autoFocus');
    });
  });

  describe('Register Page', () => {
    it('should not have accessibility violations', async () => {
      const { container } = render(
        <TestWrapper>
          <Register />
        </TestWrapper>
      );

      const results = await axe(container);
      expect(results).toHaveNoViolations();
    });

    it('should have proper form structure', () => {
      render(
        <TestWrapper>
          <Register />
        </TestWrapper>
      );

      expect(screen.getByRole('heading', { level: 1 })).toBeInTheDocument();
      expect(screen.getByLabelText('Email Address')).toBeInTheDocument();
      expect(screen.getByLabelText('Password')).toBeInTheDocument();
      expect(screen.getByLabelText('Confirm Password')).toBeInTheDocument();
      expect(
        screen.getByRole('button', { name: /sign up/i })
      ).toBeInTheDocument();
    });
  });

  describe('Dashboard Page', () => {
    it('should not have accessibility violations', async () => {
      const { container } = render(
        <TestWrapper>
          <Layout>
            <Dashboard />
          </Layout>
        </TestWrapper>
      );

      const results = await axe(container);
      expect(results).toHaveNoViolations();
    });

    it('should have proper heading hierarchy', () => {
      render(
        <TestWrapper>
          <Layout>
            <Dashboard />
          </Layout>
        </TestWrapper>
      );

      const headings = screen.getAllByRole('heading');
      expect(headings.length).toBeGreaterThan(0);

      // Check that headings follow proper hierarchy
      const headingLevels = headings.map((h) => parseInt(h.tagName.charAt(1)));
      const hasProperHierarchy = headingLevels.every(
        (level, index) => index === 0 || level <= headingLevels[index - 1] + 1
      );

      expect(hasProperHierarchy).toBe(true);
    });
  });

  describe('Color Contrast', () => {
    it('should have sufficient color contrast for text', () => {
      const {
        calculateContrastRatio,
        checkWCAGContrast,
      } = require('../utils/accessibility');

      // Test primary text on background
      const contrastRatio = calculateContrastRatio('#212121', '#fafafa');
      const result = checkWCAGContrast(contrastRatio);

      expect(result.passes).toBe(true);
      expect(result.level).toBe('AA');
    });

    it('should have sufficient color contrast for large text', () => {
      const {
        calculateContrastRatio,
        checkWCAGContrast,
      } = require('../utils/accessibility');

      // Test primary text on background for large text
      const contrastRatio = calculateContrastRatio('#212121', '#fafafa');
      const result = checkWCAGContrast(contrastRatio, true);

      expect(result.passes).toBe(true);
      expect(result.level).toBe('AAA');
    });
  });

  describe('Keyboard Navigation', () => {
    it('should support keyboard navigation', () => {
      render(
        <TestWrapper>
          <Layout>
            <Dashboard />
          </Layout>
        </TestWrapper>
      );

      // Check that interactive elements are focusable
      const buttons = screen.getAllByRole('button');
      buttons.forEach((button) => {
        expect(button).toHaveAttribute('tabIndex', '0');
      });
    });

    it('should have visible focus indicators', () => {
      render(
        <TestWrapper>
          <Layout>
            <Dashboard />
          </Layout>
        </TestWrapper>
      );

      const buttons = screen.getAllByRole('button');
      buttons.forEach((button) => {
        // Check that buttons have focus styles defined
        const computedStyle = window.getComputedStyle(button);
        expect(computedStyle.outline).not.toBe('none');
      });
    });
  });

  describe('Screen Reader Support', () => {
    it('should have proper ARIA labels', () => {
      render(
        <TestWrapper>
          <Layout>
            <Dashboard />
          </Layout>
        </TestWrapper>
      );

      // Check for ARIA labels on interactive elements
      const buttons = screen.getAllByRole('button');
      buttons.forEach((button) => {
        const hasLabel =
          button.hasAttribute('aria-label') ||
          button.hasAttribute('aria-labelledby') ||
          button.textContent?.trim();
        expect(hasLabel).toBe(true);
      });
    });

    it('should have proper alt text for images', () => {
      render(
        <TestWrapper>
          <Layout>
            <Dashboard />
          </Layout>
        </TestWrapper>
      );

      const images = screen.getAllByRole('img');
      images.forEach((img) => {
        expect(img).toHaveAttribute('alt');
      });
    });
  });
});
