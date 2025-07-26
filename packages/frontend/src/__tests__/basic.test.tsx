import React from 'react';
import { render, screen } from '@testing-library/react';
import { ThemeProvider } from '@mui/material/styles';
import theme from '../design/theme';

// Simple test component
const TestComponent: React.FC = () => {
  return (
    <div>
      <h1>Test Component</h1>
      <p>This is a test</p>
    </div>
  );
};

describe('Basic UI Tests', () => {
  it('should render a simple component', () => {
    render(
      <ThemeProvider theme={theme}>
        <TestComponent />
      </ThemeProvider>
    );

    expect(screen.getByText('Test Component')).toBeInTheDocument();
    expect(screen.getByText('This is a test')).toBeInTheDocument();
  });

  it('should render Material-UI components', () => {
    const { Box, Typography } = require('@mui/material');

    render(
      <ThemeProvider theme={theme}>
        <Box>
          <Typography variant="h1">Material-UI Test</Typography>
          <Typography variant="body1">
            This is a Material-UI component test
          </Typography>
        </Box>
      </ThemeProvider>
    );

    expect(screen.getByText('Material-UI Test')).toBeInTheDocument();
    expect(
      screen.getByText('This is a Material-UI component test')
    ).toBeInTheDocument();
  });

  it('should handle basic user interactions', () => {
    const { Button } = require('@mui/material');

    const handleClick = jest.fn();

    render(
      <ThemeProvider theme={theme}>
        <Button onClick={handleClick} data-testid="test-button">
          Click Me
        </Button>
      </ThemeProvider>
    );

    const button = screen.getByTestId('test-button');
    expect(button).toBeInTheDocument();
    expect(button).toHaveTextContent('Click Me');
  });
});
