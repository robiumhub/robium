import React from 'react';
import {
  Box,
  Typography,
  Paper,
  Button,
  TextField,
  Chip,
  Card,
  CardContent,
  Divider,
} from '@mui/material';

import {
  Palette as PaletteIcon,
  TextFields as TypographyIcon,
  SpaceBar as SpaceIcon,
  CropSquare as RadiusIcon,
  Filter1 as NumberIcon,
  Visibility as VisibilityIcon,
} from '@mui/icons-material';

const DesignSystemShowcase: React.FC = () => {
  return (
    <Box sx={{ p: 4, maxWidth: 1200, mx: 'auto' }}>
      <Typography variant="h3" component="h1" gutterBottom>
        Design System Showcase
      </Typography>
      <Typography variant="body1" color="text.secondary" sx={{ mb: 4 }}>
        A comprehensive guide to our design tokens and reusable UI components.
      </Typography>

      {/* Color Palette */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Box display="flex" alignItems="center" gap={1} mb={2}>
          <PaletteIcon color="primary" />
          <Typography variant="h5" component="h2">
            Color Palette
          </Typography>
        </Box>

        <Typography variant="h6" gutterBottom>
          Primary Colors
        </Typography>
        <Box
          sx={{
            display: 'grid',
            gridTemplateColumns: 'repeat(auto-fit, minmax(80px, 1fr))',
            gap: 2,
            mb: 3,
          }}
        >
          {['#2196f3', '#1976d2', '#1565c0', '#0d47a1'].map((color, index) => (
            <Box
              key={index}
              sx={{
                width: 80,
                height: 80,
                backgroundColor: color,
                borderRadius: 1,
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                border: '1px solid',
                borderColor: 'divider',
              }}
            >
              <Typography variant="caption" sx={{ color: 'white' }}>
                {color}
              </Typography>
            </Box>
          ))}
        </Box>

        <Typography variant="h6" gutterBottom>
          Semantic Colors
        </Typography>
        <Box sx={{ display: 'flex', gap: 2, flexWrap: 'wrap' }}>
          <Chip label="Success" color="success" />
          <Chip label="Warning" color="warning" />
          <Chip label="Error" color="error" />
          <Chip label="Info" color="info" />
        </Box>
      </Paper>

      {/* Typography */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Box display="flex" alignItems="center" gap={1} mb={2}>
          <TypographyIcon color="primary" />
          <Typography variant="h5" component="h2">
            Typography
          </Typography>
        </Box>

        <Box sx={{ display: 'flex', flexDirection: 'column', gap: 1 }}>
          <Typography variant="h1">Heading 1</Typography>
          <Typography variant="h2">Heading 2</Typography>
          <Typography variant="h3">Heading 3</Typography>
          <Typography variant="h4">Heading 4</Typography>
          <Typography variant="h5">Heading 5</Typography>
          <Typography variant="h6">Heading 6</Typography>
          <Typography variant="body1">Body 1 - Regular text content</Typography>
          <Typography variant="body2">Body 2 - Smaller text content</Typography>
        </Box>
      </Paper>

      {/* Buttons */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Typography variant="h5" component="h2" gutterBottom>
          Buttons
        </Typography>

        <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 2, mb: 3 }}>
          <Button variant="contained">Primary</Button>
          <Button variant="outlined">Secondary</Button>
          <Button variant="text">Text</Button>
        </Box>

        <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 2, mb: 3 }}>
          <Button variant="contained" color="success">
            Success
          </Button>
          <Button variant="contained" color="warning">
            Warning
          </Button>
          <Button variant="contained" color="error">
            Error
          </Button>
          <Button variant="contained" color="info">
            Info
          </Button>
        </Box>

        <Divider sx={{ my: 2 }} />

        <Typography variant="h6" gutterBottom>
          Button Sizes
        </Typography>
        <Box
          sx={{
            display: 'flex',
            flexWrap: 'wrap',
            gap: 2,
            alignItems: 'center',
          }}
        >
          <Button variant="contained" size="small">
            Small
          </Button>
          <Button variant="contained" size="medium">
            Medium
          </Button>
          <Button variant="contained" size="large">
            Large
          </Button>
        </Box>
      </Paper>

      {/* Inputs */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Typography variant="h5" component="h2" gutterBottom>
          Inputs
        </Typography>

        <Box
          sx={{
            display: 'flex',
            flexDirection: 'column',
            gap: 2,
            maxWidth: 300,
          }}
        >
          <TextField label="Standard Input" placeholder="Enter text..." />
          <TextField
            label="Email Input"
            type="email"
            placeholder="Enter email..."
          />
          <TextField
            label="Password Input"
            type="password"
            placeholder="Enter password..."
          />
          <TextField label="Search Input" placeholder="Search..." />
        </Box>
      </Paper>

      {/* Cards */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Typography variant="h5" component="h2" gutterBottom>
          Cards
        </Typography>

        <Box
          sx={{
            display: 'grid',
            gridTemplateColumns: 'repeat(auto-fit, minmax(250px, 1fr))',
            gap: 3,
          }}
        >
          <Card>
            <CardContent>
              <Typography variant="h6" gutterBottom>
                Information Card
              </Typography>
              <Typography variant="body2" color="text.secondary">
                This is an information card with some content.
              </Typography>
            </CardContent>
          </Card>
          <Card>
            <CardContent>
              <Typography variant="h6" gutterBottom>
                Statistics Card
              </Typography>
              <Typography variant="h4" color="primary">
                1,234
              </Typography>
              <Typography variant="body2" color="text.secondary">
                Total Items
              </Typography>
            </CardContent>
          </Card>
          <Card>
            <CardContent>
              <Typography variant="h6" gutterBottom>
                Action Card
              </Typography>
              <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
                This card has an action button.
              </Typography>
              <Button variant="contained" size="small">
                Take Action
              </Button>
            </CardContent>
          </Card>
        </Box>
      </Paper>

      {/* Spacing */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Box display="flex" alignItems="center" gap={1} mb={2}>
          <SpaceIcon color="primary" />
          <Typography variant="h5" component="h2">
            Spacing Scale
          </Typography>
        </Box>

        <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
          {[8, 16, 24, 32, 48].map((spacing) => (
            <Box key={spacing} display="flex" alignItems="center" gap={2}>
              <Box
                sx={{
                  width: spacing,
                  height: 20,
                  backgroundColor: 'primary.main',
                  borderRadius: 1,
                }}
              />
              <Typography variant="body2">{spacing}px</Typography>
            </Box>
          ))}
        </Box>
      </Paper>

      {/* Shadows */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Box display="flex" alignItems="center" gap={1} mb={2}>
          <VisibilityIcon color="primary" />
          <Typography variant="h5" component="h2">
            Shadows
          </Typography>
        </Box>

        <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
          {[1, 2, 3, 4, 5].map((level) => (
            <Box
              key={level}
              sx={{
                p: 2,
                backgroundColor: 'background.paper',
                boxShadow: level,
                borderRadius: 1,
              }}
            >
              <Typography variant="body2">Shadow Level {level}</Typography>
            </Box>
          ))}
        </Box>
      </Paper>

      {/* Chips */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Typography variant="h5" component="h2" gutterBottom>
          Chips
        </Typography>

        <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
          <Chip label="Default" />
          <Chip label="Primary" color="primary" />
          <Chip label="Secondary" color="secondary" />
          <Chip label="Success" color="success" />
          <Chip label="Warning" color="warning" />
          <Chip label="Error" color="error" />
          <Chip label="Info" color="info" />
        </Box>
      </Paper>
    </Box>
  );
};

export default DesignSystemShowcase;
