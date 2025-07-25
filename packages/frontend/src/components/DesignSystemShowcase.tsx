import React, { useState } from 'react';
import {
  Box,
  Typography,
  Grid,
  Paper,
  Divider,
  Chip,
  Avatar,
} from '@mui/material';
import {
  Dashboard as DashboardIcon,
  Settings as SettingsIcon,
  Person as PersonIcon,
  Add as AddIcon,
  Edit as EditIcon,
  Delete as DeleteIcon,
  Save as SaveIcon,
  Cancel as CancelIcon,
  Refresh as RefreshIcon,
} from '@mui/icons-material';

// Import design system components
import {
  Button,
  PrimaryButton,
  SecondaryButton,
  GhostButton,
  TextButton,
  SuccessButton,
  WarningButton,
  ErrorButton,
  AddButton,
  EditButton,
  DeleteButton,
  SaveButton,
  CancelButton,
  RefreshButton,
  Input,
  PasswordInput,
  SearchInput,
  EmailInput,
  UsernameInput,
  Card,
  InfoCard,
  StatsCard,
  ActionCard,
  Modal,
  AlertModal,
  ConfirmModal,
  FormModal,
} from './ui';

// Import design tokens
import {
  colors,
  typography,
  spacing,
  borderRadius,
  shadows,
} from '../design/designTokens';

const DesignSystemShowcase: React.FC = () => {
  const [modalOpen, setModalOpen] = useState(false);
  const [alertOpen, setAlertOpen] = useState(false);
  const [confirmOpen, setConfirmOpen] = useState(false);
  const [formModalOpen, setFormModalOpen] = useState(false);

  return (
    <Box sx={{ p: 4, maxWidth: 1200, mx: 'auto' }}>
      <Typography variant="h3" gutterBottom>
        Robium Design System
      </Typography>
      <Typography variant="body1" color="text.secondary" sx={{ mb: 4 }}>
        A comprehensive design system with reusable components and consistent
        styling.
      </Typography>

      {/* Color Palette */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Typography variant="h4" gutterBottom>
          Color Palette
        </Typography>
        <Box
          sx={{
            display: 'grid',
            gridTemplateColumns: 'repeat(auto-fit, minmax(80px, 1fr))',
            gap: 2,
          }}
        >
          {Object.entries(colors.primary).map(([key, color]) => (
            <Box
              key={key}
              sx={{
                width: 80,
                height: 60,
                backgroundColor: color,
                borderRadius: 1,
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                color: 'white',
                fontSize: '0.75rem',
                fontWeight: 'bold',
              }}
            >
              {key}
            </Box>
          ))}
        </Box>
      </Paper>

      {/* Typography */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Typography variant="h4" gutterBottom>
          Typography
        </Typography>
        <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
          <Typography variant="h1">Heading 1 - Large Title</Typography>
          <Typography variant="h2">Heading 2 - Section Title</Typography>
          <Typography variant="h3">Heading 3 - Subsection Title</Typography>
          <Typography variant="h4">Heading 4 - Card Title</Typography>
          <Typography variant="h5">Heading 5 - Small Title</Typography>
          <Typography variant="h6">Heading 6 - Tiny Title</Typography>
          <Typography variant="body1">Body 1 - Main text content</Typography>
          <Typography variant="body2">
            Body 2 - Secondary text content
          </Typography>
          <Typography variant="caption">Caption - Small helper text</Typography>
        </Box>
      </Paper>

      {/* Buttons */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Typography variant="h4" gutterBottom>
          Buttons
        </Typography>
        <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 2 }}>
          <PrimaryButton>Primary</PrimaryButton>
          <SecondaryButton>Secondary</SecondaryButton>
          <GhostButton>Ghost</GhostButton>
          <TextButton>Text</TextButton>
          <SuccessButton>Success</SuccessButton>
          <WarningButton>Warning</WarningButton>
          <ErrorButton>Error</ErrorButton>
        </Box>

        <Divider sx={{ my: 3 }} />

        <Typography variant="h5" gutterBottom>
          Action Buttons
        </Typography>
        <Grid container spacing={2}>
          <Grid item>
            <AddButton>Add Item</AddButton>
          </Grid>
          <Grid item>
            <EditButton>Edit</EditButton>
          </Grid>
          <Grid item>
            <DeleteButton>Delete</DeleteButton>
          </Grid>
          <Grid item>
            <SaveButton>Save</SaveButton>
          </Grid>
          <Grid item>
            <CancelButton>Cancel</CancelButton>
          </Grid>
          <Grid item>
            <RefreshButton>Refresh</RefreshButton>
          </Grid>
        </Grid>
      </Paper>

      {/* Inputs */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Typography variant="h4" gutterBottom>
          Inputs
        </Typography>
        <Grid container spacing={3}>
          <Grid item xs={12} md={6}>
            <EmailInput label="Email" fullWidth />
          </Grid>
          <Grid item xs={12} md={6}>
            <UsernameInput label="Username" fullWidth />
          </Grid>
          <Grid item xs={12} md={6}>
            <PasswordInput label="Password" fullWidth />
          </Grid>
          <Grid item xs={12} md={6}>
            <SearchInput label="Search" fullWidth />
          </Grid>
        </Grid>
      </Paper>

      {/* Cards */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Typography variant="h4" gutterBottom>
          Cards
        </Typography>
        <Grid container spacing={3}>
          <Grid item xs={12} md={4}>
            <Card
              title="Basic Card"
              subtitle="Card subtitle"
              description="This is a basic card with title, subtitle, and description."
              chips={['Tag 1', 'Tag 2']}
            />
          </Grid>
          <Grid item xs={12} md={4}>
            <InfoCard
              title="Info Card"
              icon={<DashboardIcon color="primary" />}
            >
              <Typography variant="body2">
                This is an info card with an icon and special styling.
              </Typography>
            </InfoCard>
          </Grid>
          <Grid item xs={12} md={4}>
            <StatsCard
              title="Active Projects"
              value="12"
              trend="up"
              trendValue="+3 this week"
            />
          </Grid>
        </Grid>
      </Paper>

      {/* Modals */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Typography variant="h4" gutterBottom>
          Modals
        </Typography>
        <Grid container spacing={2}>
          <Grid item>
            <PrimaryButton onClick={() => setModalOpen(true)}>
              Open Modal
            </PrimaryButton>
          </Grid>
          <Grid item>
            <WarningButton onClick={() => setAlertOpen(true)}>
              Show Alert
            </WarningButton>
          </Grid>
          <Grid item>
            <ErrorButton onClick={() => setConfirmOpen(true)}>
              Show Confirm
            </ErrorButton>
          </Grid>
          <Grid item>
            <SuccessButton onClick={() => setFormModalOpen(true)}>
              Show Form Modal
            </SuccessButton>
          </Grid>
        </Grid>
      </Paper>

      {/* Design Tokens */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Typography variant="h4" gutterBottom>
          Design Tokens
        </Typography>
        <Grid container spacing={3}>
          <Grid item xs={12} md={6}>
            <Typography variant="h6" gutterBottom>
              Spacing
            </Typography>
            <Box sx={{ display: 'flex', flexDirection: 'column', gap: 1 }}>
              {Object.entries(spacing)
                .slice(0, 8)
                .map(([key, value]) => (
                  <Box
                    key={key}
                    sx={{ display: 'flex', alignItems: 'center', gap: 2 }}
                  >
                    <Box
                      sx={{
                        width: value,
                        height: 20,
                        backgroundColor: 'primary.main',
                        borderRadius: 1,
                      }}
                    />
                    <Typography variant="body2">
                      {key}: {value}
                    </Typography>
                  </Box>
                ))}
            </Box>
          </Grid>
          <Grid item xs={12} md={6}>
            <Typography variant="h6" gutterBottom>
              Border Radius
            </Typography>
            <Box sx={{ display: 'flex', flexDirection: 'column', gap: 1 }}>
              {Object.entries(borderRadius)
                .slice(0, 6)
                .map(([key, value]) => (
                  <Box
                    key={key}
                    sx={{ display: 'flex', alignItems: 'center', gap: 2 }}
                  >
                    <Box
                      sx={{
                        width: 60,
                        height: 20,
                        backgroundColor: 'secondary.main',
                        borderRadius: value,
                      }}
                    />
                    <Typography variant="body2">
                      {key}: {value}
                    </Typography>
                  </Box>
                ))}
            </Box>
          </Grid>
        </Grid>
      </Paper>

      {/* Modals */}
      <Modal
        open={modalOpen}
        onClose={() => setModalOpen(false)}
        title="Example Modal"
        description="This is an example modal with custom content."
      >
        <Typography variant="body1">
          You can put any content here. This modal demonstrates the design
          system's modal component.
        </Typography>
      </Modal>

      <AlertModal
        open={alertOpen}
        onClose={() => setAlertOpen(false)}
        title="Alert"
        description="This is an alert modal with a warning icon."
        severity="warning"
        onConfirm={() => setAlertOpen(false)}
      />

      <ConfirmModal
        open={confirmOpen}
        onClose={() => setConfirmOpen(false)}
        title="Confirm Action"
        description="Are you sure you want to perform this action?"
        severity="error"
        onConfirm={() => setConfirmOpen(false)}
        onCancel={() => setConfirmOpen(false)}
        destructive
      />

      <FormModal
        open={formModalOpen}
        onClose={() => setFormModalOpen(false)}
        title="Form Modal"
        description="This is a form modal with input fields."
        onSave={() => setFormModalOpen(false)}
        onCancel={() => setFormModalOpen(false)}
      >
        <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
          <EmailInput label="Email" />
          <UsernameInput label="Username" />
        </Box>
      </FormModal>
    </Box>
  );
};

export default DesignSystemShowcase;
