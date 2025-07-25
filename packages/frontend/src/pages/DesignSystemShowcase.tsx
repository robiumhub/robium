import React, { useState } from 'react';
import { Box, Typography, Paper, Divider, Chip, Avatar } from '@mui/material';
import {
  Palette as PaletteIcon,
  Typography as TypographyIcon,
  SpaceBar as SpaceIcon,
  BorderRadius as RadiusIcon,
  Filter1 as NumberIcon,
  Visibility as VisibilityIcon,
} from '@mui/icons-material';

// Import design tokens
import {
  colors,
  typography,
  spacing,
  borderRadius,
  shadows,
  zIndex,
  breakpoints,
  transitions,
} from '../design/designTokens';

// Import UI components
import {
  PrimaryButton,
  SecondaryButton,
  GhostButton,
  TextButton,
  SuccessButton,
  WarningButton,
  ErrorButton,
  AddButton,
  DeleteButton,
  EditButton,
  ViewButton,
} from '../components/ui/Button';

import {
  Input,
  PasswordInput,
  SearchInput,
  EmailInput,
  UsernameInput,
} from '../components/ui/Input';

import { InfoCard, StatsCard, ActionCard } from '../components/ui/Card';

import { AlertModal, ConfirmModal, FormModal } from '../components/ui/Modal';

const DesignSystemShowcase: React.FC = () => {
  const [showModal, setShowModal] = useState(false);
  const [modalType, setModalType] = useState<'alert' | 'confirm' | 'form'>(
    'alert'
  );

  const handleOpenModal = (type: 'alert' | 'confirm' | 'form') => {
    setModalType(type);
    setShowModal(true);
  };

  const handleCloseModal = () => {
    setShowModal(false);
  };

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
          }}
        >
          {Object.entries(colors.primary).map(([key, color]) => (
            <Box
              key={key}
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
              <Typography
                variant="caption"
                sx={{
                  color: 'white',
                  textShadow: '1px 1px 2px rgba(0,0,0,0.5)',
                }}
              >
                {key}
              </Typography>
            </Box>
          ))}
        </Box>

        <Typography variant="h6" gutterBottom sx={{ mt: 3 }}>
          Secondary Colors
        </Typography>
        <Box
          sx={{
            display: 'grid',
            gridTemplateColumns: 'repeat(auto-fit, minmax(80px, 1fr))',
            gap: 2,
          }}
        >
          {Object.entries(colors.secondary).map(([key, color]) => (
            <Box
              key={key}
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
              <Typography
                variant="caption"
                sx={{
                  color: 'white',
                  textShadow: '1px 1px 2px rgba(0,0,0,0.5)',
                }}
              >
                {key}
              </Typography>
            </Box>
          ))}
        </Box>

        <Typography variant="h6" gutterBottom sx={{ mt: 3 }}>
          Semantic Colors
        </Typography>
        <Box
          sx={{
            display: 'grid',
            gridTemplateColumns: 'repeat(auto-fit, minmax(80px, 1fr))',
            gap: 2,
          }}
        >
          {Object.entries(colors.semantic).map(([key, color]) => (
            <Box
              key={key}
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
              <Typography
                variant="caption"
                sx={{
                  color: 'white',
                  textShadow: '1px 1px 2px rgba(0,0,0,0.5)',
                }}
              >
                {key}
              </Typography>
            </Box>
          ))}
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

        <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
          <Typography variant="h1">
            Heading 1 - {typography.h1.fontSize}
          </Typography>
          <Typography variant="h2">
            Heading 2 - {typography.h2.fontSize}
          </Typography>
          <Typography variant="h3">
            Heading 3 - {typography.h3.fontSize}
          </Typography>
          <Typography variant="h4">
            Heading 4 - {typography.h4.fontSize}
          </Typography>
          <Typography variant="h5">
            Heading 5 - {typography.h5.fontSize}
          </Typography>
          <Typography variant="h6">
            Heading 6 - {typography.h6.fontSize}
          </Typography>
          <Typography variant="body1">
            Body 1 - {typography.body1.fontSize}
          </Typography>
          <Typography variant="body2">
            Body 2 - {typography.body2.fontSize}
          </Typography>
          <Typography variant="caption">
            Caption - {typography.caption.fontSize}
          </Typography>
          <Typography variant="overline">
            Overline - {typography.overline.fontSize}
          </Typography>
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
          {Object.entries(spacing).map(([key, value]) => (
            <Box key={key} display="flex" alignItems="center" gap={2}>
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
      </Paper>

      {/* Border Radius */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Box display="flex" alignItems="center" gap={1} mb={2}>
          <RadiusIcon color="primary" />
          <Typography variant="h5" component="h2">
            Border Radius
          </Typography>
        </Box>

        <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 2 }}>
          {Object.entries(borderRadius).map(([key, value]) => (
            <Box
              key={key}
              sx={{
                width: 80,
                height: 80,
                backgroundColor: 'primary.main',
                borderRadius: value,
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
              }}
            >
              <Typography variant="caption" sx={{ color: 'white' }}>
                {key}
              </Typography>
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
          {Object.entries(shadows).map(([key, value]) => (
            <Box
              key={key}
              sx={{
                p: 2,
                backgroundColor: 'background.paper',
                boxShadow: value,
                borderRadius: 1,
              }}
            >
              <Typography variant="body2">{key}</Typography>
            </Box>
          ))}
        </Box>
      </Paper>

      {/* Z-Index */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Box display="flex" alignItems="center" gap={1} mb={2}>
          <NumberIcon color="primary" />
          <Typography variant="h5" component="h2">
            Z-Index Scale
          </Typography>
        </Box>

        <Box sx={{ display: 'flex', flexDirection: 'column', gap: 1 }}>
          {Object.entries(zIndex).map(([key, value]) => (
            <Box key={key} display="flex" justifyContent="space-between">
              <Typography variant="body2">{key}</Typography>
              <Typography variant="body2" color="text.secondary">
                {value}
              </Typography>
            </Box>
          ))}
        </Box>
      </Paper>

      {/* Buttons */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Typography variant="h5" component="h2" gutterBottom>
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

        <Typography variant="h6" gutterBottom>
          Action Buttons
        </Typography>
        <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 2 }}>
          <AddButton>Add Item</AddButton>
          <EditButton>Edit</EditButton>
          <DeleteButton>Delete</DeleteButton>
          <ViewButton>View</ViewButton>
        </Box>
      </Paper>

      {/* Inputs */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Typography variant="h5" component="h2" gutterBottom>
          Input Fields
        </Typography>

        <Box
          sx={{
            display: 'flex',
            flexDirection: 'column',
            gap: 3,
            maxWidth: 400,
          }}
        >
          <Input label="Standard Input" placeholder="Enter text..." />
          <EmailInput label="Email Input" placeholder="Enter email..." />
          <UsernameInput
            label="Username Input"
            placeholder="Enter username..."
          />
          <PasswordInput
            label="Password Input"
            placeholder="Enter password..."
          />
          <SearchInput label="Search Input" placeholder="Search..." />
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
            gridTemplateColumns: { xs: '1fr', md: 'repeat(3, 1fr)' },
            gap: 3,
          }}
        >
          <InfoCard
            title="Information Card"
            content="This is an information card with some sample content."
            icon={<TypographyIcon />}
          />
          <StatsCard
            title="Statistics"
            value="1,234"
            subtitle="Total Items"
            trend="+12%"
            trendDirection="up"
          />
          <ActionCard
            title="Action Card"
            content="This card has action buttons."
            actions={[
              { label: 'View', onClick: () => console.log('View clicked') },
              { label: 'Edit', onClick: () => console.log('Edit clicked') },
            ]}
          />
        </Box>
      </Paper>

      {/* Modals */}
      <Paper sx={{ p: 3, mb: 4 }}>
        <Typography variant="h5" component="h2" gutterBottom>
          Modals
        </Typography>

        <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 2 }}>
          <PrimaryButton onClick={() => handleOpenModal('alert')}>
            Alert Modal
          </PrimaryButton>
          <PrimaryButton onClick={() => handleOpenModal('confirm')}>
            Confirm Modal
          </PrimaryButton>
          <PrimaryButton onClick={() => handleOpenModal('form')}>
            Form Modal
          </PrimaryButton>
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
          <Chip label="Avatar" avatar={<Avatar>U</Avatar>} />
          <Chip label="Clickable" onClick={() => console.log('Chip clicked')} />
          <Chip
            label="Deletable"
            onDelete={() => console.log('Chip deleted')}
          />
        </Box>
      </Paper>

      {/* Modal Components */}
      <AlertModal
        open={showModal && modalType === 'alert'}
        title="Alert Modal"
        message="This is an alert modal with important information."
        onClose={handleCloseModal}
      />

      <ConfirmModal
        open={showModal && modalType === 'confirm'}
        title="Confirm Action"
        message="Are you sure you want to perform this action?"
        onConfirm={() => {
          console.log('Action confirmed');
          handleCloseModal();
        }}
        onCancel={handleCloseModal}
      />

      <FormModal
        open={showModal && modalType === 'form'}
        title="Form Modal"
        onClose={handleCloseModal}
        onSubmit={(data) => {
          console.log('Form submitted:', data);
          handleCloseModal();
        }}
      >
        <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
          <Input label="Name" placeholder="Enter your name" />
          <EmailInput label="Email" placeholder="Enter your email" />
          <Input
            label="Message"
            placeholder="Enter your message"
            multiline
            rows={3}
          />
        </Box>
      </FormModal>
    </Box>
  );
};

export default DesignSystemShowcase;
