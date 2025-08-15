import React, { useState, useMemo } from 'react';
import {
  Drawer,
  Box,
  Typography,
  Button,
  Chip,
  TextField,
  Divider,
  IconButton,
  useTheme,
  useMediaQuery,
} from '@mui/material';
import { Clear as ClearIcon, Close as CloseIcon } from '@mui/icons-material';
import {
  TemplateFilters,
  DEFAULT_TEMPLATE_FILTERS,
  USE_CASES,
  CAPABILITIES,
  ROBOT_TARGETS,
  SIMULATORS,
  ROS_DISTROS,
  RMW_IMPLEMENTATIONS,
  LICENSES,
  DIFFICULTY_LEVELS,
  TemplateStats,
} from '../types/template';

interface TemplateFiltersProps {
  open: boolean;
  onClose: () => void;
  filters: TemplateFilters;
  onFiltersChange: (filters: TemplateFilters) => void;
  stats?: TemplateStats;
  availableTags?: string[];
  permanent?: boolean;
  activeTab?: string; // ignored now; kept for backward compatibility
}

const TemplateFiltersComponent: React.FC<TemplateFiltersProps> = ({
  open,
  onClose,
  filters,
  onFiltersChange,
  stats,
  availableTags = [],
  permanent = false,
}) => {
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('md'));
  const [localFilters, setLocalFilters] = useState<TemplateFilters>(filters);
  const [newTag, setNewTag] = useState('');

  const handleArrayToggle = (key: keyof TemplateFilters, value: string) => {
    const current = (localFilters[key] as string[]) || [];
    const exists = current.includes(value);
    const updated = exists
      ? current.filter((v) => v !== value)
      : [...current, value];
    const next = { ...localFilters, [key]: updated } as TemplateFilters;
    setLocalFilters(next);
    onFiltersChange(next);
  };

  const handleBooleanToggle = (key: keyof TemplateFilters) => {
    const next = {
      ...localFilters,
      [key]: !Boolean(localFilters[key]),
    } as TemplateFilters;
    setLocalFilters(next);
    onFiltersChange(next);
  };

  const addTag = () => {
    const tag = newTag.trim();
    if (!tag) return;
    if (!localFilters.tags.includes(tag)) {
      const next = { ...localFilters, tags: [...localFilters.tags, tag] };
      setLocalFilters(next);
      onFiltersChange(next);
    }
    setNewTag('');
  };

  const Section: React.FC<{ title: string; children: React.ReactNode }> = ({
    title,
    children,
  }) => (
    <Box sx={{ mb: 2 }}>
      <Typography variant="subtitle2" sx={{ mb: 1, fontWeight: 600 }}>
        {title}
      </Typography>
      {children}
    </Box>
  );

  const renderChips = (
    items: readonly string[],
    key: keyof TemplateFilters,
    counts?: Record<string, number>
  ) => (
    <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
      {items.map((item) => {
        const selected =
          (localFilters[key] as string[])?.includes(item) || false;
        const count = counts?.[item] ?? undefined;
        return (
          <Chip
            key={item}
            label={count !== undefined ? `${item} (${count})` : item}
            clickable
            color={selected ? 'primary' : 'default'}
            variant={selected ? 'filled' : 'outlined'}
            onClick={() => handleArrayToggle(key, item)}
            sx={{ height: 28 }}
          />
        );
      })}
    </Box>
  );

  const filterContent = (
    <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
      {/* Tags Section - Moved to top */}
      <Section title="Tags">
        <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap', mb: 1 }}>
          {localFilters.tags.map((tag) => (
            <Chip
              key={tag}
              label={tag}
              onDelete={() => handleArrayToggle('tags', tag)}
              color="primary"
            />
          ))}
        </Box>
        <TextField
          size="small"
          placeholder="Add tag and press Enter"
          value={newTag}
          onChange={(e) => setNewTag(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === 'Enter') {
              e.preventDefault();
              addTag();
            }
          }}
          fullWidth
        />
      </Section>

      <Divider />

      {/* Attribute toggles */}
      <Section title="Attributes">
        <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
          <Chip
            label="Requires GPU"
            clickable
            color={localFilters.requires_gpu ? 'primary' : 'default'}
            variant={localFilters.requires_gpu ? 'filled' : 'outlined'}
            onClick={() => handleBooleanToggle('requires_gpu')}
          />
          <Chip
            label="Official only"
            clickable
            color={localFilters.official_only ? 'primary' : 'default'}
            variant={localFilters.official_only ? 'filled' : 'outlined'}
            onClick={() => handleBooleanToggle('official_only')}
          />
          <Chip
            label="Verified only"
            clickable
            color={localFilters.verified_only ? 'primary' : 'default'}
            variant={localFilters.verified_only ? 'filled' : 'outlined'}
            onClick={() => handleBooleanToggle('verified_only')}
          />
        </Box>
      </Section>

      <Section title="Use Cases">
        {renderChips(
          USE_CASES as unknown as string[],
          'use_cases',
          stats?.use_case_counts
        )}
      </Section>

      <Section title="Capabilities">
        {renderChips(
          CAPABILITIES as unknown as string[],
          'capabilities',
          stats?.capability_counts
        )}
      </Section>

      <Section title="Robot Targets">
        {renderChips(
          ROBOT_TARGETS as unknown as string[],
          'robot_targets',
          stats?.robot_target_counts
        )}
      </Section>

      <Section title="Simulators">
        {renderChips(
          SIMULATORS as unknown as string[],
          'simulators',
          stats?.simulator_counts
        )}
      </Section>

      <Section title="ROS Distro">
        {renderChips(
          ROS_DISTROS as unknown as string[],
          'ros_distros',
          stats?.ros_distro_counts
        )}
      </Section>

      <Section title="RMW">
        {renderChips(
          RMW_IMPLEMENTATIONS as unknown as string[],
          'rmw_implementations',
          stats?.rmw_counts
        )}
      </Section>

      <Section title="Difficulty">
        {renderChips(DIFFICULTY_LEVELS as unknown as string[], 'difficulty')}
      </Section>

      <Section title="Licenses">
        {renderChips(
          LICENSES as unknown as string[],
          'licenses',
          stats?.license_counts
        )}
      </Section>
    </Box>
  );

  if (permanent) {
    return filterContent;
  }

  return (
    <Drawer
      anchor={isMobile ? 'bottom' : 'left'}
      open={open}
      onClose={onClose}
      PaperProps={{
        sx: {
          width: isMobile ? '100%' : 320,
          height: isMobile ? '80vh' : '100%',
          borderTopLeftRadius: isMobile ? 16 : 0,
          borderTopRightRadius: isMobile ? 16 : 0,
        },
      }}
    >
      <Box sx={{ p: 2 }}>
        <Box
          sx={{
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center',
            mb: 2,
          }}
        >
          <Typography variant="h6">Filters</Typography>
          {isMobile && (
            <IconButton onClick={onClose} size="small">
              <CloseIcon fontSize="small" />
            </IconButton>
          )}
        </Box>
        {filterContent}
      </Box>
    </Drawer>
  );
};

export default TemplateFiltersComponent;
