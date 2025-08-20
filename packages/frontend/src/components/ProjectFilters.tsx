import React, { useState } from 'react';
import {
  Box,
  Typography,
  Button,
  Chip,
  TextField,
  Divider,
  IconButton,
  useTheme,
  useMediaQuery,
  Drawer,
} from '@mui/material';
import { Clear as ClearIcon, Close as CloseIcon } from '@mui/icons-material';
import { ProjectFilters, FilterCategory, FilterValue } from '@robium/shared';

interface ProjectFiltersProps {
  open: boolean;
  onClose: () => void;
  filters: ProjectFilters;
  onFiltersChange: (filters: ProjectFilters) => void;
  categories: FilterCategory[];
  values: FilterValue[];
  stats?: Record<string, Record<string, number>>;
  availableTags?: string[];
  permanent?: boolean;
}

const ProjectFiltersComponent: React.FC<ProjectFiltersProps> = ({
  open,
  onClose,
  filters,
  onFiltersChange,
  categories,
  values,
  stats,
  availableTags = [],
  permanent = false,
}) => {
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('md'));
  const [localFilters, setLocalFilters] = useState<ProjectFilters>(filters);
  const [newTag, setNewTag] = useState('');

  const handleArrayToggle = (key: keyof ProjectFilters, value: string) => {
    const current = (localFilters[key] as string[]) || [];
    const exists = current.includes(value);
    const updated = exists ? current.filter((v) => v !== value) : [...current, value];
    const next = { ...localFilters, [key]: updated } as ProjectFilters;
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

  const Section: React.FC<{ title: string; children: React.ReactNode }> = ({ title, children }) => (
    <Box sx={{ mb: 2 }}>
      <Typography variant="subtitle2" sx={{ mb: 1, fontWeight: 600 }}>
        {title}
      </Typography>
      {children}
    </Box>
  );

  const renderChips = (
    categoryId: string,
    key: keyof ProjectFilters,
    counts?: Record<string, number>
  ) => {
    const categoryValues = values.filter((v) => v.categoryId === categoryId && v.isActive);
    const sortedValues = categoryValues.sort((a, b) => a.sortOrder - b.sortOrder);

    return (
      <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
        {sortedValues.map((item) => {
          const selected = (localFilters[key] as string[])?.includes(item.value) || false;
          const count = counts?.[item.value] ?? undefined;
          return (
            <Chip
              key={item.id}
              label={count !== undefined ? `${item.displayName} (${count})` : item.displayName}
              clickable
              color={selected ? 'primary' : 'default'}
              variant={selected ? 'filled' : 'outlined'}
              onClick={() => handleArrayToggle(key, item.value)}
              sx={{ height: 28 }}
            />
          );
        })}
      </Box>
    );
  };

  const filterContent = (
    <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
      {/* Tags Section */}
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

      {/* Dynamic filter sections based on categories */}
      {categories
        .filter((cat) => cat.isActive && cat.id !== 'tags')
        .sort((a, b) => a.sortOrder - b.sortOrder)
        .map((category) => {
          const key = category.name as keyof ProjectFilters;
          const counts = stats?.[category.name];

          return (
            <Section key={category.id} title={category.displayName}>
              {renderChips(category.id, key, counts)}
            </Section>
          );
        })}
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

export default ProjectFiltersComponent;
