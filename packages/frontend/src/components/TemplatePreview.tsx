import React from 'react';
import {
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  Button,
  Typography,
  Box,
  Chip,
  Grid,
  Avatar,
  Divider,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  Paper,
  useTheme,
  useMediaQuery,
} from '@mui/material';
import {
  Close as CloseIcon,
  PlayArrow as LaunchIcon,
  BookmarkBorder as BookmarkIcon,
  Bookmark as BookmarkedIcon,
  Star as StarIcon,
  Download as DownloadIcon,
  AccessTime as TimeIcon,
  Person as PersonIcon,
  Code as CodeIcon,
  Memory as GpuIcon,
  Verified as VerifiedIcon,
  Security as OfficialIcon,
  NewReleases as NewIcon,
  CheckCircle as CheckIcon,
  Info as InfoIcon,
} from '@mui/icons-material';
import { Template } from '../types/template';

interface TemplatePreviewProps {
  template: Template | null;
  open: boolean;
  onClose: () => void;
  onLaunch: (template: Template) => void;
  onBookmark: (templateId: string, bookmarked: boolean) => void;
  isBookmarked?: boolean;
}

const TemplatePreview: React.FC<TemplatePreviewProps> = ({
  template,
  open,
  onClose,
  onLaunch,
  onBookmark,
  isBookmarked = false,
}) => {
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('md'));

  if (!template) return null;

  const formatDate = (dateString: string) => {
    return new Date(dateString).toLocaleDateString('en-US', {
      year: 'numeric',
      month: 'long',
      day: 'numeric',
    });
  };

  const getDifficultyColor = (difficulty: string) => {
    switch (difficulty) {
      case 'beginner':
        return theme.palette.success.main;
      case 'intermediate':
        return theme.palette.warning.main;
      case 'advanced':
        return theme.palette.error.main;
      default:
        return theme.palette.grey[500];
    }
  };

  const renderBadges = () => (
    <Box sx={{ display: 'flex', gap: 1, mb: 2, flexWrap: 'wrap' }}>
      {template.is_official && (
        <Chip
          icon={<OfficialIcon />}
          label="Official"
          color="primary"
          variant="filled"
        />
      )}
      {template.is_verified && (
        <Chip
          icon={<VerifiedIcon />}
          label="Verified"
          color="success"
          variant="filled"
        />
      )}
      {template.requires_gpu && (
        <Chip
          icon={<GpuIcon />}
          label="GPU Required"
          color="warning"
          variant="filled"
        />
      )}
      {template.is_new && (
        <Chip
          icon={<NewIcon />}
          label="New"
          color="secondary"
          variant="filled"
        />
      )}
    </Box>
  );

  const renderStats = () => (
    <Box sx={{ display: 'flex', gap: 3, mb: 3, color: 'text.secondary' }}>
      {template.installs_7d !== undefined && (
        <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
          <DownloadIcon fontSize="small" />
          <Typography variant="body2">
            {template.installs_7d.toLocaleString()} installs (7d)
          </Typography>
        </Box>
      )}

      {template.rating !== undefined && (
        <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
          <StarIcon
            fontSize="small"
            sx={{ color: theme.palette.warning.main }}
          />
          <Typography variant="body2">
            {template.rating.toFixed(1)}
            {template.rating_count && ` (${template.rating_count} reviews)`}
          </Typography>
        </Box>
      )}

      <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
        <TimeIcon fontSize="small" />
        <Typography variant="body2">
          Updated {formatDate(template.updated_at)}
        </Typography>
      </Box>
    </Box>
  );

  const renderMetadata = () => (
    <Grid container spacing={2} sx={{ mb: 3 }}>
      {template.metadata?.use_cases &&
        template.metadata.use_cases.length > 0 && (
          <Grid item xs={12} sm={6}>
            <Typography variant="subtitle2" sx={{ mb: 1, fontWeight: 600 }}>
              Use Cases
            </Typography>
            <Box sx={{ display: 'flex', gap: 0.5, flexWrap: 'wrap' }}>
              {template.metadata.use_cases.map((useCase) => (
                <Chip
                  key={useCase}
                  label={useCase}
                  size="small"
                  color="primary"
                />
              ))}
            </Box>
          </Grid>
        )}

      {template.metadata?.capabilities &&
        template.metadata.capabilities.length > 0 && (
          <Grid item xs={12} sm={6}>
            <Typography variant="subtitle2" sx={{ mb: 1, fontWeight: 600 }}>
              Capabilities
            </Typography>
            <Box sx={{ display: 'flex', gap: 0.5, flexWrap: 'wrap' }}>
              {template.metadata.capabilities.map((capability) => (
                <Chip
                  key={capability}
                  label={capability}
                  size="small"
                  color="secondary"
                />
              ))}
            </Box>
          </Grid>
        )}

      {template.metadata?.robot_targets &&
        template.metadata.robot_targets.length > 0 && (
          <Grid item xs={12} sm={6}>
            <Typography variant="subtitle2" sx={{ mb: 1, fontWeight: 600 }}>
              Robot Targets
            </Typography>
            <Box sx={{ display: 'flex', gap: 0.5, flexWrap: 'wrap' }}>
              {template.metadata.robot_targets.map((target) => (
                <Chip
                  key={target}
                  label={target}
                  size="small"
                  variant="outlined"
                />
              ))}
            </Box>
          </Grid>
        )}

      {template.metadata?.simulators &&
        template.metadata.simulators.length > 0 && (
          <Grid item xs={12} sm={6}>
            <Typography variant="subtitle2" sx={{ mb: 1, fontWeight: 600 }}>
              Simulators
            </Typography>
            <Box sx={{ display: 'flex', gap: 0.5, flexWrap: 'wrap' }}>
              {template.metadata.simulators.map((simulator) => (
                <Chip
                  key={simulator}
                  label={simulator}
                  size="small"
                  variant="outlined"
                />
              ))}
            </Box>
          </Grid>
        )}

      {template.metadata?.difficulty && (
        <Grid item xs={12} sm={6}>
          <Typography variant="subtitle2" sx={{ mb: 1, fontWeight: 600 }}>
            Difficulty
          </Typography>
          <Chip
            label={template.metadata.difficulty}
            size="small"
            sx={{
              backgroundColor: getDifficultyColor(template.metadata.difficulty),
              color: 'white',
            }}
          />
        </Grid>
      )}

      {template.license && (
        <Grid item xs={12} sm={6}>
          <Typography variant="subtitle2" sx={{ mb: 1, fontWeight: 600 }}>
            License
          </Typography>
          <Chip label={template.license} size="small" variant="outlined" />
        </Grid>
      )}
    </Grid>
  );

  const renderRequirements = () => {
    const requirements = [];

    if (template.metadata?.hardware_requirements) {
      requirements.push(...template.metadata.hardware_requirements);
    }

    if (template.metadata?.dependencies) {
      requirements.push(...template.metadata.dependencies);
    }

    if (requirements.length === 0) return null;

    return (
      <Box sx={{ mb: 3 }}>
        <Typography variant="subtitle2" sx={{ mb: 1, fontWeight: 600 }}>
          Requirements
        </Typography>
        <List dense>
          {requirements.map((req, index) => (
            <ListItem key={index} sx={{ py: 0.5 }}>
              <ListItemIcon sx={{ minWidth: 32 }}>
                <CheckIcon fontSize="small" color="success" />
              </ListItemIcon>
              <ListItemText primary={req} />
            </ListItem>
          ))}
        </List>
      </Box>
    );
  };

  const renderIncludedModules = () => (
    <Box sx={{ mb: 3 }}>
      <Typography variant="subtitle2" sx={{ mb: 1, fontWeight: 600 }}>
        Included Modules ({template.module_count})
      </Typography>
      <Paper
        variant="outlined"
        sx={{ p: 2, backgroundColor: theme.palette.grey[50] }}
      >
        <Typography variant="body2" color="text.secondary">
          This template includes {template.module_count} modules and{' '}
          {template.package_count} ROS packages.
          {/* TODO: Show actual module names when available */}
        </Typography>
      </Paper>
    </Box>
  );

  return (
    <Dialog
      open={open}
      onClose={onClose}
      maxWidth="md"
      fullWidth
      fullScreen={isMobile}
      PaperProps={{
        sx: {
          maxHeight: isMobile ? '100%' : '90vh',
        },
      }}
    >
      <DialogTitle sx={{ pb: 1 }}>
        <Box
          sx={{
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'space-between',
          }}
        >
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
            <Avatar sx={{ bgcolor: 'primary.main' }}>
              {template.name.charAt(0).toUpperCase()}
            </Avatar>
            <Box>
              <Typography variant="h6" component="h2">
                {template.name}
              </Typography>
              <Typography variant="body2" color="text.secondary">
                by {template.author}
              </Typography>
            </Box>
          </Box>
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            <Button
              variant="outlined"
              size="small"
              startIcon={isBookmarked ? <BookmarkedIcon /> : <BookmarkIcon />}
              onClick={() => onBookmark(template.id, !isBookmarked)}
            >
              {isBookmarked ? 'Bookmarked' : 'Bookmark'}
            </Button>
            <Button variant="outlined" size="small" onClick={onClose}>
              <CloseIcon />
            </Button>
          </Box>
        </Box>
      </DialogTitle>

      <DialogContent sx={{ pt: 0 }}>
        {/* Badges */}
        {renderBadges()}

        {/* Stats */}
        {renderStats()}

        {/* Description */}
        <Typography variant="body1" sx={{ mb: 3 }}>
          {template.description}
        </Typography>

        <Divider sx={{ my: 3 }} />

        {/* Metadata */}
        {renderMetadata()}

        {/* Requirements */}
        {renderRequirements()}

        {/* Included Modules */}
        {renderIncludedModules()}

        {/* Tags */}
        {template.tags && template.tags.length > 0 && (
          <Box sx={{ mb: 3 }}>
            <Typography variant="subtitle2" sx={{ mb: 1, fontWeight: 600 }}>
              Tags
            </Typography>
            <Box sx={{ display: 'flex', gap: 0.5, flexWrap: 'wrap' }}>
              {template.tags.map((tag) => (
                <Chip key={tag} label={tag} size="small" variant="outlined" />
              ))}
            </Box>
          </Box>
        )}
      </DialogContent>

      <DialogActions sx={{ p: 3, pt: 0 }}>
        <Button onClick={onClose} variant="outlined">
          Cancel
        </Button>
        <Button
          onClick={() => onLaunch(template)}
          variant="contained"
          startIcon={<LaunchIcon />}
          size="large"
        >
          Launch Template
        </Button>
      </DialogActions>
    </Dialog>
  );
};

export default TemplatePreview;
