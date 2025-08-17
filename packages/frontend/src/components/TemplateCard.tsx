import React, { useState } from 'react';
import {
  Card,
  CardContent,
  CardActions,
  Typography,
  Box,
  Chip,
  Button,
  IconButton,
  Tooltip,
  Avatar,
  Badge,
  Skeleton,
  useTheme,
  useMediaQuery,
} from '@mui/material';
import {
  PlayArrow as LaunchIcon,
  Visibility as PreviewIcon,
  BookmarkBorder as BookmarkIcon,
  Bookmark as BookmarkedIcon,
  Star as StarIcon,
  Download as DownloadIcon,
  Verified as VerifiedIcon,
  Security as OfficialIcon,
  Memory as GpuIcon,
  NewReleases as NewIcon,
  AccessTime as TimeIcon,
  Person as PersonIcon,
} from '@mui/icons-material';
import { Template } from '../types/template';

interface TemplateCardProps {
  template: Template;
  onPreview: (template: Template) => void;
  onLaunch: (template: Template) => void;
  onBookmark: (templateId: string, bookmarked: boolean) => void;
  isBookmarked?: boolean;
  loading?: boolean;
}

const TemplateCard: React.FC<TemplateCardProps> = ({
  template,
  onPreview,
  onLaunch,
  onBookmark,
  isBookmarked = false,
  loading = false,
}) => {
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('sm'));
  const [imageLoaded, setImageLoaded] = useState(false);
  const [imageError, setImageError] = useState(false);

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

  const getLicenseColor = (license: string) => {
    switch (license) {
      case 'MIT':
        return theme.palette.success.main;
      case 'Apache-2.0':
        return theme.palette.info.main;
      case 'GPL-3.0':
        return theme.palette.warning.main;
      default:
        return theme.palette.grey[500];
    }
  };

  const formatDate = (dateString: string) => {
    const date = new Date(dateString);
    const now = new Date();
    const diffTime = Math.abs(now.getTime() - date.getTime());
    const diffDays = Math.ceil(diffTime / (1000 * 60 * 60 * 24));

    if (diffDays === 1) return 'Today';
    if (diffDays <= 7) return `${diffDays} days ago`;
    if (diffDays <= 30) return `${Math.floor(diffDays / 7)} weeks ago`;
    return date.toLocaleDateString();
  };

  const renderBadges = () => (
    <Box sx={{ display: 'flex', gap: 0.5, mb: 1, flexWrap: 'wrap' }}>
      {template.is_official && (
        <Chip
          icon={<OfficialIcon />}
          label="Official"
          size="small"
          color="primary"
          variant="filled"
        />
      )}
      {template.is_verified && (
        <Chip
          icon={<VerifiedIcon />}
          label="Verified"
          size="small"
          color="success"
          variant="filled"
        />
      )}
      {template.requires_gpu && (
        <Chip
          icon={<GpuIcon />}
          label="GPU"
          size="small"
          color="warning"
          variant="filled"
        />
      )}
      {template.is_new && (
        <Chip
          icon={<NewIcon />}
          label="New"
          size="small"
          color="secondary"
          variant="filled"
        />
      )}
    </Box>
  );

  const renderPills = () => (
    <Box sx={{ display: 'flex', gap: 0.5, mb: 2, flexWrap: 'wrap' }}>
      {/* Use Cases (1-2) */}
      {template.metadata?.use_cases?.slice(0, 2).map((useCase) => (
        <Chip
          key={useCase}
          label={useCase}
          size="small"
          variant="outlined"
          color="primary"
        />
      ))}

      {/* Capabilities (1-3) */}
      {template.metadata?.capabilities?.slice(0, 3).map((capability) => (
        <Chip
          key={capability}
          label={capability}
          size="small"
          variant="outlined"
          color="secondary"
        />
      ))}
    </Box>
  );

  const renderStats = () => (
    <Box
      sx={{
        display: 'flex',
        alignItems: 'center',
        gap: 2,
        mb: 2,
        color: 'text.secondary',
      }}
    >
      {template.installs_7d !== undefined && (
        <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
          <DownloadIcon fontSize="small" />
          <Typography variant="caption">
            {template.installs_7d.toLocaleString()}
          </Typography>
        </Box>
      )}

      {template.rating !== undefined && (
        <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
          <StarIcon
            fontSize="small"
            sx={{ color: theme.palette.warning.main }}
          />
          <Typography variant="caption">
            {template.rating.toFixed(1)}
            {template.rating_count && ` (${template.rating_count})`}
          </Typography>
        </Box>
      )}

      <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
        <TimeIcon fontSize="small" />
        <Typography variant="caption">
          {formatDate(template.updated_at)}
        </Typography>
      </Box>
    </Box>
  );

  const renderAuthor = () => (
    <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 1 }}>
      <Avatar sx={{ width: 20, height: 20, fontSize: '0.75rem' }}>
        <PersonIcon fontSize="small" />
      </Avatar>
      <Typography variant="caption" color="text.secondary">
        {template.author}
      </Typography>
    </Box>
  );

  if (loading) {
    return (
      <Card sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
        <CardContent sx={{ flexGrow: 1 }}>
          <Skeleton variant="text" width="60%" height={24} sx={{ mb: 1 }} />
          <Skeleton variant="text" width="100%" height={16} sx={{ mb: 2 }} />
          <Skeleton
            variant="rectangular"
            width="100%"
            height={120}
            sx={{ mb: 2 }}
          />
          <Box sx={{ display: 'flex', gap: 1, mb: 2 }}>
            <Skeleton variant="rectangular" width={60} height={24} />
            <Skeleton variant="rectangular" width={80} height={24} />
          </Box>
          <Skeleton variant="text" width="40%" height={16} />
        </CardContent>
        <CardActions>
          <Skeleton variant="rectangular" width={80} height={32} />
          <Skeleton variant="rectangular" width={60} height={32} />
        </CardActions>
      </Card>
    );
  }

  return (
    <Card
      sx={{
        height: '100%',
        display: 'flex',
        flexDirection: 'column',
        transition: 'all 0.2s ease-in-out',
        '&:hover': {
          transform: 'translateY(-2px)',
          boxShadow: theme.shadows[8],
        },
        position: 'relative',
        overflow: 'hidden',
      }}
    >
      {/* Thumbnail/GIF Section */}
      <Box
        sx={{
          height: 160,
          backgroundColor: theme.palette.grey[100],
          position: 'relative',
          overflow: 'hidden',
        }}
      >
        {template.thumbnail_url && !imageError ? (
          <img
            src={template.thumbnail_url}
            alt={template.name}
            style={{
              width: '100%',
              height: '100%',
              objectFit: 'cover',
              opacity: imageLoaded ? 1 : 0,
              transition: 'opacity 0.3s ease-in-out',
            }}
            onLoad={() => setImageLoaded(true)}
            onError={() => setImageError(true)}
          />
        ) : (
          <Box
            sx={{
              width: '100%',
              height: '100%',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              backgroundColor: theme.palette.grey[200],
            }}
          >
            <Typography variant="h4" color="text.secondary">
              {template.name.charAt(0).toUpperCase()}
            </Typography>
          </Box>
        )}

        {/* Hover overlay for actions */}
        <Box
          sx={{
            position: 'absolute',
            top: 0,
            left: 0,
            right: 0,
            bottom: 0,
            backgroundColor: 'rgba(0, 0, 0, 0.7)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            gap: 1,
            opacity: 0,
            transition: 'opacity 0.2s ease-in-out',
            '&:hover': {
              opacity: 1,
            },
          }}
        >
          <Button
            variant="contained"
            size="small"
            startIcon={<PreviewIcon />}
            onClick={() => onPreview(template)}
          >
            Preview
          </Button>
          <Button
            variant="outlined"
            size="small"
            startIcon={<LaunchIcon />}
            onClick={() => onLaunch(template)}
            sx={{ color: 'white', borderColor: 'white' }}
          >
            Launch
          </Button>
        </Box>
      </Box>

      <CardContent sx={{ flexGrow: 1, p: 2 }}>
        {/* Header Row */}
        <Box
          sx={{
            display: 'flex',
            alignItems: 'flex-start',
            justifyContent: 'space-between',
            mb: 1,
          }}
        >
          <Typography
            variant="h6"
            component="h2"
            sx={{
              fontWeight: 600,
              lineHeight: 1.2,
              display: '-webkit-box',
              WebkitLineClamp: 2,
              WebkitBoxOrient: 'vertical',
              overflow: 'hidden',
              flex: 1,
              mr: 1,
            }}
          >
            {template.name}
          </Typography>

          <Tooltip
            title={isBookmarked ? 'Remove from bookmarks' : 'Add to bookmarks'}
          >
            <IconButton
              size="small"
              onClick={() => onBookmark(template.id, !isBookmarked)}
              color={isBookmarked ? 'primary' : 'default'}
            >
              {isBookmarked ? <BookmarkedIcon /> : <BookmarkIcon />}
            </IconButton>
          </Tooltip>
        </Box>

        {/* Badges */}
        {renderBadges()}

        {/* Summary */}
        <Typography
          variant="body2"
          color="text.secondary"
          sx={{
            mb: 2,
            lineHeight: 1.4,
            display: '-webkit-box',
            WebkitLineClamp: 2,
            WebkitBoxOrient: 'vertical',
            overflow: 'hidden',
          }}
        >
          {template.summary || template.description}
        </Typography>

        {/* Pills */}
        {renderPills()}

        {/* Stats */}
        {renderStats()}

        {/* Author */}
        {template.author && renderAuthor()}

        {/* License and Difficulty */}
        <Box sx={{ display: 'flex', gap: 1, mb: 1 }}>
          {template.license && (
            <Chip
              label={template.license}
              size="small"
              variant="outlined"
              sx={{
                borderColor: getLicenseColor(template.license),
                color: getLicenseColor(template.license),
              }}
            />
          )}
          {template.metadata?.difficulty && (
            <Chip
              label={template.metadata.difficulty}
              size="small"
              variant="outlined"
              sx={{
                borderColor: getDifficultyColor(template.metadata.difficulty),
                color: getDifficultyColor(template.metadata.difficulty),
              }}
            />
          )}
        </Box>
      </CardContent>

      {/* Quick Actions Row */}
      <CardActions sx={{ p: 2, pt: 0, gap: 1 }}>
        <Button
          variant="outlined"
          size="small"
          startIcon={<PreviewIcon />}
          onClick={() => onPreview(template)}
          fullWidth={isMobile}
        >
          Preview
        </Button>
        <Button
          variant="contained"
          size="small"
          startIcon={<LaunchIcon />}
          onClick={() => onLaunch(template)}
          fullWidth={isMobile}
        >
          Use Template
        </Button>
      </CardActions>
    </Card>
  );
};

export default TemplateCard;
