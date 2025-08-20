import React from 'react';
import {
  Card as MuiCard,
  CardProps as MuiCardProps,
  CardContent,
  CardHeader,
  CardActions,
  CardMedia,
  Typography,
  Box,
  Chip,
  Avatar,
  IconButton,
  Collapse,
} from '@mui/material';
import {
  ExpandMore as ExpandMoreIcon,
  MoreVert as MoreVertIcon,
  Favorite as FavoriteIcon,
  Share as ShareIcon,
} from '@mui/icons-material';

// Extended card props
export interface CardProps extends Omit<MuiCardProps, 'variant'> {
  variant?: 'elevated' | 'outlined' | 'filled';
  size?: 'small' | 'medium' | 'large';
  title?: string;
  subtitle?: string;
  description?: string;
  image?: string;
  imageHeight?: number | string;
  avatar?: string;
  chips?: string[];
  actions?: React.ReactNode;
  expandable?: boolean;
  collapsible?: boolean;
  defaultExpanded?: boolean;
  onExpand?: (expanded: boolean) => void;
  onFavorite?: () => void;
  onShare?: () => void;
  onMore?: () => void;
  children?: React.ReactNode;
}

// Card component
const Card: React.FC<CardProps> = ({
  variant = 'elevated',
  size = 'medium',
  title,
  subtitle,
  description,
  image,
  imageHeight = 200,
  avatar,
  chips,
  actions,
  expandable = false,
  collapsible = false,
  defaultExpanded = false,
  onExpand,
  onFavorite,
  onShare,
  onMore,
  children,
  ...props
}) => {
  const [expanded, setExpanded] = React.useState(defaultExpanded);

  const handleExpandClick = () => {
    const newExpanded = !expanded;
    setExpanded(newExpanded);
    onExpand?.(newExpanded);
  };

  const getCardElevation = () => {
    switch (variant) {
      case 'elevated':
        return 2;
      case 'outlined':
        return 0;
      case 'filled':
        return 1;
      default:
        return 2;
    }
  };

  const getCardPadding = () => {
    switch (size) {
      case 'small':
        return 2;
      case 'large':
        return 4;
      default:
        return 3;
    }
  };

  return (
    <MuiCard
      elevation={getCardElevation()}
      variant={variant === 'outlined' ? 'outlined' : undefined}
      sx={{
        height: '100%',
        display: 'flex',
        flexDirection: 'column',
        transition: 'all 0.3s ease',
        '&:hover': {
          transform: 'translateY(-4px)',
          boxShadow: variant === 'elevated' ? 8 : 2,
        },
        ...(variant === 'filled' && {
          backgroundColor: 'grey.50',
        }),
      }}
      {...props}
    >
      {/* Card Media */}
      {image && (
        <CardMedia
          component="img"
          height={imageHeight}
          image={image}
          alt={title || 'Card image'}
          sx={{
            objectFit: 'cover',
          }}
        />
      )}

      {/* Card Header */}
      {(title || subtitle || avatar) && (
        <CardHeader
          avatar={
            avatar ? (
              <Avatar src={avatar} alt={title}>
                {title?.charAt(0)}
              </Avatar>
            ) : undefined
          }
          action={
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
              {onFavorite && (
                <IconButton
                  aria-label="add to favorites"
                  onClick={onFavorite}
                  size="small"
                >
                  <FavoriteIcon />
                </IconButton>
              )}
              {onShare && (
                <IconButton aria-label="share" onClick={onShare} size="small">
                  <ShareIcon />
                </IconButton>
              )}
              {onMore && (
                <IconButton
                  aria-label="more options"
                  onClick={onMore}
                  size="small"
                >
                  <MoreVertIcon />
                </IconButton>
              )}
              {(expandable || collapsible) && (
                <IconButton
                  onClick={handleExpandClick}
                  sx={{
                    transform: expanded ? 'rotate(180deg)' : 'rotate(0deg)',
                    transition: 'transform 0.3s ease',
                  }}
                  size="small"
                >
                  <ExpandMoreIcon />
                </IconButton>
              )}
            </Box>
          }
          title={
            title ? (
              <Typography variant="h6" component="div">
                {title}
              </Typography>
            ) : undefined
          }
          subheader={
            subtitle ? (
              <Typography variant="body2" color="text.secondary">
                {subtitle}
              </Typography>
            ) : undefined
          }
        />
      )}

      {/* Card Content */}
      <CardContent sx={{ flexGrow: 1, p: getCardPadding() }}>
        {description && (
          <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
            {description}
          </Typography>
        )}

        {/* Chips */}
        {chips && chips.length > 0 && (
          <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1, mb: 2 }}>
            {chips.map((chip, index) => (
              <Chip
                key={index}
                label={chip}
                size="small"
                variant="outlined"
                color="primary"
              />
            ))}
          </Box>
        )}

        {/* Children content */}
        {children}
      </CardContent>

      {/* Expandable content */}
      {(expandable || collapsible) && (
        <Collapse in={expanded} timeout="auto" unmountOnExit>
          <CardContent sx={{ pt: 0 }}>
            <Typography variant="body2" color="text.secondary">
              Additional content goes here...
            </Typography>
          </CardContent>
        </Collapse>
      )}

      {/* Card Actions */}
      {actions && (
        <CardActions sx={{ p: getCardPadding(), pt: 0 }}>{actions}</CardActions>
      )}
    </MuiCard>
  );
};

// Specialized card components
export const InfoCard: React.FC<
  Omit<CardProps, 'variant'> & { icon?: React.ReactNode }
> = ({ icon, ...props }) => (
  <Card
    variant="elevated"
    sx={{
      borderLeft: '4px solid',
      borderLeftColor: 'primary.main',
    }}
    {...props}
  >
    {icon && (
      <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>{icon}</Box>
    )}
    {props.children}
  </Card>
);

export const StatsCard: React.FC<
  Omit<CardProps, 'variant'> & {
    value: string | number;
    trend?: 'up' | 'down' | 'neutral';
    trendValue?: string;
  }
> = ({ value, trend, trendValue, ...props }) => (
  <Card
    variant="elevated"
    sx={{
      textAlign: 'center',
      '&:hover': {
        transform: 'translateY(-2px)',
      },
    }}
    {...props}
  >
    <CardContent>
      <Typography variant="h3" component="div" sx={{ mb: 1 }}>
        {value}
      </Typography>
      {props.title && (
        <Typography variant="body2" color="text.secondary" sx={{ mb: 1 }}>
          {props.title}
        </Typography>
      )}
      {trend && trendValue && (
        <Typography
          variant="caption"
          sx={{
            color:
              trend === 'up'
                ? 'success.main'
                : trend === 'down'
                  ? 'error.main'
                  : 'text.secondary',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            gap: 0.5,
          }}
        >
          {trend === 'up' ? '↗' : trend === 'down' ? '↘' : '→'} {trendValue}
        </Typography>
      )}
    </CardContent>
  </Card>
);

export const ActionCard: React.FC<
  Omit<CardProps, 'variant'> & {
    onClick?: () => void;
    clickable?: boolean;
  }
> = ({ onClick, clickable = true, ...props }) => (
  <Card
    variant="elevated"
    onClick={onClick}
    sx={{
      cursor: clickable ? 'pointer' : 'default',
      '&:hover': {
        transform: clickable ? 'translateY(-4px)' : 'none',
      },
    }}
    {...props}
  />
);

export default Card;
