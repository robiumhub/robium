import React from 'react';
import { Box, Skeleton as MuiSkeleton, Card, CardContent } from '@mui/material';

// Skeleton variants
interface SkeletonProps {
  variant?: 'text' | 'rectangular' | 'circular';
  width?: string | number;
  height?: string | number;
  animation?: 'pulse' | 'wave' | false;
}

// Text skeleton component
export const TextSkeleton: React.FC<SkeletonProps> = ({
  variant = 'text',
  width = '100%',
  height,
  animation = 'pulse',
}) => (
  <MuiSkeleton
    variant={variant}
    width={width}
    height={height}
    animation={animation}
  />
);

// Card skeleton component
export const CardSkeleton: React.FC<{
  height?: number;
  showContent?: boolean;
  lines?: number;
}> = ({ height = 200, showContent = true, lines = 3 }) => (
  <Card sx={{ height }}>
    <CardContent>
      <MuiSkeleton variant="rectangular" height={height * 0.4} sx={{ mb: 2 }} />
      {showContent && (
        <Box>
          {Array.from({ length: lines }).map((_, index) => (
            <MuiSkeleton
              key={index}
              variant="text"
              width={index === lines - 1 ? '60%' : '100%'}
              sx={{ mb: 1 }}
            />
          ))}
        </Box>
      )}
    </CardContent>
  </Card>
);

// Dashboard stats skeleton
export const StatsSkeleton: React.FC = () => (
  <Box
    sx={{
      display: 'grid',
      gridTemplateColumns: {
        xs: '1fr',
        sm: 'repeat(2, 1fr)',
        md: 'repeat(3, 1fr)',
      },
      gap: 3,
    }}
  >
    {Array.from({ length: 3 }).map((_, index) => (
      <Card key={index}>
        <CardContent>
          <Box
            display="flex"
            alignItems="center"
            justifyContent="space-between"
          >
            <Box flex={1}>
              <MuiSkeleton variant="text" width="60%" sx={{ mb: 1 }} />
              <MuiSkeleton variant="text" width="40%" height={40} />
            </Box>
            <MuiSkeleton variant="circular" width={40} height={40} />
          </Box>
        </CardContent>
      </Card>
    ))}
  </Box>
);

// List skeleton component
export const ListSkeleton: React.FC<{
  items?: number;
  showAvatar?: boolean;
  showSubtitle?: boolean;
}> = ({ items = 5, showAvatar = true, showSubtitle = true }) => (
  <Box>
    {Array.from({ length: items }).map((_, index) => (
      <Box
        key={index}
        sx={{
          display: 'flex',
          alignItems: 'center',
          py: 2,
          borderBottom: index < items - 1 ? '1px solid' : 'none',
          borderColor: 'divider',
        }}
      >
        {showAvatar && (
          <MuiSkeleton
            variant="circular"
            width={40}
            height={40}
            sx={{ mr: 2 }}
          />
        )}
        <Box flex={1}>
          <MuiSkeleton variant="text" width="70%" />
          {showSubtitle && (
            <MuiSkeleton variant="text" width="50%" sx={{ mt: 0.5 }} />
          )}
        </Box>
      </Box>
    ))}
  </Box>
);

// Table skeleton component
export const TableSkeleton: React.FC<{
  rows?: number;
  columns?: number;
}> = ({ rows = 5, columns = 4 }) => (
  <Box>
    {/* Header */}
    <Box
      sx={{
        display: 'flex',
        py: 2,
        borderBottom: '2px solid',
        borderColor: 'divider',
      }}
    >
      {Array.from({ length: columns }).map((_, index) => (
        <Box key={index} flex={1} sx={{ px: 1 }}>
          <MuiSkeleton variant="text" width="80%" />
        </Box>
      ))}
    </Box>
    {/* Rows */}
    {Array.from({ length: rows }).map((_, rowIndex) => (
      <Box
        key={rowIndex}
        sx={{
          display: 'flex',
          py: 2,
          borderBottom: rowIndex < rows - 1 ? '1px solid' : 'none',
          borderColor: 'divider',
        }}
      >
        {Array.from({ length: columns }).map((_, colIndex) => (
          <Box key={colIndex} flex={1} sx={{ px: 1 }}>
            <MuiSkeleton variant="text" width="90%" />
          </Box>
        ))}
      </Box>
    ))}
  </Box>
);

// Dashboard skeleton component
export const DashboardSkeleton: React.FC = () => (
  <Box>
    {/* Header */}
    <MuiSkeleton variant="text" width="40%" height={40} sx={{ mb: 1 }} />
    <MuiSkeleton variant="text" width="60%" sx={{ mb: 4 }} />

    {/* Stats Cards */}
    <StatsSkeleton />

    {/* Content Grid */}
    <Box sx={{ mt: 4 }}>
      <Box
        sx={{
          display: 'grid',
          gridTemplateColumns: { xs: '1fr', md: '2fr 1fr' },
          gap: 3,
        }}
      >
        <Card>
          <CardContent>
            <MuiSkeleton
              variant="text"
              width="30%"
              height={32}
              sx={{ mb: 2 }}
            />
            <MuiSkeleton variant="text" width="100%" sx={{ mb: 1 }} />
            <MuiSkeleton variant="text" width="80%" sx={{ mb: 1 }} />
            <MuiSkeleton variant="text" width="60%" />
          </CardContent>
        </Card>
        <Card>
          <CardContent>
            <MuiSkeleton
              variant="text"
              width="40%"
              height={32}
              sx={{ mb: 2 }}
            />
            <MuiSkeleton variant="text" width="100%" sx={{ mb: 1 }} />
            <MuiSkeleton variant="text" width="70%" />
          </CardContent>
        </Card>
      </Box>
    </Box>
  </Box>
);

// Form skeleton component
export const FormSkeleton: React.FC<{
  fields?: number;
  showSubmit?: boolean;
}> = ({ fields = 4, showSubmit = true }) => (
  <Box>
    {Array.from({ length: fields }).map((_, index) => (
      <Box key={index} sx={{ mb: 3 }}>
        <MuiSkeleton variant="text" width="30%" sx={{ mb: 1 }} />
        <MuiSkeleton variant="rectangular" height={56} />
      </Box>
    ))}
    {showSubmit && (
      <MuiSkeleton variant="rectangular" height={48} width="100%" />
    )}
  </Box>
);

// Navigation skeleton component
export const NavigationSkeleton: React.FC = () => (
  <Box>
    {Array.from({ length: 4 }).map((_, index) => (
      <Box
        key={index}
        sx={{
          display: 'flex',
          alignItems: 'center',
          py: 1.5,
          px: 2,
        }}
      >
        <MuiSkeleton variant="circular" width={24} height={24} sx={{ mr: 2 }} />
        <MuiSkeleton variant="text" width="60%" />
      </Box>
    ))}
  </Box>
);

const SkeletonComponents = {
  TextSkeleton,
  CardSkeleton,
  StatsSkeleton,
  ListSkeleton,
  TableSkeleton,
  DashboardSkeleton,
  FormSkeleton,
  NavigationSkeleton,
};

export default SkeletonComponents;
