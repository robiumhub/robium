import React from 'react';
import {
  PieChart,
  Pie,
  Cell,
  ResponsiveContainer,
  Tooltip,
  Legend,
} from 'recharts';
import { Box, Typography, useTheme } from '@mui/material';

interface RobotCategoryData {
  category: string;
  robot_count: string;
}

interface RobotCategoryPieChartProps {
  data: RobotCategoryData[];
  title?: string;
  height?: number;
}

const RobotCategoryPieChart: React.FC<RobotCategoryPieChartProps> = ({
  data,
  title = 'Robot Categories',
  height = 300,
}) => {
  const theme = useTheme();

  // Transform data for Recharts
  const chartData = data.map((item) => ({
    name: item.category,
    value: parseInt(item.robot_count),
  }));

  // Generate colors for pie slices
  const colors = [
    theme.palette.primary.main,
    theme.palette.secondary.main,
    theme.palette.success.main,
    theme.palette.warning.main,
    theme.palette.error.main,
    theme.palette.info.main,
    theme.palette.grey[500],
    theme.palette.grey[700],
  ];

  const CustomTooltip = ({ active, payload }: any) => {
    if (active && payload && payload.length) {
      const data = payload[0];
      const total = chartData.reduce((sum, item) => sum + item.value, 0);
      const percentage = ((data.value / total) * 100).toFixed(1);

      return (
        <Box
          sx={{
            backgroundColor: theme.palette.background.paper,
            border: `1px solid ${theme.palette.divider}`,
            borderRadius: 1,
            p: 1,
            boxShadow: theme.shadows[3],
          }}
        >
          <Typography variant="body2" color="textSecondary">
            {data.name}
          </Typography>
          <Typography variant="body2" color="primary">
            {data.value} robots ({percentage}%)
          </Typography>
        </Box>
      );
    }
    return null;
  };

  const CustomLegend = ({ payload }: any) => {
    return (
      <Box
        sx={{
          display: 'flex',
          flexWrap: 'wrap',
          justifyContent: 'center',
          mt: 2,
        }}
      >
        {payload?.map((entry: any, index: number) => (
          <Box
            key={index}
            sx={{
              display: 'flex',
              alignItems: 'center',
              mr: 2,
              mb: 1,
            }}
          >
            <Box
              sx={{
                width: 12,
                height: 12,
                backgroundColor: entry.color,
                borderRadius: '50%',
                mr: 1,
              }}
            />
            <Typography variant="body2" color="textSecondary">
              {entry.value}
            </Typography>
          </Box>
        ))}
      </Box>
    );
  };

  return (
    <Box sx={{ width: '100%', height }}>
      <Typography variant="h6" gutterBottom>
        {title}
      </Typography>
      <ResponsiveContainer width="100%" height="100%">
        <PieChart>
          <Pie
            data={chartData}
            cx="50%"
            cy="50%"
            labelLine={false}
            label={({ name, percent }) =>
              `${name} ${(percent * 100).toFixed(0)}%`
            }
            outerRadius={80}
            fill="#8884d8"
            dataKey="value"
          >
            {chartData.map((entry, index) => (
              <Cell
                key={`cell-${index}`}
                fill={colors[index % colors.length]}
              />
            ))}
          </Pie>
          <Tooltip content={<CustomTooltip />} />
          <Legend content={<CustomLegend />} />
        </PieChart>
      </ResponsiveContainer>
    </Box>
  );
};

export default RobotCategoryPieChart;
