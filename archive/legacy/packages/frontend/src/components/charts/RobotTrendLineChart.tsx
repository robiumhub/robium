import React from 'react';
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  Legend,
} from 'recharts';
import { Box, Typography, useTheme } from '@mui/material';

interface RobotTrendData {
  date: string;
  modules: number;
  projects: number;
}

interface RobotTrendLineChartProps {
  data: RobotTrendData[];
  title?: string;
  height?: number;
}

const RobotTrendLineChart: React.FC<RobotTrendLineChartProps> = ({
  data,
  title = 'Robot Usage Trends',
  height = 300,
}) => {
  const theme = useTheme();

  const CustomTooltip = ({ active, payload, label }: any) => {
    if (active && payload && payload.length) {
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
            {label}
          </Typography>
          {payload.map((entry: any, index: number) => (
            <Typography key={index} variant="body2" sx={{ color: entry.color }}>
              {entry.name}: {entry.value}
            </Typography>
          ))}
        </Box>
      );
    }
    return null;
  };

  return (
    <Box sx={{ width: '100%', height }}>
      <Typography variant="h6" gutterBottom>
        {title}
      </Typography>
      <ResponsiveContainer width="100%" height="100%">
        <LineChart
          data={data}
          margin={{ top: 20, right: 30, left: 20, bottom: 5 }}
        >
          <CartesianGrid strokeDasharray="3 3" stroke={theme.palette.divider} />
          <XAxis
            dataKey="date"
            tick={{ fontSize: 12 }}
            tickLine={false}
            axisLine={false}
          />
          <YAxis
            tick={{ fontSize: 12 }}
            tickLine={false}
            axisLine={false}
            label={{ value: 'Count', angle: -90, position: 'insideLeft' }}
          />
          <Tooltip content={<CustomTooltip />} />
          <Legend />
          <Line
            type="monotone"
            dataKey="modules"
            stroke={theme.palette.primary.main}
            strokeWidth={2}
            dot={{ fill: theme.palette.primary.main, strokeWidth: 2, r: 4 }}
            activeDot={{ r: 6 }}
            name="Modules"
          />
          <Line
            type="monotone"
            dataKey="projects"
            stroke={theme.palette.secondary.main}
            strokeWidth={2}
            dot={{ fill: theme.palette.secondary.main, strokeWidth: 2, r: 4 }}
            activeDot={{ r: 6 }}
            name="Projects"
          />
        </LineChart>
      </ResponsiveContainer>
    </Box>
  );
};

export default RobotTrendLineChart;
