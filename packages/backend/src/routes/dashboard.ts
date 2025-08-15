import { Router } from 'express';
import { Database } from '../utils/database';

const router = Router();

// GET /dashboard/stats - platform overview statistics
router.get('/stats', async (req, res) => {
  try {
    // Get user statistics
    const totalUsers = (await Database.query(
      'SELECT COUNT(*)::int AS c FROM users'
    )) as { rows: Array<{ c: number }> };

    const adminUsers = (await Database.query(
      'SELECT COUNT(*)::int AS c FROM users WHERE role = $1',
      ['admin']
    )) as { rows: Array<{ c: number }> };

    const regularUsers = (await Database.query(
      'SELECT COUNT(*)::int AS c FROM users WHERE role = $1',
      ['user']
    )) as { rows: Array<{ c: number }> };

    // Get project statistics
    const totalProjects = (await Database.query(
      'SELECT COUNT(*)::int AS c FROM projects'
    )) as { rows: Array<{ c: number }> };

    const activeProjects = (await Database.query(
      'SELECT COUNT(*)::int AS c FROM projects WHERE is_active = true'
    )) as { rows: Array<{ c: number }> };

    const templates = (await Database.query(
      'SELECT COUNT(*)::int AS c FROM projects WHERE is_active = true AND is_template = true'
    )) as { rows: Array<{ c: number }> };

    // Module statistics - count ROS packages as modules
    const totalModules = (await Database.query(
      'SELECT COUNT(*)::int AS c FROM ros_packages WHERE is_active = true'
    )) as { rows: Array<{ c: number }> };

    const activeModules = totalModules.rows[0]?.c ?? 0;

    // Robot statistics - modules table was removed, return empty array
    const robotStats: Array<{ robot_name: string; module_count: number }> = [];

    // Calculate system health metrics (placeholder values for now)
    const systemHealth = {
      cpuUsage: Math.floor(Math.random() * 30) + 20, // 20-50% for demo
      memoryUsage: Math.floor(Math.random() * 40) + 30, // 30-70% for demo
      networkUsage: Math.floor(Math.random() * 20) + 10, // 10-30% for demo
      systemStatus: 'healthy' as const,
      uptime: process.uptime(),
    };

    res.json({
      success: true,
      data: {
        // Frontend expects these fields directly
        projects: totalProjects.rows[0]?.c ?? 0,
        modules: totalModules.rows[0]?.c ?? 0,
        templates: templates.rows[0]?.c ?? 0,
        datasets: 0, // Placeholder for now
      },
    });
  } catch (error) {
    console.error('Error fetching dashboard stats:', error);
    res.status(500).json({ success: false, error: 'Failed to fetch stats' });
  }
});

export default router;
