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

    // Get module statistics
    const totalModules = (await Database.query(
      'SELECT COUNT(*)::int AS c FROM modules'
    )) as { rows: Array<{ c: number }> };

    const activeModules = (await Database.query(
      'SELECT COUNT(*)::int AS c FROM modules WHERE is_active = true'
    )) as { rows: Array<{ c: number }> };

    // Get robot statistics
    const robotStats = (await Database.query(`
      SELECT 
        robot_name,
        COUNT(*)::int AS module_count
      FROM (
        SELECT DISTINCT unnest(supported_robots) AS robot_name
        FROM modules 
        WHERE is_active = true AND supported_robots IS NOT NULL
      ) AS robot_names
      GROUP BY robot_name
      ORDER BY module_count DESC
    `)) as { rows: Array<{ robot_name: string; module_count: number }> };

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
        // User statistics
        totalUsers: totalUsers.rows[0]?.c ?? 0,
        adminUsers: adminUsers.rows[0]?.c ?? 0,
        regularUsers: regularUsers.rows[0]?.c ?? 0,
        activeUsers: totalUsers.rows[0]?.c ?? 0, // Assuming all users are active for now

        // Project statistics
        totalProjects: totalProjects.rows[0]?.c ?? 0,
        activeProjects: activeProjects.rows[0]?.c ?? 0,
        templates: templates.rows[0]?.c ?? 0,

        // Module statistics
        totalModules: totalModules.rows[0]?.c ?? 0,
        activeModules: activeModules.rows[0]?.c ?? 0,

        // Robot statistics
        robots: robotStats.rows,
        totalRobots: robotStats.rows.length,

        // System metrics
        systemHealth,

        // Storage metrics (placeholder for now)
        totalStorage: 1000000000000, // 1TB in bytes
        usedStorage: Math.floor(Math.random() * 200000000000) + 50000000000, // 50-250GB for demo

        lastUpdated: new Date().toISOString(),
      },
    });
  } catch (error) {
    console.error('Error fetching dashboard stats:', error);
    res.status(500).json({ success: false, error: 'Failed to fetch stats' });
  }
});

export default router;
