import { Router } from 'express';
import { Database } from '../utils/database';

const router = Router();

// GET /dashboard/stats - platform overview statistics
router.get('/stats', async (req, res) => {
  try {
    const projects = (await Database.query('SELECT COUNT(*)::int AS c FROM projects WHERE is_active = true')) as { rows: Array<{ c: number }> };
    const modules = (await Database.query('SELECT COUNT(*)::int AS c FROM modules WHERE is_active = true')) as { rows: Array<{ c: number }> };
    const templates = (await Database.query("SELECT COUNT(*)::int AS c FROM projects WHERE is_active = true AND is_template = true")) as { rows: Array<{ c: number }> };
    const datasets = { rows: [{ c: 0 }] }; // placeholder until datasets table exists

    res.json({
      success: true,
      data: {
        projects: projects.rows[0]?.c ?? 0,
        modules: modules.rows[0]?.c ?? 0,
        templates: templates.rows[0]?.c ?? 0,
        datasets: datasets.rows[0]?.c ?? 0,
        lastUpdated: new Date().toISOString(),
      },
    });
  } catch (error) {
    console.error('Error fetching dashboard stats:', error);
    res.status(500).json({ success: false, error: 'Failed to fetch stats' });
  }
});

export default router;


