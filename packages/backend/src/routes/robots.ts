import express from 'express';
import { Database } from '../utils/database';
import { AuthRequest } from '../types';

const router = express.Router();

// GET /robots - List supported robots with module counts
router.get('/', async (req: AuthRequest, res) => {
  try {
    const result = (await Database.query(
      `
      WITH robots AS (
        SELECT UNNEST(ARRAY['turtlebot3','turtlebot4','raspberrypi','nvidia-orin']) AS code
      )
      SELECT r.code,
             CASE r.code
               WHEN 'turtlebot3' THEN 'TurtleBot 3'
               WHEN 'turtlebot4' THEN 'TurtleBot 4'
               WHEN 'raspberrypi' THEN 'Raspberry Pi'
               WHEN 'nvidia-orin' THEN 'NVIDIA Orin'
               ELSE r.code
             END AS name,
             COUNT(m.id) AS module_count
      FROM robots r
      LEFT JOIN modules m ON m.is_active = true AND (m.supported_robots IS NOT NULL AND r.code = ANY(m.supported_robots))
      GROUP BY r.code
      ORDER BY name
      `
    )) as { rows: Array<{ code: string; name: string; module_count: number }> };

    res.json({ success: true, data: result.rows });
  } catch (error) {
    console.error('Error fetching robots:', error);
    res.status(500).json({ success: false, error: 'Failed to fetch robots' });
  }
});

export default router;
