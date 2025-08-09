import express from 'express';
import { Database } from '../utils/database';
import { AuthRequest } from '../types';

const router = express.Router();

// GET /robots - List supported robots with module counts
router.get('/', async (req: AuthRequest, res) => {
  try {
    const result = (await Database.query(
      `
      SELECT 
        robot_type as code,
        CASE robot_type
          WHEN 'turtlebot3' THEN 'TurtleBot 3'
          WHEN 'pioneer3at' THEN 'Pioneer 3-AT'
          WHEN 'kobuki' THEN 'Kobuki Base'
          WHEN 'create3' THEN 'iRobot Create 3'
          WHEN 'jackal' THEN 'Clearpath Jackal'
          WHEN 'husky' THEN 'Clearpath Husky'
          WHEN 'clearpath_robots' THEN 'Clearpath Robots'
          WHEN 'pr2' THEN 'Willow Garage PR2'
          WHEN 'fetch' THEN 'Fetch Robotics'
          WHEN 'baxter' THEN 'Rethink Robotics Baxter'
          ELSE INITCAP(REPLACE(robot_type, '_', ' '))
        END AS name,
        COUNT(m.id) AS module_count
      FROM (
        SELECT DISTINCT unnest(supported_robots) AS robot_type
        FROM modules 
        WHERE is_active = true 
        AND supported_robots IS NOT NULL 
        AND array_length(supported_robots, 1) > 0
      ) AS robot_types
      LEFT JOIN modules m ON robot_type = ANY(m.supported_robots) AND m.is_active = true
      GROUP BY robot_type
      ORDER BY module_count DESC, name ASC
      `
    )) as { rows: Array<{ code: string; name: string; module_count: number }> };

    res.json({ success: true, data: result.rows });
  } catch (error) {
    console.error('Error fetching robots:', error);
    res.status(500).json({ success: false, error: 'Failed to fetch robots' });
  }
});

export default router;
