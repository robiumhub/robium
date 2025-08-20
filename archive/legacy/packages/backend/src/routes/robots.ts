import express from 'express';
import { Database } from '../utils/database';
import { AuthRequest } from '../types';

const router = express.Router();

// GET /robots - List supported robots with module counts
router.get('/', async (req: AuthRequest, res) => {
  try {
    // For SQLite, we'll return a simplified list of supported robots
    // since SQLite doesn't have the same array functions as PostgreSQL
    const robots = [
      { code: 'turtlebot3', name: 'TurtleBot 3', module_count: 5 },
      { code: 'pioneer3at', name: 'Pioneer 3-AT', module_count: 3 },
      { code: 'kobuki', name: 'Kobuki Base', module_count: 4 },
      { code: 'create3', name: 'iRobot Create 3', module_count: 2 },
      { code: 'jackal', name: 'Clearpath Jackal', module_count: 3 },
      { code: 'husky', name: 'Clearpath Husky', module_count: 3 },
      { code: 'clearpath_robots', name: 'Clearpath Robots', module_count: 6 },
      { code: 'pr2', name: 'Willow Garage PR2', module_count: 2 },
      { code: 'fetch', name: 'Fetch Robotics', module_count: 2 },
      { code: 'baxter', name: 'Rethink Robotics Baxter', module_count: 1 },
    ];

    res.json({ success: true, data: robots });
  } catch (error) {
    console.error('Error fetching robots:', error);
    res.status(500).json({ success: false, error: 'Failed to fetch robots' });
  }
});

export default router;
