import express from 'express';
import { Database } from '../utils/database';
import { AuthRequest } from '../types';

const router = express.Router();

// GET /api/ros-packages - Get all ROS packages
router.get('/', async (req: AuthRequest, res) => {
  try {
    const { category, type, search } = req.query;

    let query = 'SELECT * FROM ros_packages WHERE is_active = true';
    const params: string[] = [];
    let paramIndex = 1;

    // Add category filter
    if (category && typeof category === 'string') {
      query += ` AND category = $${paramIndex}`;
      params.push(category);
      paramIndex++;
    }

    // Add type filter
    if (type && typeof type === 'string') {
      query += ` AND type = $${paramIndex}`;
      params.push(type);
      paramIndex++;
    }

    // Add search filter
    if (search && typeof search === 'string') {
      query += ` AND (name ILIKE $${paramIndex} OR description ILIKE $${paramIndex})`;
      params.push(`%${search}%`);
      paramIndex++;
    }

    query += ' ORDER BY name';

    const result = (await Database.query(query, params)) as {
      rows: Array<Record<string, any>>;
    };

    res.json({
      success: true,
      data: result.rows,
      count: result.rows.length,
    });
  } catch (error) {
    console.error('Error fetching ROS packages:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch ROS packages',
    });
  }
});

// GET /api/ros-packages/categories - Get all categories
router.get('/categories', async (req: AuthRequest, res) => {
  try {
    const result = (await Database.query(
      'SELECT DISTINCT category FROM ros_packages WHERE is_active = true AND category IS NOT NULL ORDER BY category'
    )) as { rows: Array<{ category: string }> };

    res.json({
      success: true,
      data: result.rows.map((row) => row.category),
    });
  } catch (error) {
    console.error('Error fetching ROS package categories:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch ROS package categories',
    });
  }
});

// GET /api/ros-packages/types - Get all types
router.get('/types', async (req: AuthRequest, res) => {
  try {
    const result = (await Database.query(
      'SELECT DISTINCT type FROM ros_packages WHERE is_active = true ORDER BY type'
    )) as { rows: Array<{ type: string }> };

    res.json({
      success: true,
      data: result.rows.map((row) => row.type),
    });
  } catch (error) {
    console.error('Error fetching ROS package types:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch ROS package types',
    });
  }
});

// GET /api/ros-packages/search - Search ROS packages
router.get('/search', async (req: AuthRequest, res) => {
  try {
    const { q, category, type, limit = '10' } = req.query;

    if (!q) {
      return res.status(400).json({
        success: false,
        error: 'Search query is required',
      });
    }

    let query = `
      SELECT * FROM ros_packages 
      WHERE is_active = true 
      AND (name ILIKE $1 OR description ILIKE $1 OR tags::text ILIKE $1)
    `;
    const params: string[] = [`%${q}%`];
    let paramIndex = 2;

    // Add category filter
    if (category && typeof category === 'string') {
      query += ` AND category = $${paramIndex}`;
      params.push(category);
      paramIndex++;
    }

    // Add type filter
    if (type && typeof type === 'string') {
      query += ` AND type = $${paramIndex}`;
      params.push(type);
      paramIndex++;
    }

    query += ` ORDER BY name LIMIT $${paramIndex}`;
    params.push(parseInt(limit as string).toString());

    const result = (await Database.query(query, params)) as {
      rows: Array<Record<string, any>>;
    };

    res.json({
      success: true,
      data: result.rows,
      count: result.rows.length,
      query: q,
    });
  } catch (error) {
    console.error('Error searching ROS packages:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to search ROS packages',
    });
  }
});

// GET /api/ros-packages/:id - Get specific ROS package (must be last)
router.get('/:id', async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;

    const result = (await Database.query(
      'SELECT * FROM ros_packages WHERE id = $1 AND is_active = true',
      [id]
    )) as { rows: Array<Record<string, any>> };

    if (result.rows.length === 0) {
      return res.status(404).json({
        success: false,
        error: 'ROS package not found',
      });
    }

    res.json({
      success: true,
      data: result.rows[0],
    });
  } catch (error) {
    console.error('Error fetching ROS package:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch ROS package',
    });
  }
});

export default router;
