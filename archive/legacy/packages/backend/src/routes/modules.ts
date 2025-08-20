import express from 'express';
import { Database } from '../utils/database';
import { AuthRequest } from '../types';

const router = express.Router();

// GET /api/modules - Get all modules
router.get('/', async (req: AuthRequest, res) => {
  try {
    const { category, type, search } = req.query;

    let query = `
      SELECT m.*, 
             m.supported_robots
      FROM modules m
      WHERE m.is_active = 1
    `;
    const params: string[] = [];
    let paramIndex = 1;

    // Add category filter
    if (category && typeof category === 'string') {
      query += ` AND m.category = ?`;
      params.push(category);
      paramIndex++;
    }

    // Add type filter
    if (type && typeof type === 'string') {
      query += ` AND m.type = ?`;
      params.push(type);
      paramIndex++;
    }

    // Add search filter
    if (search && typeof search === 'string') {
      query += ` AND (LOWER(m.name) LIKE ? OR LOWER(m.description) LIKE ?)`;
      params.push(`%${search.toLowerCase()}%`);
      params.push(`%${search.toLowerCase()}%`);
      paramIndex += 2;
    }

    query += ' ORDER BY m.name';

    const result = (await Database.query(query, params)) as {
      rows: Array<Record<string, any>>;
    };

    res.json({
      success: true,
      data: result.rows,
      count: result.rows.length,
    });
  } catch (error) {
    console.error('Error fetching modules:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch modules',
    });
  }
});

// GET /api/modules/categories - Get all categories
router.get('/categories', async (req: AuthRequest, res) => {
  try {
    const result = (await Database.query(
      'SELECT DISTINCT category FROM modules WHERE is_active = 1 AND category IS NOT NULL ORDER BY category'
    )) as { rows: Array<{ category: string }> };

    res.json({
      success: true,
      data: result.rows.map((row) => row.category),
    });
  } catch (error) {
    console.error('Error fetching module categories:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch module categories',
    });
  }
});

// GET /api/modules/types - Get all types
router.get('/types', async (req: AuthRequest, res) => {
  try {
    const result = (await Database.query(
      'SELECT DISTINCT type FROM modules WHERE is_active = 1 ORDER BY type'
    )) as { rows: Array<{ type: string }> };

    res.json({
      success: true,
      data: result.rows.map((row) => row.type),
    });
  } catch (error) {
    console.error('Error fetching module types:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch module types',
    });
  }
});

// GET /api/modules/search - Search modules
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
      SELECT rp.*, 
             COALESCE(rp.supported_robots, '{}'::text[]) AS supported_robots
      FROM ros_packages rp
      WHERE rp.is_active = true 
      AND (rp.name ILIKE $1 OR rp.description ILIKE $1 OR rp.tags::text ILIKE $1)
    `;
    const params: string[] = [`%${q}%`];
    let paramIndex = 2;

    // Add category filter
    if (category && typeof category === 'string') {
      query += ` AND rp.category = $${paramIndex}`;
      params.push(category);
      paramIndex++;
    }

    // Add type filter
    if (type && typeof type === 'string') {
      query += ` AND rp.type = $${paramIndex}`;
      params.push(type);
      paramIndex++;
    }

    query += ` ORDER BY rp.name LIMIT $${paramIndex}`;
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
    console.error('Error searching modules:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to search modules',
    });
  }
});

// GET /api/modules/:id - Get a specific module
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
        error: 'Module not found',
      });
    }

    res.json({
      success: true,
      data: result.rows[0],
    });
  } catch (error) {
    console.error('Error fetching module:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch module',
    });
  }
});

export default router;
