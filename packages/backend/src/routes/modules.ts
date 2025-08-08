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
             COUNT(DISTINCT mp.package_id) as package_count,
             COUNT(DISTINCT md.dependency_module_id) as dependency_count
      FROM modules m
      LEFT JOIN module_packages mp ON m.id = mp.module_id
      LEFT JOIN module_dependencies md ON m.id = md.module_id
      WHERE m.is_active = true
    `;
    const params: string[] = [];
    let paramIndex = 1;

    // Add category filter
    if (category && typeof category === 'string') {
      query += ` AND m.category = $${paramIndex}`;
      params.push(category);
      paramIndex++;
    }

    // Add type filter
    if (type && typeof type === 'string') {
      query += ` AND m.type = $${paramIndex}`;
      params.push(type);
      paramIndex++;
    }

    // Add search filter
    if (search && typeof search === 'string') {
      query += ` AND (m.name ILIKE $${paramIndex} OR m.description ILIKE $${paramIndex})`;
      params.push(`%${search}%`);
      paramIndex++;
    }

    query += ' GROUP BY m.id ORDER BY m.name';

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
      'SELECT DISTINCT category FROM modules WHERE is_active = true AND category IS NOT NULL ORDER BY category'
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
      'SELECT DISTINCT type FROM modules WHERE is_active = true ORDER BY type'
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
      SELECT m.*, 
             COUNT(DISTINCT mp.package_id) as package_count,
             COUNT(DISTINCT md.dependency_module_id) as dependency_count
      FROM modules m
      LEFT JOIN module_packages mp ON m.id = mp.module_id
      LEFT JOIN module_dependencies md ON m.id = md.module_id
      WHERE m.is_active = true 
      AND (m.name ILIKE $1 OR m.description ILIKE $1 OR m.tags::text ILIKE $1)
    `;
    const params: string[] = [`%${q}%`];
    let paramIndex = 2;

    // Add category filter
    if (category && typeof category === 'string') {
      query += ` AND m.category = $${paramIndex}`;
      params.push(category);
      paramIndex++;
    }

    // Add type filter
    if (type && typeof type === 'string') {
      query += ` AND m.type = $${paramIndex}`;
      params.push(type);
      paramIndex++;
    }

    query += ` GROUP BY m.id ORDER BY m.name LIMIT $${paramIndex}`;
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

// GET /api/modules/:id/packages - Get packages for a specific module
router.get('/:id/packages', async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;

    const result = (await Database.query(
      `
      SELECT rp.*, mp.is_required, mp.order_index
      FROM module_packages mp
      JOIN ros_packages rp ON mp.package_id = rp.id
      WHERE mp.module_id = $1
      ORDER BY mp.order_index
    `,
      [id]
    )) as { rows: Array<Record<string, any>> };

    res.json({
      success: true,
      data: result.rows,
      count: result.rows.length,
    });
  } catch (error) {
    console.error('Error fetching module packages:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch module packages',
    });
  }
});

// GET /api/modules/:id/dependencies - Get dependencies for a specific module
router.get('/:id/dependencies', async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;

    const result = (await Database.query(
      `
      SELECT m.*, md.dependency_type
      FROM module_dependencies md
      JOIN modules m ON md.dependency_module_id = m.id
      WHERE md.module_id = $1
      ORDER BY m.name
    `,
      [id]
    )) as { rows: Array<Record<string, any>> };

    res.json({
      success: true,
      data: result.rows,
      count: result.rows.length,
    });
  } catch (error) {
    console.error('Error fetching module dependencies:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch module dependencies',
    });
  }
});

// GET /api/modules/:id - Get specific module with packages and dependencies (must be last)
router.get('/:id', async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;

    // Get module details
    const moduleResult = (await Database.query(
      'SELECT * FROM modules WHERE id = $1 AND is_active = true',
      [id]
    )) as { rows: Array<Record<string, any>> };

    if (moduleResult.rows.length === 0) {
      return res.status(404).json({
        success: false,
        error: 'Module not found',
      });
    }

    const module = moduleResult.rows[0];

    // Get packages for this module
    const packagesResult = (await Database.query(
      `
      SELECT rp.*, mp.is_required, mp.order_index
      FROM module_packages mp
      JOIN ros_packages rp ON mp.package_id = rp.id
      WHERE mp.module_id = $1
      ORDER BY mp.order_index
    `,
      [id]
    )) as { rows: Array<Record<string, any>> };

    // Get dependencies for this module
    const dependenciesResult = (await Database.query(
      `
      SELECT m.*, md.dependency_type
      FROM module_dependencies md
      JOIN modules m ON md.dependency_module_id = m.id
      WHERE md.module_id = $1
      ORDER BY m.name
    `,
      [id]
    )) as { rows: Array<Record<string, any>> };

    // Get modules that depend on this module
    const dependentsResult = (await Database.query(
      `
      SELECT m.*, md.dependency_type
      FROM module_dependencies md
      JOIN modules m ON md.module_id = m.id
      WHERE md.dependency_module_id = $1
      ORDER BY m.name
    `,
      [id]
    )) as { rows: Array<Record<string, any>> };

    res.json({
      success: true,
      data: {
        ...module,
        packages: packagesResult.rows,
        dependencies: dependenciesResult.rows,
        dependents: dependentsResult.rows,
      },
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
