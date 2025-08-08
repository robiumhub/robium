import express from 'express';
import { Database } from '../utils/database';
import { AuthRequest } from '../types';
import { authenticateToken } from '../middleware/auth';
import crypto from 'crypto';

const router = express.Router();

// GET /api/projects - Get all projects
router.get('/', async (req: AuthRequest, res) => {
  try {
    const { type, search, is_template } = req.query;

    let query = `
      SELECT p.*, 
             COUNT(DISTINCT pmd.module_id) as module_count,
             COUNT(DISTINCT pp.package_id) as package_count
      FROM projects p
      LEFT JOIN project_module_dependencies pmd ON p.id = pmd.project_id
      LEFT JOIN project_packages pp ON p.id = pp.project_id
      WHERE p.is_active = true
    `;
    const params: string[] = [];
    let paramIndex = 1;

    // Add type filter
    if (type && typeof type === 'string') {
      query += ` AND p.type = $${paramIndex}`;
      params.push(type);
      paramIndex++;
    }

    // Add template filter
    if (is_template !== undefined) {
      query += ` AND p.is_template = $${paramIndex}`;
      params.push(is_template === 'true' ? 'true' : 'false');
      paramIndex++;
    }

    // Add search filter
    if (search && typeof search === 'string') {
      query += ` AND (p.name ILIKE $${paramIndex} OR p.description ILIKE $${paramIndex})`;
      params.push(`%${search}%`);
      paramIndex++;
    }

    query += ' GROUP BY p.id ORDER BY p.created_at DESC';

    const result = (await Database.query(query, params)) as {
      rows: Array<Record<string, any>>;
    };

    res.json({
      success: true,
      data: result.rows,
      count: result.rows.length,
    });
  } catch (error) {
    console.error('Error fetching projects:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch projects',
    });
  }
});

// GET /api/projects/:id - Get specific project with modules and packages
router.get('/:id', async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;

    // Get project details
    const projectResult = (await Database.query(
      'SELECT * FROM projects WHERE id = $1 AND is_active = true',
      [id]
    )) as { rows: Array<Record<string, any>> };

    if (projectResult.rows.length === 0) {
      return res.status(404).json({
        success: false,
        error: 'Project not found',
      });
    }

    const project = projectResult.rows[0];

    // Get modules for this project
    const modulesResult = (await Database.query(
      `
      SELECT m.*, pmd.dependency_type, pmd.order_index
      FROM project_module_dependencies pmd
      JOIN modules m ON pmd.module_id = m.id
      WHERE pmd.project_id = $1
      ORDER BY pmd.order_index
    `,
      [id]
    )) as { rows: Array<Record<string, any>> };

    // Get packages for this project
    const packagesResult = (await Database.query(
      `
      SELECT rp.*, pp.is_required, pp.order_index
      FROM project_packages pp
      JOIN ros_packages rp ON pp.package_id = rp.id
      WHERE pp.project_id = $1
      ORDER BY pp.order_index
    `,
      [id]
    )) as { rows: Array<Record<string, any>> };

    // Get files for this project
    const filesResult = (await Database.query(
      `
      SELECT * FROM project_files
      WHERE project_id = $1
      ORDER BY file_type, file_path
    `,
      [id]
    )) as { rows: Array<Record<string, any>> };

    res.json({
      success: true,
      data: {
        ...project,
        modules: modulesResult.rows,
        packages: packagesResult.rows,
        files: filesResult.rows,
      },
    });
  } catch (error) {
    console.error('Error fetching project:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch project',
    });
  }
});

// GET /api/projects/categories - Get all categories
router.get('/categories', async (req: AuthRequest, res) => {
  try {
    const result = (await Database.query(
      'SELECT DISTINCT category FROM projects WHERE is_active = true AND category IS NOT NULL ORDER BY category'
    )) as { rows: Array<{ category: string }> };

    res.json({
      success: true,
      data: result.rows.map((row) => row.category),
    });
  } catch (error) {
    console.error('Error fetching project categories:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch project categories',
    });
  }
});

// GET /api/projects/types - Get all types
router.get('/types', async (req: AuthRequest, res) => {
  try {
    const result = (await Database.query(
      'SELECT DISTINCT type FROM projects WHERE is_active = true ORDER BY type'
    )) as { rows: Array<{ type: string }> };

    res.json({
      success: true,
      data: result.rows.map((row) => row.type),
    });
  } catch (error) {
    console.error('Error fetching project types:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch project types',
    });
  }
});

// GET /api/projects/templates - Get template projects
router.get('/templates', async (req: AuthRequest, res) => {
  try {
    const { category, search } = req.query;

    let query = `
      SELECT p.*, 
             COUNT(DISTINCT pmd.module_id) as module_count,
             COUNT(DISTINCT pp.package_id) as package_count
      FROM projects p
      LEFT JOIN project_module_dependencies pmd ON p.id = pmd.project_id
      LEFT JOIN project_packages pp ON p.id = pp.project_id
      WHERE p.is_active = true AND p.is_template = true
    `;
    const params: string[] = [];
    let paramIndex = 1;

    // Add category filter
    if (category && typeof category === 'string') {
      query += ` AND p.category = $${paramIndex}`;
      params.push(category);
      paramIndex++;
    }

    // Add search filter
    if (search && typeof search === 'string') {
      query += ` AND (p.name ILIKE $${paramIndex} OR p.description ILIKE $${paramIndex})`;
      params.push(`%${search}%`);
      paramIndex++;
    }

    query += ' GROUP BY p.id ORDER BY p.name';

    const result = (await Database.query(query, params)) as {
      rows: Array<Record<string, any>>;
    };

    res.json({
      success: true,
      data: result.rows,
      count: result.rows.length,
    });
  } catch (error) {
    console.error('Error fetching template projects:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch template projects',
    });
  }
});

// GET /api/projects/search - Search projects
router.get('/search', async (req: AuthRequest, res) => {
  try {
    const { q, type, limit = '10' } = req.query;

    if (!q) {
      return res.status(400).json({
        success: false,
        error: 'Search query is required',
      });
    }

    let query = `
      SELECT p.*, 
             COUNT(DISTINCT pmd.module_id) as module_count,
             COUNT(DISTINCT pp.package_id) as package_count
      FROM projects p
      LEFT JOIN project_module_dependencies pmd ON p.id = pmd.project_id
      LEFT JOIN project_packages pp ON p.id = pp.project_id
      WHERE p.is_active = true 
      AND (p.name ILIKE $1 OR p.description ILIKE $1 OR p.metadata::text ILIKE $1)
    `;
    const params: string[] = [`%${q}%`];
    let paramIndex = 2;

    // Add type filter
    if (type && typeof type === 'string') {
      query += ` AND p.type = $${paramIndex}`;
      params.push(type);
      paramIndex++;
    }

    query += ` GROUP BY p.id ORDER BY p.name LIMIT $${paramIndex}`;
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
    console.error('Error searching projects:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to search projects',
    });
  }
});

// GET /api/projects/:id/modules - Get modules for a specific project
router.get('/:id/modules', async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;

    const result = (await Database.query(
      `
      SELECT m.*, pmd.dependency_type, pmd.order_index
      FROM project_module_dependencies pmd
      JOIN modules m ON pmd.module_id = m.id
      WHERE pmd.project_id = $1
      ORDER BY pmd.order_index
    `,
      [id]
    )) as { rows: Array<Record<string, any>> };

    res.json({
      success: true,
      data: result.rows,
      count: result.rows.length,
    });
  } catch (error) {
    console.error('Error fetching project modules:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch project modules',
    });
  }
});

// GET /api/projects/:id/packages - Get packages for a specific project
router.get('/:id/packages', async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;

    const result = (await Database.query(
      `
      SELECT rp.*, pp.is_required, pp.order_index
      FROM project_packages pp
      JOIN ros_packages rp ON pp.package_id = rp.id
      WHERE pp.project_id = $1
      ORDER BY pp.order_index
    `,
      [id]
    )) as { rows: Array<Record<string, any>> };

    res.json({
      success: true,
      data: result.rows,
      count: result.rows.length,
    });
  } catch (error) {
    console.error('Error fetching project packages:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch project packages',
    });
  }
});

// GET /api/projects/:id/files - Get files for a specific project
router.get('/:id/files', async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;

    const result = (await Database.query(
      `
      SELECT * FROM project_files
      WHERE project_id = $1
      ORDER BY file_type, file_path
    `,
      [id]
    )) as { rows: Array<Record<string, any>> };

    res.json({
      success: true,
      data: result.rows,
      count: result.rows.length,
    });
  } catch (error) {
    console.error('Error fetching project files:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch project files',
    });
  }
});

// POST /api/projects - Create a new project
router.post('/', authenticateToken, async (req: AuthRequest, res) => {
  try {
    const userId = req.user?.userId;
    const {
      name,
      description,
      tags = [],
      algorithms = [],
      environment = {},
      settings = {},
    } = req.body;

    // Validate required fields
    if (!name || !description) {
      return res.status(400).json({
        success: false,
        error: 'Name and description are required',
      });
    }

    // Generate a unique ID for the project
    const projectId = crypto.randomUUID();

    // Create the project with basic fields first
    const result = (await Database.query(
      `
      INSERT INTO projects (
        id, name, description, owner_id
      ) VALUES (
        $1, $2, $3, $4
      ) RETURNING *
    `,
      [
        projectId,
        name,
        description,
        userId, // owner_id
      ]
    )) as { rows: Array<Record<string, any>> };

    const newProject = result.rows[0];

    res.status(201).json({
      success: true,
      data: newProject,
      message: 'Project created successfully',
    });
  } catch (error) {
    console.error('Error creating project:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to create project',
    });
  }
});

// DELETE /api/projects/:id - Delete a project
router.delete('/:id', authenticateToken, async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;
    const userId = req.user?.userId;

    // Check if project exists and user has permission
    const projectResult = (await Database.query(
      'SELECT * FROM projects WHERE id = $1 AND is_active = true',
      [id]
    )) as { rows: Array<Record<string, any>> };

    if (projectResult.rows.length === 0) {
      return res.status(404).json({
        success: false,
        error: 'Project not found',
      });
    }

    const project = projectResult.rows[0];

    // Check if user is admin or the project owner
    // Allow deletion if user is admin, project owner, or if project has no owner (created_by is null)
    if (
      req.user?.role !== 'admin' &&
      project.created_by !== null &&
      project.created_by !== userId
    ) {
      return res.status(403).json({
        success: false,
        error: 'You do not have permission to delete this project',
      });
    }

    // Soft delete the project by setting is_active to false
    await Database.query(
      'UPDATE projects SET is_active = false, updated_at = NOW(), updated_by = $1 WHERE id = $2',
      [userId, id]
    );

    // Also delete related records (optional - you might want to keep them for audit)
    await Database.query(
      'DELETE FROM project_module_dependencies WHERE project_id = $1',
      [id]
    );
    await Database.query('DELETE FROM project_packages WHERE project_id = $1', [
      id,
    ]);
    await Database.query('DELETE FROM project_files WHERE project_id = $1', [
      id,
    ]);

    res.json({
      success: true,
      message: 'Project deleted successfully',
    });
  } catch (error) {
    console.error('Error deleting project:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to delete project',
    });
  }
});

export default router;
