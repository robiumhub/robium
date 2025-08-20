import express from 'express';
import { Database } from '../utils/database';
import { authMiddleware, AuthRequest } from '../middleware/auth';
// Import types directly to avoid TypeScript path issues
interface Project {
  id: string;
  name: string;
  description: string;
  ownerId: string;
  isActive: boolean;
  isTemplate: boolean;
  tags: string[];
  config: Record<string, any>;
  metadata: any;
  templateVisibility?: 'public' | 'private';
  templateVersion?: string;
  templatePublishedAt?: Date;
  createdAt: Date;
  updatedAt: Date;
}

interface ProjectFilters {
  useCases: string[];
  capabilities: string[];
  robots: string[];
  simulators: string[];
  difficulty: string[];
  tags: string[];
  searchQuery?: string;
}

interface FilterCategory {
  id: string;
  name: string;
  displayName: string;
  description?: string;
  type: 'string' | 'boolean' | 'number';
  isActive: boolean;
  sortOrder: number;
  createdAt: Date;
  updatedAt: Date;
}

interface FilterValue {
  id: string;
  categoryId: string;
  value: string;
  displayName: string;
  description?: string;
  isActive: boolean;
  sortOrder: number;
  createdAt: Date;
  updatedAt: Date;
}

const router = express.Router();

// GET /api/projects/templates - Get all templates
router.get('/templates', authMiddleware, async (req: AuthRequest, res) => {
  try {
    const db = Database.getDatabase();

    // Query templates from database - show all public templates and user's own templates
    const templates = await new Promise<any[]>((resolve, reject) => {
      db.all(
        `
        SELECT id, name, description, owner_id, is_active, is_template,
               tags, config, metadata, template_visibility, template_version,
               template_published_at, created_at, updated_at
        FROM projects
        WHERE is_template = 1 AND (template_visibility = 'public' OR owner_id = ?)
        ORDER BY created_at DESC
      `,
        [req.user?.id || '1'], // Use authenticated user's ID, fallback to '1' for testing
        (err, rows) => {
          if (err) reject(err);
          else resolve(rows || []);
        }
      );
    });

    // Transform the data to match our Project interface
    const transformedTemplates: Project[] = templates.map((row) => ({
      id: row.id,
      name: row.name,
      description: row.description || '',
      ownerId: row.owner_id,
      isActive: Boolean(row.is_active),
      isTemplate: Boolean(row.is_template),
      tags: JSON.parse(row.tags || '[]'),
      config: JSON.parse(row.config || '{}'),
      metadata: JSON.parse(row.metadata || '{}'),
      templateVisibility: row.template_visibility,
      templateVersion: row.template_version,
      templatePublishedAt: row.template_published_at
        ? new Date(row.template_published_at)
        : undefined,
      createdAt: new Date(row.created_at),
      updatedAt: new Date(row.updated_at),
    }));

    res.json({
      success: true,
      data: {
        projects: transformedTemplates,
      },
    });
  } catch (error) {
    console.error('Error fetching templates:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch templates',
    });
  }
});

// GET /api/projects - Get user's projects
router.get('/', authMiddleware, async (req: AuthRequest, res) => {
  try {
    const db = Database.getDatabase();

    // Query projects from database - only return projects owned by the authenticated user
    const projects = await new Promise<any[]>((resolve, reject) => {
      db.all(
        `
        SELECT id, name, description, owner_id, is_active, is_template,
               tags, config, metadata, created_at, updated_at
        FROM projects
        WHERE is_template = 0 AND owner_id = ?
        ORDER BY created_at DESC
      `,
        [req.user?.id || '1'], // Use authenticated user's ID, fallback to '1' for testing
        (err, rows) => {
          if (err) reject(err);
          else resolve(rows || []);
        }
      );
    });

    // Transform the data to match our Project interface
    const transformedProjects: Project[] = projects.map((row) => ({
      id: row.id,
      name: row.name,
      description: row.description || '',
      ownerId: row.owner_id,
      isActive: Boolean(row.is_active),
      isTemplate: Boolean(row.is_template),
      tags: JSON.parse(row.tags || '[]'),
      config: JSON.parse(row.config || '{}'),
      metadata: JSON.parse(row.metadata || '{}'),
      createdAt: new Date(row.created_at),
      updatedAt: new Date(row.updated_at),
    }));

    res.json({
      success: true,
      data: {
        projects: transformedProjects,
      },
    });
  } catch (error) {
    console.error('Error fetching projects:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch projects',
    });
  }
});

// GET /api/projects/:id - Get specific project
router.get('/:id', authMiddleware, async (req, res) => {
  try {
    const { id } = req.params;
    const db = Database.getDatabase();

    const project = await new Promise<any>((resolve, reject) => {
      db.get(
        `
        SELECT id, name, description, owner_id, is_active, is_template,
               tags, config, metadata, template_visibility, template_version,
               template_published_at, created_at, updated_at
        FROM projects
        WHERE id = ?
      `,
        [id],
        (err, row) => {
          if (err) reject(err);
          else resolve(row);
        }
      );
    });

    if (!project) {
      return res.status(404).json({
        success: false,
        error: 'Project not found',
      });
    }

    const transformedProject: Project = {
      id: project.id,
      name: project.name,
      description: project.description || '',
      ownerId: project.owner_id,
      isActive: Boolean(project.is_active),
      isTemplate: Boolean(project.is_template),
      tags: JSON.parse(project.tags || '[]'),
      config: JSON.parse(project.config || '{}'),
      metadata: JSON.parse(project.metadata || '{}'),
      templateVisibility: project.template_visibility,
      templateVersion: project.template_version,
      templatePublishedAt: project.template_published_at
        ? new Date(project.template_published_at)
        : undefined,
      createdAt: new Date(project.created_at),
      updatedAt: new Date(project.updated_at),
    };

    res.json({
      success: true,
      data: {
        project: transformedProject,
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

// GET /api/projects/filters/categories - Get filter categories
router.get('/filters/categories', async (req, res) => {
  try {
    const db = Database.getDatabase();

    const categories = await new Promise<any[]>((resolve, reject) => {
      db.all(
        `
        SELECT id, name, display_name, description, type, is_active, sort_order, created_at, updated_at
        FROM filter_categories
        WHERE is_active = 1
        ORDER BY sort_order ASC
      `,
        (err, rows) => {
          if (err) reject(err);
          else resolve(rows || []);
        }
      );
    });

    const transformedCategories: FilterCategory[] = categories.map((row) => ({
      id: row.id,
      name: row.name,
      displayName: row.display_name,
      description: row.description,
      type: row.type as 'string' | 'boolean' | 'number',
      isActive: Boolean(row.is_active),
      sortOrder: row.sort_order,
      createdAt: new Date(row.created_at),
      updatedAt: new Date(row.updated_at),
    }));

    res.json({
      success: true,
      data: {
        categories: transformedCategories,
      },
    });
  } catch (error) {
    console.error('Error fetching filter categories:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch filter categories',
    });
  }
});

// GET /api/projects/filters/values - Get filter values
router.get('/filters/values', async (req, res) => {
  try {
    const db = Database.getDatabase();

    const values = await new Promise<any[]>((resolve, reject) => {
      db.all(
        `
        SELECT id, category_id, value, display_name, description, is_active, sort_order, created_at, updated_at
        FROM filter_values
        WHERE is_active = 1
        ORDER BY sort_order ASC
      `,
        (err, rows) => {
          if (err) reject(err);
          else resolve(rows || []);
        }
      );
    });

    const transformedValues: FilterValue[] = values.map((row) => ({
      id: row.id,
      categoryId: row.category_id,
      value: row.value,
      displayName: row.display_name,
      description: row.description,
      isActive: Boolean(row.is_active),
      sortOrder: row.sort_order,
      createdAt: new Date(row.created_at),
      updatedAt: new Date(row.updated_at),
    }));

    res.json({
      success: true,
      data: {
        values: transformedValues,
      },
    });
  } catch (error) {
    console.error('Error fetching filter values:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch filter values',
    });
  }
});

// GET /api/projects/filters/stats - Get filter statistics
router.get('/filters/stats', authMiddleware, async (req: AuthRequest, res) => {
  try {
    const db = Database.getDatabase();
    const { isTemplate } = req.query;

    // Build the base query
    let baseQuery = `
      SELECT metadata
      FROM projects
      WHERE owner_id = ?
    `;
    const params = [req.user?.id || '1'];

    if (isTemplate === 'true') {
      baseQuery += ` AND is_template = 1 AND (template_visibility = 'public' OR owner_id = ?)`;
      params.push(req.user?.id || '1');
    } else {
      baseQuery += ` AND is_template = 0`;
    }

    const projects = await new Promise<any[]>((resolve, reject) => {
      db.all(baseQuery, params, (err, rows) => {
        if (err) reject(err);
        else resolve(rows || []);
      });
    });

    // Calculate statistics
    const stats: Record<string, Record<string, number>> = {
      useCases: {},
      capabilities: {},
      robots: {},
      simulators: {},
      difficulty: {},
      tags: {},
    };

    projects.forEach((project) => {
      const metadata = JSON.parse(project.metadata || '{}');
      const tags = JSON.parse(project.tags || '[]');

      // Count use cases
      if (metadata.useCases) {
        metadata.useCases.forEach((useCase: string) => {
          stats.useCases[useCase] = (stats.useCases[useCase] || 0) + 1;
        });
      }

      // Count capabilities
      if (metadata.capabilities) {
        metadata.capabilities.forEach((capability: string) => {
          stats.capabilities[capability] = (stats.capabilities[capability] || 0) + 1;
        });
      }

      // Count robots
      if (metadata.robots) {
        metadata.robots.forEach((robot: string) => {
          stats.robots[robot] = (stats.robots[robot] || 0) + 1;
        });
      }

      // Count simulators
      if (metadata.simulators) {
        metadata.simulators.forEach((simulator: string) => {
          stats.simulators[simulator] = (stats.simulators[simulator] || 0) + 1;
        });
      }

      // Count difficulty
      if (metadata.difficulty) {
        stats.difficulty[metadata.difficulty] = (stats.difficulty[metadata.difficulty] || 0) + 1;
      }

      // Count tags
      tags.forEach((tag: string) => {
        stats.tags[tag] = (stats.tags[tag] || 0) + 1;
      });
    });

    res.json({
      success: true,
      data: {
        stats,
      },
    });
  } catch (error) {
    console.error('Error fetching filter stats:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch filter stats',
    });
  }
});

export default router;
