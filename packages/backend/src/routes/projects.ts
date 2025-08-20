import { Router } from 'express';
import { authMiddleware, adminMiddleware, AuthRequest } from '../middleware/auth';
import { Database } from '../utils/database';

const router = Router();

// GET /api/projects
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

    // Parse JSON fields
    const formattedProjects = projects.map((project: any) => ({
      id: project.id,
      name: project.name,
      description: project.description,
      ownerId: project.owner_id,
      isActive: Boolean(project.is_active),
      isTemplate: Boolean(project.is_template),
      tags: JSON.parse(project.tags || '[]'),
      config: JSON.parse(project.config || '{}'),
      metadata: JSON.parse(project.metadata || '{}'),
      createdAt: project.created_at,
      updatedAt: project.updated_at,
    }));

    res.json({
      success: true,
      data: {
        projects: formattedProjects,
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

// POST /api/projects
router.post('/', authMiddleware, async (req: AuthRequest, res) => {
  try {
    // TODO: Implement project creation
    res.json({
      success: true,
      data: {
        project: {
          id: '2',
          name: req.body.name,
          description: req.body.description,
          ownerId: req.user?.id,
          isActive: true,
          isTemplate: false,
          tags: [],
          config: req.body.config,
          metadata: {
            useCases: [],
            capabilities: [],
            robots: [],
            simulators: [],
          },
          createdAt: new Date(),
          updatedAt: new Date(),
        },
      },
    });
  } catch (error) {
    res.status(500).json({
      success: false,
      error: 'Failed to create project',
    });
  }
});

// GET /api/projects/templates
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

    // Parse JSON fields
    const formattedTemplates = templates.map((template: any) => ({
      id: template.id,
      name: template.name,
      description: template.description,
      ownerId: template.owner_id,
      isActive: Boolean(template.is_active),
      isTemplate: Boolean(template.is_template),
      tags: JSON.parse(template.tags || '[]'),
      config: JSON.parse(template.config || '{}'),
      metadata: JSON.parse(template.metadata || '{}'),
      templateVisibility: template.template_visibility,
      templateVersion: template.template_version,
      templatePublishedAt: template.template_published_at,
      createdAt: template.created_at,
      updatedAt: template.updated_at,
    }));

    res.json({
      success: true,
      data: {
        projects: formattedTemplates,
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

// GET /api/projects/:id
router.get('/:id', authMiddleware, async (req, res) => {
  try {
    // TODO: Implement project retrieval
    res.json({
      success: true,
      data: {
        project: {
          id: req.params.id,
          name: 'Sample Project',
          description: 'A sample robotics project',
          ownerId: '1',
          isActive: true,
          isTemplate: false,
          tags: ['sample', 'robotics'],
          config: {},
          metadata: {
            useCases: ['navigation'],
            capabilities: ['mapping'],
            robots: ['turtlebot'],
            simulators: ['gazebo'],
          },
          createdAt: new Date(),
          updatedAt: new Date(),
        },
      },
    });
  } catch (error) {
    res.status(500).json({
      success: false,
      error: 'Failed to fetch project',
    });
  }
});

// PUT /api/projects/:id/settings
router.put('/:id/settings', authMiddleware, async (req, res) => {
  try {
    // TODO: Implement project settings update
    res.json({
      success: true,
      data: {
        project: {
          id: req.params.id,
          name: 'Updated Project',
          config: req.body.config,
          metadata: req.body.metadata,
          updatedAt: new Date(),
        },
      },
    });
  } catch (error) {
    res.status(500).json({
      success: false,
      error: 'Failed to update project settings',
    });
  }
});

// POST /api/projects/:id/clone
router.post('/:id/clone', authMiddleware, async (req: AuthRequest, res) => {
  try {
    // TODO: Implement project cloning
    res.json({
      success: true,
      data: {
        clonedProject: {
          id: '3',
          name: req.body.name || 'Cloned Project',
          ownerId: req.user?.id,
          isActive: true,
          isTemplate: false,
          tags: [],
          config: {},
          metadata: {
            useCases: [],
            capabilities: [],
            robots: [],
            simulators: [],
          },
          createdAt: new Date(),
          updatedAt: new Date(),
        },
      },
    });
  } catch (error) {
    res.status(500).json({
      success: false,
      error: 'Failed to clone project',
    });
  }
});

// POST /api/projects/:id/convert-to-template
router.post('/:id/convert-to-template', adminMiddleware, async (req, res) => {
  try {
    // TODO: Implement convert to template
    res.json({
      success: true,
      data: {
        project: {
          id: req.params.id,
          isTemplate: true,
          templateVisibility: req.body.visibility || 'public',
          templateVersion: req.body.version || '1.0.0',
          templatePublishedAt: new Date(),
          updatedAt: new Date(),
        },
      },
    });
  } catch (error) {
    res.status(500).json({
      success: false,
      error: 'Failed to convert project to template',
    });
  }
});

// DELETE /api/projects/:id
router.delete('/:id', authMiddleware, async (req, res) => {
  try {
    // TODO: Implement project deletion
    res.json({
      success: true,
      message: 'Project deleted successfully',
    });
  } catch (error) {
    res.status(500).json({
      success: false,
      error: 'Failed to delete project',
    });
  }
});

// GET /api/projects/facets
router.get('/facets', authMiddleware, async (req, res) => {
  try {
    // TODO: Implement facets calculation
    res.json({
      success: true,
      data: {
        useCaseCounts: { navigation: 5, manipulation: 3 },
        capabilityCounts: { mapping: 4, planning: 2 },
        robotCounts: { turtlebot: 6, ur5: 2 },
        simulatorCounts: { gazebo: 7, rviz: 1 },
        tagCounts: { sample: 3, robotics: 8 },
      },
    });
  } catch (error) {
    res.status(500).json({
      success: false,
      error: 'Failed to fetch facets',
    });
  }
});

export default router;
