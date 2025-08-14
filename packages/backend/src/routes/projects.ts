import express from 'express';
import { Database } from '../utils/database';
import { AuthRequest } from '../types';
import { authenticateToken } from '../middleware/auth';
import crypto from 'crypto';
import { dockerfileGenerationService } from '../services/DockerfileGenerationService';
import { ROSProjectConfig } from '@robium/shared';

const router = express.Router();

// GET /api/projects - Get all projects
router.get('/', async (req: AuthRequest, res) => {
  try {
    const { type, search, is_template } = req.query;

    let query = `
      SELECT p.*, 
             COALESCE(p.tags, '{}'::text[]) AS tags,
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

// NOTE: Specific routes must be defined before parameterized ':id' route

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

// GET /api/projects/:id/settings - Get project configuration/settings
router.get('/:id/settings', async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;

    const result = (await Database.query(
      `SELECT id, name, config, metadata FROM projects WHERE id = $1 AND is_active = true`,
      [id]
    )) as { rows: Array<Record<string, any>> };

    if (result.rows.length === 0) {
      return res
        .status(404)
        .json({ success: false, error: 'Project not found' });
    }

    res.json({
      success: true,
      data: {
        config: result.rows[0].config || {},
        metadata: result.rows[0].metadata || {},
      },
    });
  } catch (error) {
    console.error('Error fetching project settings:', error);
    res
      .status(500)
      .json({ success: false, error: 'Failed to fetch project settings' });
  }
});

// PUT /api/projects/:id/settings - Update project configuration/settings
router.put(
  '/:id/settings',
  authenticateToken,
  async (req: AuthRequest, res) => {
    try {
      const { id } = req.params;
      const { config = {}, metadata = {} } = req.body || {};

      const result = (await Database.query(
        `UPDATE projects SET config = $1, metadata = $2, updated_at = NOW(), updated_by = $3 WHERE id = $4 AND is_active = true RETURNING id, name, config, metadata`,
        [config, metadata, req.user?.userId ?? null, id]
      )) as { rows: Array<Record<string, any>> };

      if (result.rows.length === 0) {
        return res
          .status(404)
          .json({ success: false, error: 'Project not found' });
      }

      res.json({
        success: true,
        data: result.rows[0],
        message: 'Project settings updated',
      });
    } catch (error) {
      console.error('Error updating project settings:', error);
      res
        .status(500)
        .json({ success: false, error: 'Failed to update project settings' });
    }
  }
);

// POST /api/projects/:id/clone - Clone an existing project
router.post('/:id/clone', authenticateToken, async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;
    const userId = req.user?.userId;

    // Load source project
    const source = (await Database.query(
      `SELECT * FROM projects WHERE id = $1 AND is_active = true`,
      [id]
    )) as { rows: Array<Record<string, any>> };

    if (source.rows.length === 0) {
      return res
        .status(404)
        .json({ success: false, error: 'Source project not found' });
    }

    const src = source.rows[0];
    const cloneId = crypto.randomUUID();
    const cloneName = `${src.name}-copy`;

    await Database.transaction(async (client: any) => {
      // Insert cloned project
      await client.query(
        `INSERT INTO projects (id, name, description, owner_id, tags, version, author, maintainer_email, license, type, is_active, is_template, config, metadata, workspace_path, source_path, config_path, created_by)
         VALUES ($1,$2,$3,$4,$5,$6,$7,$8,$9,$10,true,false,$11,$12,NULL,NULL,NULL,$13)`,
        [
          cloneId,
          cloneName,
          src.description,
          userId,
          src.tags || [],
          src.version || '1.0.0',
          src.author || null,
          src.maintainer_email || null,
          src.license || 'Apache-2.0',
          'custom',
          src.config || {},
          src.metadata || {},
          userId,
        ]
      );

      // Copy module dependencies
      await client.query(
        `INSERT INTO project_module_dependencies (project_id, module_id, dependency_type, version_constraint, order_index)
         SELECT $1, module_id, dependency_type, version_constraint, order_index FROM project_module_dependencies WHERE project_id = $2`,
        [cloneId, id]
      );

      // Copy project packages
      await client.query(
        `INSERT INTO project_packages (project_id, package_id, is_required, order_index)
         SELECT $1, package_id, is_required, order_index FROM project_packages WHERE project_id = $2`,
        [cloneId, id]
      );

      // Copy project files
      await client.query(
        `INSERT INTO project_files (project_id, file_path, file_type, content, content_hash, is_generated)
         SELECT $1, file_path, file_type, content, content_hash, is_generated FROM project_files WHERE project_id = $2`,
        [cloneId, id]
      );
    });

    const cloned = (await Database.query(
      `SELECT * FROM projects WHERE id = $1`,
      [cloneId]
    )) as {
      rows: Array<Record<string, any>>;
    };

    res
      .status(201)
      .json({ success: true, data: cloned.rows[0], message: 'Project cloned' });
  } catch (error) {
    console.error('Error cloning project:', error);
    res.status(500).json({ success: false, error: 'Failed to clone project' });
  }
});

// GET /api/projects/:id - Get specific project with modules and packages
router.get('/:id', async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;

    // Get project details
    const projectResult = (await Database.query(
      `SELECT p.*, COALESCE(p.tags, '{}'::text[]) AS tags 
       FROM projects p 
       WHERE p.id = $1 AND p.is_active = true`,
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

// ROS-specific endpoints

// GET /api/projects/ros/distros - Get available ROS distributions
router.get('/ros/distros', async (req: AuthRequest, res) => {
  try {
    const distros = dockerfileGenerationService.getROSDistros();
    res.json({
      success: true,
      data: distros,
    });
  } catch (error) {
    console.error('Error fetching ROS distributions:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch ROS distributions',
    });
  }
});

// GET /api/projects/ros/robots - Get available robots for a distribution
router.get('/ros/robots', async (req: AuthRequest, res) => {
  try {
    const { distro } = req.query;

    if (!distro || typeof distro !== 'string') {
      return res.status(400).json({
        success: false,
        error: 'Distribution parameter is required',
      });
    }

    const robots = dockerfileGenerationService.getRobotsByDistro(distro);
    res.json({
      success: true,
      data: robots,
    });
  } catch (error) {
    console.error('Error fetching ROS robots:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch ROS robots',
    });
  }
});

// GET /api/projects/ros/modules - Get compatible modules for a robot
router.get('/ros/modules', async (req: AuthRequest, res) => {
  try {
    const { robot } = req.query;

    if (!robot || typeof robot !== 'string') {
      return res.status(400).json({
        success: false,
        error: 'Robot parameter is required',
      });
    }

    const modules = dockerfileGenerationService.getCompatibleModules(robot);
    res.json({
      success: true,
      data: modules,
    });
  } catch (error) {
    console.error('Error fetching ROS modules:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch ROS modules',
    });
  }
});

// POST /api/projects/ros/generate - Generate ROS Dockerfiles
router.post(
  '/ros/generate',
  authenticateToken,
  async (req: AuthRequest, res) => {
    try {
      const config: ROSProjectConfig = req.body;

      // Validate required fields
      if (!config.distro || !config.robot) {
        return res.status(400).json({
          success: false,
          error: 'Distribution and robot are required',
        });
      }

      const result = await dockerfileGenerationService.generateROSDockerfiles(
        config,
        {
          includeCompose: true,
          includeBake: true,
          validateOnly: false,
        }
      );

      if (result.errors.length > 0) {
        return res.status(400).json({
          success: false,
          error: 'Failed to generate ROS Dockerfiles',
          errors: result.errors,
        });
      }

      res.json({
        success: true,
        data: {
          dockerfile: result.content,
          compose: result.composeContent,
          bake: result.bakeContent,
          paths: {
            dockerfile: result.path,
            compose: result.composePath,
            bake: result.bakePath,
          },
        },
        message: 'ROS Dockerfiles generated successfully',
      });
    } catch (error) {
      console.error('Error generating ROS Dockerfiles:', error);
      res.status(500).json({
        success: false,
        error: 'Failed to generate ROS Dockerfiles',
      });
    }
  }
);

// POST /api/projects/ros/preview - Preview ROS Dockerfiles without saving
router.post(
  '/ros/preview',
  authenticateToken,
  async (req: AuthRequest, res) => {
    try {
      const config: ROSProjectConfig = req.body;

      // Validate required fields
      if (!config.distro || !config.robot) {
        return res.status(400).json({
          success: false,
          error: 'Distribution and robot are required',
        });
      }

      const result = await dockerfileGenerationService.generateROSDockerfiles(
        config,
        {
          includeCompose: true,
          includeBake: true,
          validateOnly: true,
        }
      );

      res.json({
        success: true,
        data: {
          dockerfile: result.content,
          compose: result.composeContent,
          bake: result.bakeContent,
          warnings: result.warnings,
          optimizationSuggestions: result.optimizationSuggestions,
        },
      });
    } catch (error) {
      console.error('Error previewing ROS Dockerfiles:', error);
      res.status(500).json({
        success: false,
        error: 'Failed to preview ROS Dockerfiles',
      });
    }
  }
);

// POST /api/projects - Create a new project
router.post('/', authenticateToken, async (req: AuthRequest, res) => {
  try {
    const userId = req.user?.userId;
    const {
      name,
      description,
      tags = [],
      algorithms = [],
      is_template = false,
      rosConfig = null,
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
        id, name, description, owner_id, tags, is_template, type
      ) VALUES (
        $1, $2, $3, $4, $5, $6, $7
      ) RETURNING *
    `,
      [
        projectId,
        name,
        description,
        userId, // owner_id
        Array.isArray(tags) ? tags : [],
        Boolean(is_template),
        Boolean(is_template) ? 'template' : 'custom',
      ]
    )) as { rows: Array<Record<string, any>> };

    const newProject = result.rows[0];

    // Persist selected modules into project_module_dependencies
    if (Array.isArray(algorithms) && algorithms.length > 0) {
      const values: string[] = [];
      const placeholders: string[] = [];
      let idx = 1;
      for (const moduleId of algorithms) {
        placeholders.push(
          `($${idx++}, $${idx++}, 'required', NULL, ${idx++ - 1})`
        );
        // project_id, module_id, dependency_type 'required', version_constraint NULL, order_index
        values.push(newProject.id, moduleId, String(placeholders.length - 1));
      }

      // Build a deterministic order_index sequence 0..n-1
      // We'll rebuild placeholders accordingly
      const rows: string[] = [];
      let param = 1;
      for (let orderIndex = 0; orderIndex < algorithms.length; orderIndex++) {
        rows.push(`($${param++}, $${param++}, 'required', NULL, $${param++})`);
      }

      const insertParams: any[] = [];
      for (let i = 0; i < algorithms.length; i++) {
        insertParams.push(newProject.id, algorithms[i], i);
      }

      await Database.query(
        `INSERT INTO project_module_dependencies (project_id, module_id, dependency_type, version_constraint, order_index)
         VALUES ${rows.join(', ')}`,
        insertParams
      );
    }

    // Handle ROS configuration if provided
    if (rosConfig && typeof rosConfig === 'object') {
      try {
        // Generate ROS Dockerfiles
        const rosResult =
          await dockerfileGenerationService.generateROSDockerfiles(rosConfig, {
            includeCompose: true,
            includeBake: true,
            validateOnly: false,
            outputDir: `./generated/${projectId}`,
          });

        if (rosResult.errors.length > 0) {
          console.warn(
            'ROS Dockerfile generation had errors:',
            rosResult.errors
          );
        }

        // Update project with ROS configuration
        await Database.query(
          `UPDATE projects SET 
           config = COALESCE(config, '{}'::jsonb) || $1::jsonb,
           metadata = COALESCE(metadata, '{}'::jsonb) || $2::jsonb
           WHERE id = $3`,
          [
            JSON.stringify({ ros: rosConfig }),
            JSON.stringify({
              rosGenerated: true,
              rosFiles: {
                dockerfile: rosResult.path,
                compose: rosResult.composePath,
                bake: rosResult.bakePath,
              },
            }),
            projectId,
          ]
        );

        // Add ROS modules to project dependencies
        if (rosConfig.modules && Array.isArray(rosConfig.modules)) {
          const rosModules = rosConfig.modules.map(
            (moduleId: string, index: number) => ({
              projectId,
              moduleId,
              orderIndex: algorithms.length + index, // Append after existing algorithms
            })
          );

          if (rosModules.length > 0) {
            const rosRows: string[] = [];
            const rosParams: any[] = [];
            let param = 1;

            for (const { projectId, moduleId, orderIndex } of rosModules) {
              rosRows.push(
                `($${param++}, $${param++}, 'required', NULL, $${param++})`
              );
              rosParams.push(projectId, moduleId, orderIndex);
            }

            await Database.query(
              `INSERT INTO project_module_dependencies (project_id, module_id, dependency_type, version_constraint, order_index)
               VALUES ${rosRows.join(', ')}`,
              rosParams
            );
          }
        }
      } catch (error) {
        console.error('Error handling ROS configuration:', error);
        // Don't fail the entire project creation, just log the error
      }
    }

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
