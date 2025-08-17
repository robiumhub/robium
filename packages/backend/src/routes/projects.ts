import express from 'express';
import { Database } from '../utils/database';
import { AuthRequest } from '../types';
import { authenticateToken } from '../middleware/auth';
import crypto from 'crypto';
import { dockerfileGenerationService } from '../services/DockerfileGenerationService';
import { ROSProjectConfig } from '@robium/shared';
import { getGitHubService } from '../services/GitHubService';
import { projectScaffoldService } from '../services/ProjectScaffoldService';

const router = express.Router();

// GET /api/projects - Get current user's projects
router.get('/', authenticateToken, async (req: AuthRequest, res) => {
  try {
    const { type, search, is_template } = req.query;
    const ownerId = req.user?.userId as string;

    let query = `
      SELECT p.*, 
             COALESCE(p.tags, '{}'::text[]) AS tags,
             COUNT(DISTINCT pp.package_id) as package_count
      FROM projects p
      LEFT JOIN project_packages pp ON p.id = pp.project_id
      WHERE p.is_active = true AND p.owner_id = $1
    `;
    const params: string[] = [ownerId];
    let paramIndex = 2;

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
    const {
      category,
      search,
      use_cases,
      capabilities,
      robot_targets,
      simulators,
      ros_distros,
      rmw_implementations,
      licenses,
      difficulty,
      tags,
      requires_gpu,
      official_only,
      verified_only,
      sort_by = 'updated_at',
      sort_order = 'desc',
    } = req.query;

    let query = `
      SELECT p.*, 
             COUNT(DISTINCT pp.package_id) as package_count
      FROM projects p
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
      query += ` AND (p.name ILIKE $${paramIndex} OR p.description ILIKE $${paramIndex} OR p.tags::text ILIKE $${paramIndex})`;
      params.push(`%${search}%`);
      paramIndex++;
    }

    // Add metadata filters
    if (use_cases && typeof use_cases === 'string') {
      const useCases = use_cases.split(',');
      useCases.forEach((useCase) => {
        query += ` AND p.metadata->>'use_cases' LIKE $${paramIndex}`;
        params.push(`%${useCase}%`);
        paramIndex++;
      });
    }

    if (capabilities && typeof capabilities === 'string') {
      const caps = capabilities.split(',');
      caps.forEach((cap) => {
        query += ` AND p.metadata->>'capabilities' LIKE $${paramIndex}`;
        params.push(`%${cap}%`);
        paramIndex++;
      });
    }

    if (robot_targets && typeof robot_targets === 'string') {
      const targets = robot_targets.split(',');
      targets.forEach((target) => {
        query += ` AND p.metadata->>'robot_targets' LIKE $${paramIndex}`;
        params.push(`%${target}%`);
        paramIndex++;
      });
    }

    if (simulators && typeof simulators === 'string') {
      const sims = simulators.split(',');
      sims.forEach((sim) => {
        query += ` AND p.metadata->>'simulators' LIKE $${paramIndex}`;
        params.push(`%${sim}%`);
        paramIndex++;
      });
    }

    if (ros_distros && typeof ros_distros === 'string') {
      const distros = ros_distros.split(',');
      distros.forEach((distro) => {
        query += ` AND p.metadata->>'ros_distros' LIKE $${paramIndex}`;
        params.push(`%${distro}%`);
        paramIndex++;
      });
    }

    if (rmw_implementations && typeof rmw_implementations === 'string') {
      const rmws = rmw_implementations.split(',');
      rmws.forEach((rmw) => {
        query += ` AND p.metadata->>'rmw_implementations' LIKE $${paramIndex}`;
        params.push(`%${rmw}%`);
        paramIndex++;
      });
    }

    if (licenses && typeof licenses === 'string') {
      const licenseList = licenses.split(',');
      query += ` AND p.license = ANY($${paramIndex})`;
      params.push(licenseList as any);
      paramIndex++;
    }

    if (difficulty && typeof difficulty === 'string') {
      const difficulties = difficulty.split(',');
      difficulties.forEach((diff) => {
        query += ` AND p.metadata->>'difficulty' = $${paramIndex}`;
        params.push(diff);
        paramIndex++;
      });
    }

    if (tags && typeof tags === 'string') {
      const tagList = tags.split(',');
      query += ` AND p.tags && $${paramIndex}`;
      params.push(tagList as any);
      paramIndex++;
    }

    if (requires_gpu === 'true') {
      query += ` AND (p.config->>'simulation' = 'isaac' OR p.config->>'deployment' = 'cloud_gpu')`;
    }

    if (official_only === 'true') {
      query += ` AND p.author = 'Robium Team'`;
    }

    if (verified_only === 'true') {
      query += ` AND p.metadata->>'verified' = 'true'`;
    }

    query += ' GROUP BY p.id';

    // Add sorting
    const validSortFields = [
      'name',
      'created_at',
      'updated_at',
      'author',
      'license',
    ];
    const sortField = validSortFields.includes(sort_by as string)
      ? sort_by
      : 'updated_at';
    const sortDir = sort_order === 'asc' ? 'ASC' : 'DESC';
    query += ` ORDER BY p.${sortField} ${sortDir}`;

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

// GET /api/projects/templates/stats - Get template statistics
router.get('/templates/stats', async (req: AuthRequest, res) => {
  try {
    // Get total count
    const totalResult = (await Database.query(
      'SELECT COUNT(*) as total FROM projects WHERE is_active = true AND is_template = true'
    )) as { rows: Array<{ total: string }> };
    const totalTemplates = parseInt(totalResult.rows[0].total);

    // Get tag counts
    const tagResult = (await Database.query(`
      SELECT unnest(tags) as tag, COUNT(*) as count
      FROM projects 
      WHERE is_active = true AND is_template = true AND tags IS NOT NULL
      GROUP BY tag
      ORDER BY count DESC
    `)) as { rows: Array<{ tag: string; count: string }> };

    // Get license counts
    const licenseResult = (await Database.query(`
      SELECT license, COUNT(*) as count
      FROM projects 
      WHERE is_active = true AND is_template = true
      GROUP BY license
      ORDER BY count DESC
    `)) as { rows: Array<{ license: string; count: string }> };

    // Get metadata counts (simplified for now)
    const metadataResult = (await Database.query(`
      SELECT 
        COUNT(*) FILTER (WHERE metadata->>'difficulty' = 'beginner') as beginner_count,
        COUNT(*) FILTER (WHERE metadata->>'difficulty' = 'intermediate') as intermediate_count,
        COUNT(*) FILTER (WHERE metadata->>'difficulty' = 'advanced') as advanced_count,
        COUNT(*) FILTER (WHERE author = 'Robium Team') as official_count,
        COUNT(*) FILTER (WHERE metadata->>'verified' = 'true') as verified_count,
        COUNT(*) FILTER (WHERE config->>'simulation' = 'isaac' OR config->>'deployment' = 'cloud_gpu') as gpu_count
      FROM projects 
      WHERE is_active = true AND is_template = true
    `)) as {
      rows: Array<{
        beginner_count: string;
        intermediate_count: string;
        advanced_count: string;
        official_count: string;
        verified_count: string;
        gpu_count: string;
      }>;
    };

    const stats = {
      total_templates: totalTemplates,
      tag_counts: Object.fromEntries(
        tagResult.rows.map((row) => [row.tag, parseInt(row.count)])
      ),
      license_counts: Object.fromEntries(
        licenseResult.rows.map((row) => [row.license, parseInt(row.count)])
      ),
      difficulty_counts: {
        beginner: parseInt(metadataResult.rows[0].beginner_count) || 0,
        intermediate: parseInt(metadataResult.rows[0].intermediate_count) || 0,
        advanced: parseInt(metadataResult.rows[0].advanced_count) || 0,
      },
      official_count: parseInt(metadataResult.rows[0].official_count) || 0,
      verified_count: parseInt(metadataResult.rows[0].verified_count) || 0,
      gpu_count: parseInt(metadataResult.rows[0].gpu_count) || 0,
    };

    res.json({
      success: true,
      data: stats,
    });
  } catch (error) {
    console.error('Error fetching template statistics:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to fetch template statistics',
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
    const requestedName =
      req.body && typeof req.body.name === 'string' && req.body.name.trim()
        ? (req.body.name as string).trim()
        : null;

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
    const cloneName = requestedName || `${src.name}-copy`;

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

    // If admin, create a new GitHub repo from template (prefer fork)
    if (
      req.user?.role === 'admin' &&
      src.github_repo_owner &&
      src.github_repo_name
    ) {
      try {
        const gh = getGitHubService();
        const safeName = cloneName
          .toLowerCase()
          .replace(/[^a-z0-9-_]+/g, '-')
          .slice(0, 100);
        const forkOrg = process.env.GITHUB_FORK_ORG;
        let newRepo;
        try {
          newRepo = await gh.forkRepo(
            src.github_repo_owner,
            src.github_repo_name,
            {
              name: safeName,
              organization: forkOrg,
              waitSeconds: 60,
            }
          );
        } catch (forkErr) {
          console.warn(
            'Fork failed, falling back to template generate:',
            forkErr
          );
          newRepo = await gh.createRepoFromTemplate(
            src.github_repo_owner,
            src.github_repo_name,
            {
              name: safeName,
              description: src.description || cloneName,
              private: false,
            }
          );
        }

        await Database.query(
          `UPDATE projects SET github_repo_owner = $1, github_repo_name = $2, github_repo_url = $3, github_repo_id = $4, updated_at = NOW(), updated_by = $5 WHERE id = $6`,
          [
            newRepo.owner.login,
            newRepo.name,
            newRepo.html_url,
            newRepo.id,
            userId,
            cloneId,
          ]
        );

        cloned.rows[0].github_repo_owner = newRepo.owner.login;
        cloned.rows[0].github_repo_name = newRepo.name;
        cloned.rows[0].github_repo_url = newRepo.html_url;
        cloned.rows[0].github_repo_id = newRepo.id;
      } catch (ghErr) {
        console.error('GitHub template repo creation failed:', ghErr);
      }
    } else if (req.user?.role === 'admin') {
      // No repo info on template; create a new repo for the cloned project
      try {
        const gh = getGitHubService();
        const safeName = cloneName
          .toLowerCase()
          .replace(/[^a-z0-9-_]+/g, '-')
          .slice(0, 100);
        const repo = await gh.createRepoForAuthenticatedUser({
          name: safeName,
          description: src.description || cloneName,
          private: false,
          autoInit: true,
        });
        await Database.query(
          `UPDATE projects SET github_repo_owner = $1, github_repo_name = $2, github_repo_url = $3, github_repo_id = $4, updated_at = NOW(), updated_by = $5 WHERE id = $6`,
          [repo.owner.login, repo.name, repo.html_url, repo.id, userId, cloneId]
        );
        cloned.rows[0].github_repo_owner = repo.owner.login;
        cloned.rows[0].github_repo_name = repo.name;
        cloned.rows[0].github_repo_url = repo.html_url;
        cloned.rows[0].github_repo_id = repo.id;
      } catch (ghErr) {
        console.error('GitHub repo creation for clone failed:', ghErr);
      }
    }

    res
      .status(201)
      .json({ success: true, data: cloned.rows[0], message: 'Project cloned' });
  } catch (error) {
    console.error('Error cloning project:', error);
    res.status(500).json({ success: false, error: 'Failed to clone project' });
  }
});

// GET /api/projects/:id - Get specific project with modules and packages
router.get('/:id', authenticateToken, async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;
    const ownerId = req.user?.userId as string;

    // Get project details
    const projectResult = (await Database.query(
      `SELECT p.*, COALESCE(p.tags, '{}'::text[]) AS tags 
       FROM projects p 
       WHERE p.id = $1 AND p.is_active = true AND p.owner_id = $2`,
      [id, ownerId]
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
    const { name, description, config } = req.body;

    // Validate required fields
    if (!name) {
      return res.status(400).json({
        success: false,
        error: 'Name is required',
      });
    }

    // Validate config against shared schema
    if (config) {
      try {
        const { schemaValidator } = await import('@robium/shared');
        const validationResult = schemaValidator.validateProjectConfig(config);

        if (!validationResult.valid) {
          return res.status(400).json({
            success: false,
            error: 'Invalid configuration',
            details: validationResult.errors,
          });
        }
      } catch (validationError) {
        console.error('Schema validation error:', validationError);
        return res.status(400).json({
          success: false,
          error: 'Configuration validation failed',
        });
      }
    }

    // Generate a unique ID for the project
    const projectId = crypto.randomUUID();

    // Create the project with the new config structure
    const result = (await Database.query(
      `
      INSERT INTO projects (
        id, name, description, owner_id, config, type
      ) VALUES (
        $1, $2, $3, $4, $5, $6
      ) RETURNING *
    `,
      [
        projectId,
        name,
        description || '',
        userId, // owner_id
        config ? JSON.stringify(config) : '{}',
        'custom',
      ]
    )) as { rows: Array<Record<string, any>> };

    const newProject = result.rows[0];

    // If admin user, create a GitHub repo and persist repo info + scaffold
    if (req.user?.role === 'admin') {
      try {
        const gh = getGitHubService();
        const repo = await gh.createRepoForAuthenticatedUser({
          name: name
            .toLowerCase()
            .replace(/[^a-z0-9-_]+/g, '-')
            .slice(0, 100),
          description: description || `Robium project ${name}`,
          private: false,
          autoInit: true,
        });

        await Database.query(
          `UPDATE projects SET github_repo_owner = $1, github_repo_name = $2, github_repo_url = $3, github_repo_id = $4, updated_at = NOW(), updated_by = $5 WHERE id = $6`,
          [
            repo.owner.login,
            repo.name,
            repo.html_url,
            repo.id,
            userId,
            projectId,
          ]
        );

        newProject.github_repo_owner = repo.owner.login;
        newProject.github_repo_name = repo.name;
        newProject.github_repo_url = repo.html_url;
        newProject.github_repo_id = repo.id;

        // Generate scaffold files and push as initial commit
        const files = projectScaffoldService.generateScaffold(name);
        await gh.createOrUpdateFiles(
          repo.owner.login,
          repo.name,
          files,
          'chore: initial project scaffold'
        );

        // Replace Dockerfile with generated content to match UI preview
        try {
          let gen: any;
          if (
            newProject.config &&
            newProject.config.robotTarget !== undefined
          ) {
            gen = await dockerfileGenerationService.generateFromProjectConfig(
              projectId,
              newProject.config,
              { includeComments: true, optimize: true }
            );
          } else {
            gen = await dockerfileGenerationService.generateDockerfile(
              projectId,
              {
                includeComments: true,
                optimize: true,
              }
            );
          }

          if (gen?.content) {
            await gh.createOrUpdateFiles(
              repo.owner.login,
              repo.name,
              [{ path: 'Dockerfile', content: gen.content }],
              'chore: update Dockerfile to generated version'
            );
          }
        } catch (genErr) {
          console.error('Failed to generate and push Dockerfile:', genErr);
        }
      } catch (ghErr) {
        console.error('GitHub repo creation failed:', ghErr);
        // Non-blocking
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

    // Only project owner or admin can delete
    if (req.user?.role !== 'admin' && project.owner_id !== userId) {
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

// POST /api/projects/:id/convert-to-template - Admin only: turn project into a template
router.post(
  '/:id/convert-to-template',
  authenticateToken,
  async (req: AuthRequest, res) => {
    try {
      if (req.user?.role !== 'admin') {
        return res.status(403).json({ success: false, error: 'Forbidden' });
      }
      const { id } = req.params;
      const result = (await Database.query(
        `UPDATE projects SET is_template = true, updated_at = NOW(), updated_by = $1 WHERE id = $2 AND is_active = true RETURNING *`,
        [req.user.userId, id]
      )) as { rows: Array<Record<string, any>> };
      if (result.rows.length === 0) {
        return res
          .status(404)
          .json({ success: false, error: 'Project not found' });
      }
      res.json({
        success: true,
        data: result.rows[0],
        message: 'Project converted to template',
      });
    } catch (error) {
      console.error('Error converting to template:', error);
      res.status(500).json({
        success: false,
        error: 'Failed to convert project to template',
      });
    }
  }
);
