import express from 'express';
import { Database } from '../utils/database';
import { authMiddleware, AuthRequest } from '../middleware/auth';
import crypto from 'crypto';
import { getGitHubService } from '../services/GitHubService';
import { projectScaffoldService } from '../services/ProjectScaffoldService';
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

// POST /api/projects - Create a new project
router.post('/', authMiddleware, async (req: AuthRequest, res) => {
  try {
    const { name, description, tags, isTemplate, config, metadata, github } = req.body;
    const userId = req.user?.id;

    if (!userId) {
      return res.status(401).json({
        success: false,
        error: 'Authentication required',
      });
    }

    if (!name || !name.trim()) {
      return res.status(400).json({
        success: false,
        error: 'Project name is required',
      });
    }

    const db = Database.getDatabase();

    // Generate new project ID
    const projectId = crypto.randomUUID();
    const projectName = name.trim();

    // Insert new project
    await new Promise<void>((resolve, reject) => {
      db.run(
        `
        INSERT INTO projects (
          id, name, description, owner_id, is_active, is_template,
          tags, config, metadata, created_at, updated_at
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)
      `,
        [
          projectId,
          projectName,
          description || '',
          userId,
          1, // is_active
          isTemplate ? 1 : 0, // is_template
          JSON.stringify(tags || []),
          JSON.stringify(config || {}),
          JSON.stringify(metadata || {}),
        ],
        (err) => {
          if (err) reject(err);
          else resolve();
        }
      );
    });

    // Get the created project
    const createdProject = await new Promise<any>((resolve, reject) => {
      db.get(
        `
        SELECT id, name, description, owner_id, is_active, is_template,
               tags, config, metadata, created_at, updated_at
        FROM projects
        WHERE id = ?
      `,
        [projectId],
        (err, row) => {
          if (err) reject(err);
          else resolve(row);
        }
      );
    });

    if (!createdProject) {
      return res.status(500).json({
        success: false,
        error: 'Failed to create project',
      });
    }

    // Transform the data to match our Project interface
    const transformedProject: Project = {
      id: createdProject.id,
      name: createdProject.name,
      description: createdProject.description || '',
      ownerId: createdProject.owner_id,
      isActive: Boolean(createdProject.is_active),
      isTemplate: Boolean(createdProject.is_template),
      tags: JSON.parse(createdProject.tags || '[]'),
      config: JSON.parse(createdProject.config || '{}'),
      metadata: JSON.parse(createdProject.metadata || '{}'),
      createdAt: new Date(createdProject.created_at),
      updatedAt: new Date(createdProject.updated_at),
    };

    // GitHub repository creation (if requested and user is admin, or if creating a template)
    let githubRepo = null;
    if (github?.createRepo && (req.user?.role === 'admin' || isTemplate)) {
      try {
        const gh = getGitHubService();
        const repo = await gh.createRepoForAuthenticatedUser({
          name: (github.repoName || projectName)
            .toLowerCase()
            .replace(/[^a-z0-9-_]+/g, '-')
            .slice(0, 100),
          description: description || `Robium project ${projectName}`,
          private: github.visibility === 'private',
          autoInit: true,
        });

        // Update project with GitHub repo info
        await new Promise<void>((resolve, reject) => {
          db.run(
            `
            UPDATE projects SET 
              github_repo_owner = ?, 
              github_repo_name = ?, 
              github_repo_url = ?, 
              github_repo_id = ?, 
              updated_at = CURRENT_TIMESTAMP
            WHERE id = ?
          `,
            [repo.owner.login, repo.name, repo.html_url, repo.id, projectId],
            (err) => {
              if (err) reject(err);
              else resolve();
            }
          );
        });

        // Generate scaffold files and push as initial commit
        const files = projectScaffoldService.generateScaffold(projectName);
        await gh.createOrUpdateFiles(
          repo.owner.login,
          repo.name,
          files,
          'chore: initial project scaffold'
        );

        githubRepo = {
          id: repo.id,
          name: repo.name,
          full_name: repo.full_name,
          html_url: repo.html_url,
          owner: repo.owner,
        };

        console.log(`Successfully created GitHub repo: ${repo.html_url}`);
      } catch (ghErr) {
        console.error('GitHub repo creation failed:', ghErr);
        // Non-blocking - project creation still succeeds
      }
    }

    res.status(201).json({
      success: true,
      data: {
        project: transformedProject,
        githubRepo,
      },
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

// POST /api/projects/:id/clone - Clone an existing project/template
router.post('/:id/clone', authMiddleware, async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;
    const { name, github } = req.body;
    const userId = req.user?.id;

    if (!userId) {
      return res.status(401).json({
        success: false,
        error: 'Authentication required',
      });
    }

    if (!name || !name.trim()) {
      return res.status(400).json({
        success: false,
        error: 'Project name is required',
      });
    }

    const db = Database.getDatabase();

    // Get the source project/template
    const sourceProject = await new Promise<any>((resolve, reject) => {
      db.get(
        `
        SELECT id, name, description, owner_id, is_active, is_template,
               tags, config, metadata, template_visibility, template_version,
               template_published_at, created_at, updated_at
        FROM projects
        WHERE id = ? AND (is_template = 1 OR owner_id = ?)
      `,
        [id, userId],
        (err, row) => {
          if (err) reject(err);
          else resolve(row);
        }
      );
    });

    if (!sourceProject) {
      return res.status(404).json({
        success: false,
        error: 'Project not found or access denied',
      });
    }

    // Generate new project ID
    const cloneId = crypto.randomUUID();
    const cloneName = name.trim();

    // Insert cloned project
    await new Promise<void>((resolve, reject) => {
      db.run(
        `
        INSERT INTO projects (
          id, name, description, owner_id, is_active, is_template,
          tags, config, metadata, created_at, updated_at
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)
      `,
        [
          cloneId,
          cloneName,
          sourceProject.description || cloneName,
          userId,
          1, // is_active
          0, // is_template (cloned projects are not templates by default)
          sourceProject.tags || '[]',
          sourceProject.config || '{}',
          sourceProject.metadata || '{}',
        ],
        (err) => {
          if (err) reject(err);
          else resolve();
        }
      );
    });

    // Get the cloned project
    const clonedProject = await new Promise<any>((resolve, reject) => {
      db.get(
        `
        SELECT id, name, description, owner_id, is_active, is_template,
               tags, config, metadata, template_visibility, template_version,
               template_published_at, created_at, updated_at
        FROM projects
        WHERE id = ?
      `,
        [cloneId],
        (err, row) => {
          if (err) reject(err);
          else resolve(row);
        }
      );
    });

    if (!clonedProject) {
      return res.status(500).json({
        success: false,
        error: 'Failed to create cloned project',
      });
    }

    // Transform the data to match our Project interface
    const transformedProject: Project = {
      id: clonedProject.id,
      name: clonedProject.name,
      description: clonedProject.description || '',
      ownerId: clonedProject.owner_id,
      isActive: Boolean(clonedProject.is_active),
      isTemplate: Boolean(clonedProject.is_template),
      tags: JSON.parse(clonedProject.tags || '[]'),
      config: JSON.parse(clonedProject.config || '{}'),
      metadata: JSON.parse(clonedProject.metadata || '{}'),
      templateVisibility: clonedProject.template_visibility,
      templateVersion: clonedProject.template_version,
      templatePublishedAt: clonedProject.template_published_at
        ? new Date(clonedProject.template_published_at)
        : undefined,
      createdAt: new Date(clonedProject.created_at),
      updatedAt: new Date(clonedProject.updated_at),
    };

    // GitHub repository creation (if requested and user is admin, or if cloning from template)
    let githubRepo = null;
    const isCloningFromTemplate = sourceProject.is_template === 1;

    if (github?.createRepo && (req.user?.role === 'admin' || isCloningFromTemplate)) {
      try {
        const gh = getGitHubService();
        const repo = await gh.createRepoForAuthenticatedUser({
          name: (github.repoName || cloneName)
            .toLowerCase()
            .replace(/[^a-z0-9-_]+/g, '-')
            .slice(0, 100),
          description: clonedProject.description || `Robium project ${cloneName}`,
          private: github.visibility === 'private',
          autoInit: true,
        });

        // Update project with GitHub repo info
        await new Promise<void>((resolve, reject) => {
          db.run(
            `
            UPDATE projects SET 
              github_repo_owner = ?, 
              github_repo_name = ?, 
              github_repo_url = ?, 
              github_repo_id = ?, 
              updated_at = CURRENT_TIMESTAMP
            WHERE id = ?
          `,
            [repo.owner.login, repo.name, repo.html_url, repo.id, cloneId],
            (err) => {
              if (err) reject(err);
              else resolve();
            }
          );
        });

        // Generate scaffold files and push as initial commit
        const files = projectScaffoldService.generateScaffold(cloneName);
        await gh.createOrUpdateFiles(
          repo.owner.login,
          repo.name,
          files,
          'chore: initial project scaffold'
        );

        githubRepo = {
          id: repo.id,
          name: repo.name,
          full_name: repo.full_name,
          html_url: repo.html_url,
          owner: repo.owner,
        };

        console.log(`Successfully created GitHub repo for cloned project: ${repo.html_url}`);
      } catch (ghErr) {
        console.error('GitHub repo creation failed for cloned project:', ghErr);
        // Non-blocking - project cloning still succeeds
      }
    }

    res.json({
      success: true,
      data: {
        project: transformedProject,
        githubRepo,
      },
      message: 'Project cloned successfully',
    });
  } catch (error) {
    console.error('Error cloning project:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to clone project',
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

// POST /api/projects/filters/categories - Create new filter category
router.post('/filters/categories', authMiddleware, async (req: AuthRequest, res) => {
  try {
    const { name, displayName, description, isActive = true, sortOrder = 0 } = req.body;
    const db = Database.getDatabase();

    // Validate required fields
    if (!name || !displayName) {
      return res.status(400).json({
        success: false,
        error: 'Name and displayName are required',
      });
    }

    // Check if category with same name already exists
    const existingCategory = await new Promise<any>((resolve, reject) => {
      db.get('SELECT id FROM filter_categories WHERE name = ?', [name], (err, row) => {
        if (err) reject(err);
        else resolve(row);
      });
    });

    if (existingCategory) {
      return res.status(409).json({
        success: false,
        error: 'Category with this name already exists',
      });
    }

    // Generate a unique ID for the category
    const categoryId = crypto.randomUUID();

    // Insert new category
    const result = await new Promise<any>((resolve, reject) => {
      db.run(
        `
        INSERT INTO filter_categories (id, name, display_name, description, type, is_active, sort_order, created_at, updated_at)
        VALUES (?, ?, ?, ?, ?, ?, ?, datetime('now'), datetime('now'))
      `,
        [categoryId, name, displayName, description || '', 'string', isActive ? 1 : 0, sortOrder],
        function (err) {
          if (err) reject(err);
          else resolve({ id: categoryId });
        }
      );
    });

    // Fetch the created category
    const newCategory = await new Promise<any>((resolve, reject) => {
      db.get(
        `
        SELECT id, name, display_name, description, type, is_active, sort_order, created_at, updated_at
        FROM filter_categories
        WHERE id = ?
      `,
        [result.id],
        (err, row) => {
          if (err) reject(err);
          else resolve(row);
        }
      );
    });

    const transformedCategory: FilterCategory = {
      id: newCategory.id,
      name: newCategory.name,
      displayName: newCategory.display_name,
      description: newCategory.description,
      type: newCategory.type as 'string' | 'boolean' | 'number',
      isActive: Boolean(newCategory.is_active),
      sortOrder: newCategory.sort_order,
      createdAt: new Date(newCategory.created_at),
      updatedAt: new Date(newCategory.updated_at),
    };

    res.status(201).json({
      success: true,
      data: transformedCategory,
      message: 'Category created successfully',
    });
  } catch (error) {
    console.error('Error creating filter category:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to create filter category',
    });
  }
});

// PUT /api/projects/filters/categories/:id - Update filter category
router.put('/filters/categories/:id', authMiddleware, async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;
    const { displayName, description, isActive, sortOrder } = req.body;
    const db = Database.getDatabase();

    // Check if category exists
    const existingCategory = await new Promise<any>((resolve, reject) => {
      db.get('SELECT id FROM filter_categories WHERE id = ?', [id], (err, row) => {
        if (err) reject(err);
        else resolve(row);
      });
    });

    if (!existingCategory) {
      return res.status(404).json({
        success: false,
        error: 'Category not found',
      });
    }

    // Update category
    await new Promise<void>((resolve, reject) => {
      db.run(
        `
        UPDATE filter_categories
        SET display_name = ?, description = ?, is_active = ?, sort_order = ?, updated_at = datetime('now')
        WHERE id = ?
      `,
        [displayName, description || '', isActive ? 1 : 0, sortOrder, id],
        (err) => {
          if (err) reject(err);
          else resolve();
        }
      );
    });

    // Fetch the updated category
    const updatedCategory = await new Promise<any>((resolve, reject) => {
      db.get(
        `
        SELECT id, name, display_name, description, type, is_active, sort_order, created_at, updated_at
        FROM filter_categories
        WHERE id = ?
      `,
        [id],
        (err, row) => {
          if (err) reject(err);
          else resolve(row);
        }
      );
    });

    const transformedCategory: FilterCategory = {
      id: updatedCategory.id,
      name: updatedCategory.name,
      displayName: updatedCategory.display_name,
      description: updatedCategory.description,
      type: updatedCategory.type as 'string' | 'boolean' | 'number',
      isActive: Boolean(updatedCategory.is_active),
      sortOrder: updatedCategory.sort_order,
      createdAt: new Date(updatedCategory.created_at),
      updatedAt: new Date(updatedCategory.updated_at),
    };

    res.json({
      success: true,
      data: transformedCategory,
      message: 'Category updated successfully',
    });
  } catch (error) {
    console.error('Error updating filter category:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to update filter category',
    });
  }
});

// DELETE /api/projects/filters/categories/:id - Delete filter category
router.delete('/filters/categories/:id', authMiddleware, async (req: AuthRequest, res) => {
  try {
    const { id } = req.params;
    const db = Database.getDatabase();

    // Check if category exists
    const existingCategory = await new Promise<any>((resolve, reject) => {
      db.get('SELECT id FROM filter_categories WHERE id = ?', [id], (err, row) => {
        if (err) reject(err);
        else resolve(row);
      });
    });

    if (!existingCategory) {
      return res.status(404).json({
        success: false,
        error: 'Category not found',
      });
    }

    // Check if category has associated values
    const associatedValues = await new Promise<any[]>((resolve, reject) => {
      db.all(
        'SELECT id FROM filter_values WHERE category_id = ? AND is_active = 1',
        [id],
        (err, rows) => {
          if (err) reject(err);
          else resolve(rows || []);
        }
      );
    });

    if (associatedValues.length > 0) {
      return res.status(400).json({
        success: false,
        error: 'Cannot delete category with associated filter values',
      });
    }

    // Delete category
    await new Promise<void>((resolve, reject) => {
      db.run('DELETE FROM filter_categories WHERE id = ?', [id], (err) => {
        if (err) reject(err);
        else resolve();
      });
    });

    res.json({
      success: true,
      message: 'Category deleted successfully',
    });
  } catch (error) {
    console.error('Error deleting filter category:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to delete filter category',
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

// POST /api/projects/filters/values - Create new filter value
router.post('/filters/values', authMiddleware, async (req: AuthRequest, res) => {
  try {
    const db = Database.getDatabase();
    const {
      categoryId,
      value,
      displayName,
      description,
      isActive = true,
      sortOrder = 0,
    } = req.body;

    // Validate required fields
    if (!categoryId || !value || !displayName) {
      return res.status(400).json({
        success: false,
        error: 'Category ID, value, and display name are required',
      });
    }

    // Check if category exists
    const category = await new Promise<any>((resolve, reject) => {
      db.get(
        'SELECT id FROM filter_categories WHERE id = ? AND is_active = 1',
        [categoryId],
        (err, row) => {
          if (err) reject(err);
          else resolve(row);
        }
      );
    });

    if (!category) {
      return res.status(400).json({
        success: false,
        error: 'Category not found',
      });
    }

    // Check if value already exists in this category
    const existingValue = await new Promise<any>((resolve, reject) => {
      db.get(
        'SELECT id FROM filter_values WHERE category_id = ? AND value = ?',
        [categoryId, value],
        (err, row) => {
          if (err) reject(err);
          else resolve(row);
        }
      );
    });

    if (existingValue) {
      return res.status(400).json({
        success: false,
        error: 'Filter value already exists in this category',
      });
    }

    // Generate ID and insert
    const id = crypto.randomUUID();
    await new Promise<void>((resolve, reject) => {
      db.run(
        `
        INSERT INTO filter_values (id, category_id, value, display_name, description, is_active, sort_order, created_at, updated_at)
        VALUES (?, ?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)
      `,
        [id, categoryId, value, displayName, description || null, isActive ? 1 : 0, sortOrder],
        (err) => {
          if (err) reject(err);
          else resolve();
        }
      );
    });

    // Get the created value
    const createdValue = await new Promise<any>((resolve, reject) => {
      db.get(
        `
        SELECT id, category_id, value, display_name, description, is_active, sort_order, created_at, updated_at
        FROM filter_values
        WHERE id = ?
      `,
        [id],
        (err, row) => {
          if (err) reject(err);
          else resolve(row);
        }
      );
    });

    const transformedValue: FilterValue = {
      id: createdValue.id,
      categoryId: createdValue.category_id,
      value: createdValue.value,
      displayName: createdValue.display_name,
      description: createdValue.description,
      isActive: Boolean(createdValue.is_active),
      sortOrder: createdValue.sort_order,
      createdAt: new Date(createdValue.created_at),
      updatedAt: new Date(createdValue.updated_at),
    };

    res.status(201).json({
      success: true,
      data: transformedValue,
      message: 'Filter value created successfully',
    });
  } catch (error) {
    console.error('Error creating filter value:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to create filter value',
    });
  }
});

// PUT /api/projects/filters/values/:id - Update filter value
router.put('/filters/values/:id', authMiddleware, async (req: AuthRequest, res) => {
  try {
    const db = Database.getDatabase();
    const { id } = req.params;
    const { categoryId, value, displayName, description, isActive, sortOrder } = req.body;

    // Check if value exists
    const existingValue = await new Promise<any>((resolve, reject) => {
      db.get('SELECT id FROM filter_values WHERE id = ?', [id], (err, row) => {
        if (err) reject(err);
        else resolve(row);
      });
    });

    if (!existingValue) {
      return res.status(404).json({
        success: false,
        error: 'Filter value not found',
      });
    }

    // Build update query
    const updates: string[] = [];
    const params: any[] = [];

    if (categoryId !== undefined) {
      updates.push('category_id = ?');
      params.push(categoryId);
    }
    if (value !== undefined) {
      updates.push('value = ?');
      params.push(value);
    }
    if (displayName !== undefined) {
      updates.push('display_name = ?');
      params.push(displayName);
    }
    if (description !== undefined) {
      updates.push('description = ?');
      params.push(description);
    }
    if (isActive !== undefined) {
      updates.push('is_active = ?');
      params.push(isActive ? 1 : 0);
    }
    if (sortOrder !== undefined) {
      updates.push('sort_order = ?');
      params.push(sortOrder);
    }

    if (updates.length === 0) {
      return res.status(400).json({
        success: false,
        error: 'No updates provided',
      });
    }

    updates.push('updated_at = CURRENT_TIMESTAMP');
    params.push(id);

    // Update the value
    await new Promise<void>((resolve, reject) => {
      db.run(`UPDATE filter_values SET ${updates.join(', ')} WHERE id = ?`, params, (err) => {
        if (err) reject(err);
        else resolve();
      });
    });

    // Get the updated value
    const updatedValue = await new Promise<any>((resolve, reject) => {
      db.get(
        `
        SELECT id, category_id, value, display_name, description, is_active, sort_order, created_at, updated_at
        FROM filter_values
        WHERE id = ?
      `,
        [id],
        (err, row) => {
          if (err) reject(err);
          else resolve(row);
        }
      );
    });

    const transformedValue: FilterValue = {
      id: updatedValue.id,
      categoryId: updatedValue.category_id,
      value: updatedValue.value,
      displayName: updatedValue.display_name,
      description: updatedValue.description,
      isActive: Boolean(updatedValue.is_active),
      sortOrder: updatedValue.sort_order,
      createdAt: new Date(updatedValue.created_at),
      updatedAt: new Date(updatedValue.updated_at),
    };

    res.json({
      success: true,
      data: transformedValue,
      message: 'Filter value updated successfully',
    });
  } catch (error) {
    console.error('Error updating filter value:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to update filter value',
    });
  }
});

// DELETE /api/projects/filters/values/:id - Delete filter value
router.delete('/filters/values/:id', authMiddleware, async (req: AuthRequest, res) => {
  try {
    const db = Database.getDatabase();
    const { id } = req.params;

    // Check if value exists
    const existingValue = await new Promise<any>((resolve, reject) => {
      db.get('SELECT id FROM filter_values WHERE id = ?', [id], (err, row) => {
        if (err) reject(err);
        else resolve(row);
      });
    });

    if (!existingValue) {
      return res.status(404).json({
        success: false,
        error: 'Filter value not found',
      });
    }

    // Delete the value
    await new Promise<void>((resolve, reject) => {
      db.run('DELETE FROM filter_values WHERE id = ?', [id], (err) => {
        if (err) reject(err);
        else resolve();
      });
    });

    res.json({
      success: true,
      message: 'Filter value deleted successfully',
    });
  } catch (error) {
    console.error('Error deleting filter value:', error);
    res.status(500).json({
      success: false,
      error: 'Failed to delete filter value',
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
