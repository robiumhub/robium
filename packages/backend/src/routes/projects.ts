import express from 'express';
import { authenticateToken } from '../middleware/auth';
import { requirePermission } from '../middleware/rbac';
import { Permission } from '../utils/permissions';
import { pool } from '../utils/database';
import { logger } from '../utils/logger';
import { ConflictError, NotFoundError, ValidationError } from '../utils/errors';
import { AuthRequest } from '../types';

const router = express.Router();

// Get all projects for the authenticated user
router.get('/', authenticateToken, requirePermission(Permission.PROJECT_READ), async (req: AuthRequest, res) => {
  try {
    const { userId } = req.user!;
    
    const query = `
      SELECT p.id, p.name, p.description, p.owner_id, p.created_at, p.updated_at,
             u.username as owner_username
      FROM projects p
      JOIN users u ON p.owner_id = u.id
      WHERE p.owner_id = $1
      ORDER BY p.created_at DESC
    `;
    
    const result = await pool.query(query, [userId]);
    
    logger.info('Projects fetched successfully', {
      userId,
      projectCount: result.rows.length,
      requestId: (req as any).requestId
    });
    
    res.json({
      success: true,
      data: result.rows,
      count: result.rows.length
    });
  } catch (error) {
    logger.error('Failed to fetch projects', {
      userId: req.user?.userId,
      error: error instanceof Error ? error.message : 'Unknown error',
      requestId: (req as any).requestId
    });
    
    res.status(500).json({
      success: false,
      message: 'Failed to fetch projects',
      error: error instanceof Error ? error.message : 'Unknown error'
    });
  }
});

// Get a specific project by ID
router.get('/:projectId', authenticateToken, requirePermission(Permission.PROJECT_READ), async (req: AuthRequest, res) => {
  try {
    const { projectId } = req.params;
    const { userId } = req.user!;
    
    const query = `
      SELECT p.id, p.name, p.description, p.owner_id, p.created_at, p.updated_at,
             u.username as owner_username
      FROM projects p
      JOIN users u ON p.owner_id = u.id
      WHERE p.id = $1 AND p.owner_id = $2
    `;
    
    const result = await pool.query(query, [projectId, userId]);
    
    if (result.rows.length === 0) {
      throw new NotFoundError('Project not found');
    }
    
    logger.info('Project fetched successfully', {
      projectId,
      userId,
      requestId: (req as any).requestId
    });
    
    res.json({
      success: true,
      data: result.rows[0]
    });
  } catch (error) {
    logger.error('Failed to fetch project', {
      projectId: req.params.projectId,
      userId: req.user?.userId,
      error: error instanceof Error ? error.message : 'Unknown error',
      requestId: (req as any).requestId
    });
    
    if (error instanceof NotFoundError) {
      res.status(404).json({
        success: false,
        message: error.message
      });
    } else {
      res.status(500).json({
        success: false,
        message: 'Failed to fetch project',
        error: error instanceof Error ? error.message : 'Unknown error'
      });
    }
  }
});

// Create a new project
router.post('/', authenticateToken, requirePermission(Permission.PROJECT_CREATE), async (req: AuthRequest, res) => {
  try {
    const { 
      name, 
      description, 
      category,
      tags,
      isPublic,
      algorithms,
      environment,
      settings
    } = req.body;
    const { userId } = req.user!;
    
    // Validate required fields
    if (!name || typeof name !== 'string' || name.trim().length === 0) {
      throw new ValidationError('Project name is required');
    }
    
    if (name.trim().length > 255) {
      throw new ValidationError('Project name must be less than 255 characters');
    }
    
    if (description && typeof description !== 'string') {
      throw new ValidationError('Project description must be a string');
    }
    
    // Check if project name already exists for this user
    const existingQuery = 'SELECT id FROM projects WHERE name = $1 AND owner_id = $2';
    const existingResult = await pool.query(existingQuery, [name.trim(), userId]);
    
    if (existingResult.rows.length > 0) {
      throw new ConflictError('A project with this name already exists');
    }
    
    // Start a transaction
    const client = await pool.connect();
    try {
      await client.query('BEGIN');
      
      // Create the project
      const insertProjectQuery = `
        INSERT INTO projects (name, description, owner_id)
        VALUES ($1, $2, $3)
        RETURNING id, name, description, owner_id, created_at, updated_at
      `;
      
      const projectResult = await client.query(insertProjectQuery, [
        name.trim(),
        description ? description.trim() : null,
        userId
      ]);
      
      const newProject = projectResult.rows[0];
      
      // Create project configuration
      const insertConfigQuery = `
        INSERT INTO project_configurations (
          project_id, type, version, base_image, workdir,
          system_dependencies, python_dependencies, node_dependencies,
          environment_variables, ports, volumes, command,
          category, tags, is_public, algorithms,
          max_memory, cpu_limit, enable_gpu,
          auto_save, enable_debugging, enable_logging, backup_frequency
        ) VALUES (
          $1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12,
          $13, $14, $15, $16, $17, $18, $19, $20, $21, $22, $23
        )
        RETURNING id
      `;
      
      // Determine project type from base image
      const projectType = environment?.baseImage?.includes('python') ? 'python' :
                         environment?.baseImage?.includes('node') ? 'nodejs' : 'python';
      
      await client.query(insertConfigQuery, [
        newProject.id,
        projectType,
        '1.0.0',
        environment?.baseImage || 'python:3.11-slim',
        '/app',
        JSON.stringify(environment?.systemDependencies || []),
        JSON.stringify(environment?.pythonDependencies || []),
        JSON.stringify(environment?.nodeDependencies || []),
        JSON.stringify(environment?.environmentVariables || {}),
        JSON.stringify(environment?.ports || []),
        JSON.stringify(environment?.volumes || []),
        environment?.command || null,
        category || null,
        JSON.stringify(tags || []),
        isPublic || false,
        JSON.stringify(algorithms || []),
        settings?.maxMemory || 2048,
        settings?.cpuLimit || 2,
        settings?.enableGPU || false,
        settings?.autoSave !== false,
        settings?.enableDebugging || false,
        settings?.enableLogging !== false,
        settings?.backupFrequency || 'weekly'
      ]);
      
      // Register project configuration with DockerfileGenerationService
      const { dockerfileGenerationService } = require('../services/DockerfileGenerationService');
      const projectConfig = {
        id: newProject.id,
        name: newProject.name,
        version: '1.0.0',
        description: newProject.description,
        type: projectType,
        baseImage: environment?.baseImage || 'python:3.11-slim',
        workdir: '/app',
        systemDependencies: environment?.systemDependencies || [],
        pythonDependencies: environment?.pythonDependencies || [],
        nodeDependencies: environment?.nodeDependencies || [],
        environmentVariables: environment?.environmentVariables || {},
        ports: environment?.ports?.map(p => p.toString()) || [],
        volumes: environment?.volumes || [],
        command: environment?.command
      };
      
      dockerfileGenerationService.registerProjectConfig(projectConfig);
      
      // Log user activity
      const logQuery = `
        INSERT INTO user_activity_logs (user_id, project_id, action, details, ip_address, user_agent)
        VALUES ($1, $2, $3, $4, $5, $6)
      `;
      
      await client.query(logQuery, [
        userId,
        newProject.id,
        'project_created',
        JSON.stringify({ name: newProject.name, category, algorithms: algorithms?.length || 0 }),
        req.ip,
        req.get('User-Agent')
      ]);
      
      await client.query('COMMIT');
      
      logger.info('Project created successfully', {
        projectId: newProject.id,
        projectName: newProject.name,
        userId,
        requestId: (req as any).requestId
      });
      
      res.status(201).json({
        success: true,
        message: 'Project created successfully',
        data: newProject
      });
    } catch (error) {
      await client.query('ROLLBACK');
      throw error;
    } finally {
      client.release();
    }
  } catch (error) {
    logger.error('Failed to create project', {
      userId: req.user?.userId,
      error: error instanceof Error ? error.message : 'Unknown error',
      requestId: (req as any).requestId
    });
    
    if (error instanceof ValidationError) {
      res.status(400).json({
        success: false,
        message: error.message
      });
    } else if (error instanceof ConflictError) {
      res.status(409).json({
        success: false,
        message: error.message
      });
    } else {
      res.status(500).json({
        success: false,
        message: 'Failed to create project',
        error: error instanceof Error ? error.message : 'Unknown error'
      });
    }
  }
});

// Update a project
router.put('/:projectId', authenticateToken, requirePermission(Permission.PROJECT_UPDATE), async (req: AuthRequest, res) => {
  try {
    const { projectId } = req.params;
    const { name, description } = req.body;
    const { userId } = req.user!;
    
    // Validate required fields
    if (!name || typeof name !== 'string' || name.trim().length === 0) {
      throw new ValidationError('Project name is required');
    }
    
    if (name.trim().length > 255) {
      throw new ValidationError('Project name must be less than 255 characters');
    }
    
    if (description && typeof description !== 'string') {
      throw new ValidationError('Project description must be a string');
    }
    
    // Check if project exists and user owns it
    const existingQuery = 'SELECT id FROM projects WHERE id = $1 AND owner_id = $2';
    const existingResult = await pool.query(existingQuery, [projectId, userId]);
    
    if (existingResult.rows.length === 0) {
      throw new NotFoundError('Project not found');
    }
    
    // Check if new name conflicts with another project
    const conflictQuery = 'SELECT id FROM projects WHERE name = $1 AND owner_id = $2 AND id != $3';
    const conflictResult = await pool.query(conflictQuery, [name.trim(), userId, projectId]);
    
    if (conflictResult.rows.length > 0) {
      throw new ConflictError('A project with this name already exists');
    }
    
    // Update the project
    const updateQuery = `
      UPDATE projects 
      SET name = $1, description = $2, updated_at = CURRENT_TIMESTAMP
      WHERE id = $3 AND owner_id = $4
      RETURNING id, name, description, owner_id, created_at, updated_at
    `;
    
    const result = await pool.query(updateQuery, [
      name.trim(),
      description ? description.trim() : null,
      projectId,
      userId
    ]);
    
    const updatedProject = result.rows[0];
    
    logger.info('Project updated successfully', {
      projectId: updatedProject.id,
      projectName: updatedProject.name,
      userId,
      requestId: (req as any).requestId
    });
    
    res.json({
      success: true,
      message: 'Project updated successfully',
      data: updatedProject
    });
  } catch (error) {
    logger.error('Failed to update project', {
      projectId: req.params.projectId,
      userId: req.user?.userId,
      error: error instanceof Error ? error.message : 'Unknown error',
      requestId: (req as any).requestId
    });
    
    if (error instanceof ValidationError) {
      res.status(400).json({
        success: false,
        message: error.message
      });
    } else if (error instanceof ConflictError) {
      res.status(409).json({
        success: false,
        message: error.message
      });
    } else if (error instanceof NotFoundError) {
      res.status(404).json({
        success: false,
        message: error.message
      });
    } else {
      res.status(500).json({
        success: false,
        message: 'Failed to update project',
        error: error instanceof Error ? error.message : 'Unknown error'
      });
    }
  }
});

// Delete a project
router.delete('/:projectId', authenticateToken, requirePermission(Permission.PROJECT_DELETE), async (req: AuthRequest, res) => {
  try {
    const { projectId } = req.params;
    const { userId } = req.user!;
    
    // Check if project exists and user owns it
    const existingQuery = 'SELECT id, name FROM projects WHERE id = $1 AND owner_id = $2';
    const existingResult = await pool.query(existingQuery, [projectId, userId]);
    
    if (existingResult.rows.length === 0) {
      throw new NotFoundError('Project not found');
    }
    
    const projectName = existingResult.rows[0].name;
    
    // Delete the project
    const deleteQuery = 'DELETE FROM projects WHERE id = $1 AND owner_id = $2';
    await pool.query(deleteQuery, [projectId, userId]);
    
    logger.info('Project deleted successfully', {
      projectId,
      projectName,
      userId,
      requestId: (req as any).requestId
    });
    
    res.json({
      success: true,
      message: 'Project deleted successfully'
    });
  } catch (error) {
    logger.error('Failed to delete project', {
      projectId: req.params.projectId,
      userId: req.user?.userId,
      error: error instanceof Error ? error.message : 'Unknown error',
      requestId: (req as any).requestId
    });
    
    if (error instanceof NotFoundError) {
      res.status(404).json({
        success: false,
        message: error.message
      });
    } else {
      res.status(500).json({
        success: false,
        message: 'Failed to delete project',
        error: error instanceof Error ? error.message : 'Unknown error'
      });
    }
  }
});

export default router; 