import express from 'express';
import { authenticateToken } from '../middleware/auth';
import { requirePermission } from '../middleware/rbac';
import { Permission } from '../utils/permissions';
import { pool } from '../utils/database';
import { logger } from '../utils/logger';
import { ConflictError, NotFoundError, ValidationError } from '../utils/errors';
import { AuthRequest } from '../types';
import { dockerfileGenerationService } from '../services/DockerfileGenerationService';

const router = express.Router();

/**
 * GET /dockerfiles/:projectId
 * Get the current Dockerfile for a project
 */
router.get(
  '/:projectId',
  authenticateToken,
  requirePermission(Permission.PROJECT_READ),
  async (req: AuthRequest, res) => {
    try {
      const { projectId } = req.params;
      const userId = req.user?.userId;

      // Verify project ownership and get project config
      const projectResult = await pool.query(
        'SELECT id, name, owner_id, config FROM projects WHERE id = $1',
        [projectId]
      );

      if (projectResult.rows.length === 0) {
        throw new NotFoundError('Project not found');
      }

      const project = projectResult.rows[0];
      if (project.owner_id !== userId) {
        throw new ValidationError(
          'Access denied: You can only view Dockerfiles for your own projects'
        );
      }

      // Get the generated Dockerfile
      const dockerfileResult =
        dockerfileGenerationService.getGeneratedDockerfile(projectId);

      if (!dockerfileResult) {
        // Generate a new Dockerfile using stored configuration
        try {
          // Check if project has new config schema
          if (project.config && project.config.robotTarget !== undefined) {
            // Use new project config schema
            const generatedResult =
              await dockerfileGenerationService.generateFromProjectConfig(
                projectId,
                project.config,
                {
                  includeComments: true,
                  securityScan: true,
                  optimize: true,
                }
              );

            return res.json({
              success: true,
              data: {
                content: generatedResult.content,
                path: generatedResult.path,
                errors: generatedResult.errors,
                warnings: generatedResult.warnings,
                securityIssues: generatedResult.securityIssues,
                buildTime: generatedResult.buildTime,
                size: generatedResult.size,
                projectName: project.name,
              },
            });
          } else if (project.type) {
            // Use legacy configuration if available
            const projectConfig = {
              id: project.id,
              name: project.name,
              version: project.version || '1.0.0',
              description: project.description,
              type: project.type,
              baseImage: project.base_image,
              workdir: project.workdir,
              systemDependencies: project.system_dependencies || [],
              pythonDependencies: project.python_dependencies || [],
              nodeDependencies: project.node_dependencies || [],
              environmentVariables: project.environment_variables || {},
              ports: project.ports || [],
              volumes: project.volumes || [],
              command: project.command,
            };

            // Register the configuration if not already registered
            dockerfileGenerationService.registerProjectConfig(projectConfig);

            const generatedResult =
              await dockerfileGenerationService.generateDockerfile(projectId, {
                includeComments: true,
                securityScan: true,
              });

            return res.json({
              success: true,
              data: {
                content: generatedResult.content,
                path: generatedResult.path,
                errors: generatedResult.errors,
                warnings: generatedResult.warnings,
                securityIssues: generatedResult.securityIssues,
                buildTime: generatedResult.buildTime,
                size: generatedResult.size,
                projectName: project.name,
              },
            });
          } else {
            // Fallback to basic template if no configuration exists
            const dockerfileContent = `# Generated Dockerfile for ${
              project.name
            }
# Project: ${project.name}
# Type: Python
# Generated: ${new Date().toISOString()}

FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \\
    build-essential \\
    git \\
    && rm -rf /var/lib/apt/lists/*

# Set Python environment
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1

# Copy requirements (if they exist)
COPY requirements.txt* ./
RUN pip install --no-cache-dir -r requirements.txt 2>/dev/null || echo "No requirements.txt found"

# Copy application code
COPY . .

# Set environment variables
ENV FLASK_ENV=development

# Expose port
EXPOSE 5000

# Default command
CMD ["python", "app.py"]`;

            return res.json({
              success: true,
              data: {
                content: dockerfileContent,
                path: '',
                errors: [],
                warnings: [
                  'Using basic template - configure project settings for custom generation',
                ],
                securityIssues: [],
                buildTime: 0,
                size: dockerfileContent.length,
                projectName: project.name,
              },
            });
          }
        } catch (generateError) {
          logger.warn(
            `Failed to generate Dockerfile for project ${projectId}: ${
              generateError instanceof Error
                ? generateError.message
                : 'Unknown error'
            }`
          );

          // Return a basic Dockerfile template
          const basicDockerfile = `# Dockerfile for ${project.name}
FROM python:3.11-slim
WORKDIR /app
COPY . .
CMD ["python", "app.py"]`;

          return res.json({
            success: true,
            data: {
              content: basicDockerfile,
              path: '',
              errors: [
                generateError instanceof Error
                  ? generateError.message
                  : 'Unknown error',
              ],
              warnings: ['Using fallback template due to generation error'],
              securityIssues: [],
              buildTime: 0,
              size: basicDockerfile.length,
              projectName: project.name,
            },
          });
        }
      }

      // Return existing Dockerfile
      res.json({
        success: true,
        data: {
          content: dockerfileResult.content,
          path: dockerfileResult.path,
          errors: dockerfileResult.errors,
          warnings: dockerfileResult.warnings,
          securityIssues: dockerfileResult.securityIssues,
          buildTime: dockerfileResult.buildTime,
          size: dockerfileResult.size,
          projectName: project.name,
        },
      });
    } catch (error) {
      logger.error(
        `Error fetching Dockerfile: ${
          error instanceof Error ? error.message : 'Unknown error'
        }`
      );
      if (error instanceof NotFoundError || error instanceof ValidationError) {
        return res.status(error.status).json({
          success: false,
          message: error.message,
        });
      }
      res.status(500).json({
        success: false,
        message: 'Failed to fetch Dockerfile',
        error: error instanceof Error ? error.message : 'Unknown error',
      });
    }
  }
);

/**
 * POST /dockerfiles/:projectId/generate
 * Generate a new Dockerfile for a project
 */
router.post(
  '/:projectId/generate',
  authenticateToken,
  requirePermission(Permission.PROJECT_UPDATE),
  async (req: AuthRequest, res) => {
    try {
      const { projectId } = req.params;
      const userId = req.user?.userId;
      const options = req.body || {};

      // Verify project ownership and get project config
      const projectResult = await pool.query(
        'SELECT id, name, owner_id, config FROM projects WHERE id = $1',
        [projectId]
      );

      if (projectResult.rows.length === 0) {
        throw new NotFoundError('Project not found');
      }

      const project = projectResult.rows[0];
      if (project.owner_id !== userId) {
        throw new ValidationError(
          'Access denied: You can only generate Dockerfiles for your own projects'
        );
      }

      let result: any;

      // Check if project has new config schema
      if (project.config && project.config.robotTarget !== undefined) {
        // Use new project config schema
        result = await dockerfileGenerationService.generateFromProjectConfig(
          projectId,
          project.config,
          {
            includeComments: true,
            securityScan: true,
            optimize: true,
            ...options,
          }
        );
      } else {
        // Use legacy generation method
        result = await dockerfileGenerationService.generateDockerfile(
          projectId,
          {
            includeComments: true,
            securityScan: true,
            optimize: true,
            ...options,
          }
        );
      }

      logger.info(
        `Dockerfile generated for project ${projectId} by user ${userId}`
      );

      res.json({
        success: true,
        data: {
          content: result.content,
          path: result.path,
          errors: result.errors,
          warnings: result.warnings,
          securityIssues: result.securityIssues,
          buildTime: result.buildTime,
          size: result.size,
          projectName: project.name,
        },
      });
    } catch (error) {
      logger.error(
        `Error generating Dockerfile: ${
          error instanceof Error ? error.message : 'Unknown error'
        }`
      );
      if (error instanceof NotFoundError || error instanceof ValidationError) {
        return res.status(error.status).json({
          success: false,
          message: error.message,
        });
      }
      res.status(500).json({
        success: false,
        message: 'Internal server error',
      });
    }
  }
);

/**
 * DELETE /dockerfiles/:projectId
 * Remove the generated Dockerfile for a project
 */
router.delete(
  '/:projectId',
  authenticateToken,
  requirePermission(Permission.PROJECT_UPDATE),
  async (req: AuthRequest, res) => {
    try {
      const { projectId } = req.params;
      const userId = req.user?.userId;

      // Verify project ownership
      const projectResult = await pool.query(
        'SELECT id, name, owner_id FROM projects WHERE id = $1',
        [projectId]
      );

      if (projectResult.rows.length === 0) {
        throw new NotFoundError('Project not found');
      }

      const project = projectResult.rows[0];
      if (project.owner_id !== userId) {
        throw new ValidationError(
          'Access denied: You can only remove Dockerfiles for your own projects'
        );
      }

      // Remove the generated Dockerfile
      const removed =
        dockerfileGenerationService.removeGeneratedDockerfile(projectId);

      if (removed) {
        logger.info(
          `Dockerfile removed for project ${projectId} by user ${userId}`
        );
        res.json({
          success: true,
          message: 'Dockerfile removed successfully',
        });
      } else {
        res.json({
          success: true,
          message: 'No Dockerfile found to remove',
        });
      }
    } catch (error) {
      logger.error(
        `Error removing Dockerfile: ${
          error instanceof Error ? error.message : 'Unknown error'
        }`
      );
      if (error instanceof NotFoundError || error instanceof ValidationError) {
        return res.status(error.status).json({
          success: false,
          message: error.message,
        });
      }
      res.status(500).json({
        success: false,
        message: 'Internal server error',
      });
    }
  }
);

export default router;
