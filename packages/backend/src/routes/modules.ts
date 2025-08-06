import { Router, Response, NextFunction, Request } from 'express';
import { ApiResponse } from '../types';
import { sanitizeInput } from '../middleware/validation';
import * as fs from 'fs';
import * as path from 'path';

const router = Router();

// Apply input sanitization middleware (modules are publicly accessible)
router.use(sanitizeInput);

// Get all modules with pagination
router.get('/', async (req: Request, res: Response, next: NextFunction) => {
  try {
    const modulesPath = path.join(__dirname, '../../../shared/modules');
    const modules: any[] = [];

    // Read all module metadata files
    if (fs.existsSync(modulesPath)) {
      const files = fs.readdirSync(modulesPath);
      
      for (const file of files) {
        if (file.endsWith('.json')) {
          try {
            const filePath = path.join(modulesPath, file);
            const moduleData = JSON.parse(fs.readFileSync(filePath, 'utf8'));
            modules.push(moduleData);
          } catch (error) {
            console.error(`Error reading module file ${file}:`, error);
          }
        }
      }
    }

    // Sort modules by name for consistent ordering
    modules.sort((a, b) => a.name.localeCompare(b.name));

    // Handle pagination
    const page = parseInt(req.query.page as string) || 1;
    const limit = parseInt(req.query.limit as string) || 10;
    const startIndex = (page - 1) * limit;
    const endIndex = startIndex + limit;
    const paginatedModules = modules.slice(startIndex, endIndex);

    const response: ApiResponse = {
      success: true,
      data: {
        modules: paginatedModules,
        pagination: {
          currentPage: page,
          totalPages: Math.ceil(modules.length / limit),
          totalModules: modules.length,
          modulesPerPage: limit,
          hasNextPage: endIndex < modules.length,
          hasPrevPage: page > 1
        }
      },
      message: 'Modules retrieved successfully',
    };

    res.json(response);
  } catch (error) {
    next(error);
  }
});

// Get module by name
router.get('/:name', async (req: Request, res: Response, next: NextFunction) => {
  try {
    const { name } = req.params;
    const modulePath = path.join(__dirname, '../../../shared/modules', `${name}.json`);

    if (!fs.existsSync(modulePath)) {
      return res.status(404).json({
        success: false,
        error: 'Module not found',
      });
    }

    const moduleData = JSON.parse(fs.readFileSync(modulePath, 'utf8'));

    const response: ApiResponse = {
      success: true,
      data: moduleData,
      message: 'Module retrieved successfully',
    };

    res.json(response);
  } catch (error) {
    next(error);
  }
});

// Get modules by category
router.get('/category/:category', async (req: Request, res: Response, next: NextFunction) => {
  try {
    const { category } = req.params;
    const modulesPath = path.join(__dirname, '../../../shared/modules');
    const modules: any[] = [];

    if (fs.existsSync(modulesPath)) {
      const files = fs.readdirSync(modulesPath);
      
      for (const file of files) {
        if (file.endsWith('.json')) {
          try {
            const filePath = path.join(modulesPath, file);
            const moduleData = JSON.parse(fs.readFileSync(filePath, 'utf8'));
            
            if (moduleData.category === category) {
              modules.push(moduleData);
            }
          } catch (error) {
            console.error(`Error reading module file ${file}:`, error);
          }
        }
      }
    }

    const response: ApiResponse = {
      success: true,
      data: modules,
      message: `Modules in category '${category}' retrieved successfully`,
    };

    res.json(response);
  } catch (error) {
    next(error);
  }
});

// Get modules by tags
router.get('/tags/:tags', async (req: Request, res: Response, next: NextFunction) => {
  try {
    const { tags } = req.params;
    const tagList = tags.split(',').map(tag => tag.trim());
    const modulesPath = path.join(__dirname, '../../../shared/modules');
    const modules: any[] = [];

    if (fs.existsSync(modulesPath)) {
      const files = fs.readdirSync(modulesPath);
      
      for (const file of files) {
        if (file.endsWith('.json')) {
          try {
            const filePath = path.join(modulesPath, file);
            const moduleData = JSON.parse(fs.readFileSync(filePath, 'utf8'));
            
            if (moduleData.tags && moduleData.tags.some((tag: string) => tagList.includes(tag))) {
              modules.push(moduleData);
            }
          } catch (error) {
            console.error(`Error reading module file ${file}:`, error);
          }
        }
      }
    }

    const response: ApiResponse = {
      success: true,
      data: modules,
      message: `Modules with tags '${tags}' retrieved successfully`,
    };

    res.json(response);
  } catch (error) {
    next(error);
  }
});

// Get available categories
router.get('/categories/list', async (req: Request, res: Response, next: NextFunction) => {
  try {
    const modulesPath = path.join(__dirname, '../../../shared/modules');
    const categories = new Set<string>();

    if (fs.existsSync(modulesPath)) {
      const files = fs.readdirSync(modulesPath);
      
      for (const file of files) {
        if (file.endsWith('.json')) {
          try {
            const filePath = path.join(modulesPath, file);
            const moduleData = JSON.parse(fs.readFileSync(filePath, 'utf8'));
            
            if (moduleData.category) {
              categories.add(moduleData.category);
            }
          } catch (error) {
            console.error(`Error reading module file ${file}:`, error);
          }
        }
      }
    }

    const response: ApiResponse = {
      success: true,
      data: Array.from(categories),
      message: 'Available categories retrieved successfully',
    };

    res.json(response);
  } catch (error) {
    next(error);
  }
});

// Get available tags
router.get('/tags/list', async (req: Request, res: Response, next: NextFunction) => {
  try {
    const modulesPath = path.join(__dirname, '../../../shared/modules');
    const tags = new Set<string>();

    if (fs.existsSync(modulesPath)) {
      const files = fs.readdirSync(modulesPath);
      
      for (const file of files) {
        if (file.endsWith('.json')) {
          try {
            const filePath = path.join(modulesPath, file);
            const moduleData = JSON.parse(fs.readFileSync(filePath, 'utf8'));
            
            if (moduleData.tags) {
              moduleData.tags.forEach((tag: string) => tags.add(tag));
            }
          } catch (error) {
            console.error(`Error reading module file ${file}:`, error);
          }
        }
      }
    }

    const response: ApiResponse = {
      success: true,
      data: Array.from(tags),
      message: 'Available tags retrieved successfully',
    };

    res.json(response);
  } catch (error) {
    next(error);
  }
});

export default router; 