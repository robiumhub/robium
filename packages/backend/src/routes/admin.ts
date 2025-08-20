import { Router } from 'express';
import { adminMiddleware } from '../middleware/auth';

const router = Router();

// GET /api/admin/dashboard
router.get('/dashboard', adminMiddleware, async (req, res) => {
  try {
    // TODO: Implement admin dashboard
    res.json({
      success: true,
      data: {
        totalUsers: 10,
        totalProjects: 25,
        templates: 5,
      },
    });
  } catch (error) {
    res.status(500).json({
      success: false,
      error: 'Failed to fetch admin dashboard',
    });
  }
});

export default router;
