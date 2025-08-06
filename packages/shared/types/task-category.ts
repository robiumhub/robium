import { MetaCategoryId } from './meta-category';

export interface TaskCategoryHierarchy {
  level: number; // 1 = top level within meta-category, 5 = most specific
  parent?: string; // Parent task category ID
  children: string[]; // Child task category IDs
  order: number; // Display order within the same level
}

export interface Algorithm {
  id: string;
  name: string;
  description: string;
  version: string;
  complexity: 'basic' | 'intermediate' | 'advanced' | 'expert';
  packages: string[];
  dependencies?: string[];
  parameters?: Record<string, unknown>;
  tags?: string[];
  status?: 'stable' | 'beta' | 'experimental' | 'deprecated';
}

export interface TaskCategoryRequirements {
  hardware?: string[];
  software?: string[];
  rosVersion?: string[];
}

export interface TaskCategoryMetadata {
  createdAt?: string;
  updatedAt?: string;
  version?: string;
  maintainer?: string;
}

export interface TaskCategory {
  id: string;
  metaCategoryId: MetaCategoryId;
  name: string;
  description: string;
  icon: string;
  hierarchy: TaskCategoryHierarchy;
  algorithms: Algorithm[];
  requirements?: TaskCategoryRequirements;
  metadata?: TaskCategoryMetadata;
}

// Example task categories for different meta-categories
export const EXAMPLE_TASK_CATEGORIES: TaskCategory[] = [
  // Actuator Control Task Categories
  {
    id: 'motor_control',
    metaCategoryId: 'actuator',
    name: 'Motor Control',
    description:
      'Basic motor control algorithms for DC motors, stepper motors, and servo motors',
    icon: 'settings_input_component',
    hierarchy: {
      level: 1,
      children: [
        'dc_motor_control',
        'stepper_motor_control',
        'servo_motor_control',
      ],
      order: 1,
    },
    algorithms: [
      {
        id: 'pid_motor_control',
        name: 'PID Motor Control',
        description:
          'Proportional-Integral-Derivative controller for motor speed and position control',
        version: '1.0.0',
        complexity: 'intermediate',
        packages: ['control_toolbox', 'pid_controller'],
        parameters: {
          kp: { type: 'float', default: 1.0, description: 'Proportional gain' },
          ki: { type: 'float', default: 0.1, description: 'Integral gain' },
          kd: { type: 'float', default: 0.01, description: 'Derivative gain' },
        },
        tags: ['pid', 'control', 'motor'],
        status: 'stable',
      },
      {
        id: 'velocity_control',
        name: 'Velocity Control',
        description: 'Simple velocity control for motors with feedback',
        version: '1.0.0',
        complexity: 'basic',
        packages: ['motor_control'],
        tags: ['velocity', 'control', 'motor'],
        status: 'stable',
      },
    ],
    requirements: {
      hardware: ['motor', 'encoder'],
      software: ['ros2_control'],
      rosVersion: ['humble', 'iron'],
    },
  },
  {
    id: 'dc_motor_control',
    metaCategoryId: 'actuator',
    name: 'DC Motor Control',
    description: 'Specialized control algorithms for DC motors',
    icon: 'settings_input_component',
    hierarchy: {
      level: 2,
      parent: 'motor_control',
      children: [],
      order: 1,
    },
    algorithms: [
      {
        id: 'h_bridge_control',
        name: 'H-Bridge Control',
        description:
          'H-bridge motor driver control for bidirectional DC motor control',
        version: '1.0.0',
        complexity: 'basic',
        packages: ['motor_driver', 'h_bridge'],
        tags: ['h-bridge', 'dc-motor', 'bidirectional'],
        status: 'stable',
      },
    ],
  },

  // Camera Systems Task Categories
  {
    id: 'camera_calibration',
    metaCategoryId: 'camera',
    name: 'Camera Calibration',
    description:
      'Camera calibration algorithms for intrinsic and extrinsic parameters',
    icon: 'camera_alt',
    hierarchy: {
      level: 1,
      children: [
        'intrinsic_calibration',
        'extrinsic_calibration',
        'stereo_calibration',
      ],
      order: 1,
    },
    algorithms: [
      {
        id: 'chessboard_calibration',
        name: 'Chessboard Calibration',
        description: 'Camera calibration using chessboard pattern',
        version: '1.0.0',
        complexity: 'intermediate',
        packages: ['camera_calibration', 'opencv'],
        parameters: {
          board_size: {
            type: 'array',
            default: [9, 6],
            description: 'Chessboard size',
          },
          square_size: {
            type: 'float',
            default: 0.025,
            description: 'Square size in meters',
          },
        },
        tags: ['calibration', 'chessboard', 'opencv'],
        status: 'stable',
      },
    ],
    requirements: {
      hardware: ['camera', 'calibration_pattern'],
      software: ['opencv'],
      rosVersion: ['humble', 'iron'],
    },
  },

  // Navigation Task Categories
  {
    id: 'path_planning',
    metaCategoryId: 'planning',
    name: 'Path Planning',
    description: 'Path planning algorithms for robot navigation',
    icon: 'timeline',
    hierarchy: {
      level: 1,
      children: ['global_planning', 'local_planning', 'reactive_planning'],
      order: 1,
    },
    algorithms: [
      {
        id: 'a_star_planner',
        name: 'A* Path Planner',
        description: 'A* algorithm for optimal path planning in grid maps',
        version: '1.0.0',
        complexity: 'intermediate',
        packages: ['nav2_planner', 'nav2_core'],
        parameters: {
          heuristic_weight: {
            type: 'float',
            default: 1.0,
            description: 'Heuristic weight',
          },
          use_dijkstra: {
            type: 'bool',
            default: false,
            description: 'Use Dijkstra instead of A*',
          },
        },
        tags: ['a-star', 'path-planning', 'optimal'],
        status: 'stable',
      },
      {
        id: 'rrt_planner',
        name: 'RRT Path Planner',
        description:
          'Rapidly-exploring Random Tree algorithm for path planning',
        version: '1.0.0',
        complexity: 'advanced',
        packages: ['ompl', 'rrt_planner'],
        tags: ['rrt', 'sampling-based', 'path-planning'],
        status: 'stable',
      },
    ],
    requirements: {
      hardware: ['lidar', 'odometry'],
      software: ['nav2'],
      rosVersion: ['humble', 'iron'],
    },
  },

  // Localization Task Categories
  {
    id: 'amcl',
    metaCategoryId: 'localization',
    name: 'AMCL Localization',
    description: 'Adaptive Monte Carlo Localization algorithms',
    icon: 'my_location',
    hierarchy: {
      level: 1,
      children: ['particle_filter', 'pose_estimation'],
      order: 1,
    },
    algorithms: [
      {
        id: 'amcl_basic',
        name: 'Basic AMCL',
        description: 'Standard Adaptive Monte Carlo Localization',
        version: '1.0.0',
        complexity: 'intermediate',
        packages: ['amcl', 'nav2_amcl'],
        parameters: {
          min_particles: {
            type: 'int',
            default: 500,
            description: 'Minimum particles',
          },
          max_particles: {
            type: 'int',
            default: 5000,
            description: 'Maximum particles',
          },
        },
        tags: ['amcl', 'localization', 'particle-filter'],
        status: 'stable',
      },
    ],
    requirements: {
      hardware: ['lidar', 'odometry'],
      software: ['nav2'],
      rosVersion: ['humble', 'iron'],
    },
  },

  // Mapping Task Categories
  {
    id: 'occupancy_grid',
    metaCategoryId: 'mapping',
    name: 'Occupancy Grid Mapping',
    description:
      'Occupancy grid mapping algorithms for environment representation',
    icon: 'map',
    hierarchy: {
      level: 1,
      children: ['gmapping', 'cartographer', 'rtabmap'],
      order: 1,
    },
    algorithms: [
      {
        id: 'gmapping',
        name: 'GMapping',
        description: 'Grid-based FastSLAM algorithm for occupancy grid mapping',
        version: '1.0.0',
        complexity: 'intermediate',
        packages: ['gmapping', 'slam_gmapping'],
        parameters: {
          map_update_interval: {
            type: 'float',
            default: 5.0,
            description: 'Map update interval',
          },
          maxUrange: {
            type: 'float',
            default: 6.0,
            description: 'Maximum usable range',
          },
        },
        tags: ['gmapping', 'slam', 'occupancy-grid'],
        status: 'stable',
      },
    ],
    requirements: {
      hardware: ['lidar', 'odometry'],
      software: ['gmapping'],
      rosVersion: ['humble', 'iron'],
    },
  },
];

export type TaskCategoryId = (typeof EXAMPLE_TASK_CATEGORIES)[number]['id'];
export type AlgorithmId = Algorithm['id'];
export type AlgorithmComplexity = Algorithm['complexity'];
export type AlgorithmStatus = NonNullable<Algorithm['status']>;
