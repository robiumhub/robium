export interface Template {
  id: string;
  name: string;
  description: string;
  summary: string; // 1-line description
  tags: string[];
  category: string;
  type: string;
  version: string;
  author: string;
  maintainer_email: string;
  license: string;
  is_active: boolean;
  is_public: boolean;
  is_template: boolean;
  config: any; // Project configuration
  metadata: TemplateMetadata;
  module_count: number;
  package_count: number;
  created_at: string;
  updated_at: string;
  created_by: string;
  updated_by: string;
  // Stats (to be added)
  installs_7d?: number;
  rating?: number;
  rating_count?: number;
  // Visual assets (to be added)
  thumbnail_url?: string;
  gif_url?: string;
  // Badges
  is_official?: boolean;
  is_verified?: boolean;
  requires_gpu?: boolean;
  is_new?: boolean;
}

export interface TemplateMetadata {
  use_cases?: string[]; // e.g., ["Autonomous Navigation", "Object Tracking"]
  capabilities?: string[]; // e.g., ["navigation", "slam", "perception"]
  robot_targets?: string[]; // e.g., ["TurtleBot4", "UR5"]
  simulators?: string[]; // e.g., ["Gazebo", "Isaac"]
  ros_distros?: string[]; // e.g., ["humble", "iron"]
  rmw_implementations?: string[]; // e.g., ["cyclonedds", "fastrtps"]
  difficulty?: 'beginner' | 'intermediate' | 'advanced';
  estimated_runtime?: string; // e.g., "5-10 minutes"
  hardware_requirements?: string[]; // e.g., ["GPU", "Camera"]
  dependencies?: string[]; // e.g., ["CUDA 11.8", "ROS2 Humble"]
}

export interface TemplateFilters {
  use_cases: string[];
  capabilities: string[];
  robot_targets: string[];
  simulators: string[];
  ros_distros: string[];
  rmw_implementations: string[];
  tags: string[];
  requires_gpu: boolean;
  official_only: boolean;
  verified_only: boolean;
  licenses: string[];
  difficulty: string[];
}

export interface TemplateStats {
  total_templates: number;
  use_case_counts: Record<string, number>;
  capability_counts: Record<string, number>;
  robot_target_counts: Record<string, number>;
  simulator_counts: Record<string, number>;
  ros_distro_counts: Record<string, number>;
  rmw_counts: Record<string, number>;
  tag_counts: Record<string, number>;
  license_counts: Record<string, number>;
}

export const DEFAULT_TEMPLATE_FILTERS: TemplateFilters = {
  use_cases: [],
  capabilities: [],
  robot_targets: [],
  simulators: [],
  ros_distros: [],
  rmw_implementations: [],
  tags: [],
  requires_gpu: false,
  official_only: false,
  verified_only: false,
  licenses: [],
  difficulty: [],
};

export const USE_CASES = [
  'Autonomous Navigation',
  'Object Tracking',
  'Telepresence',
  'Manipulation',
  'SLAM',
  'Computer Vision',
  'Speech Recognition',
  'Path Planning',
  'Obstacle Avoidance',
  'Multi-Robot Coordination',
  'Simulation',
  'Testing',
  'Education',
  'Research',
] as const;

export const CAPABILITIES = [
  'navigation',
  'slam',
  'perception',
  'manipulation',
  'stt',
  'tts',
  'control',
  'planning',
  'localization',
  'mapping',
  'vision',
  'audio',
  'sensors',
  'actuators',
] as const;

export const ROBOT_TARGETS = [
  'TurtleBot4',
  'UR5',
  'PR2',
  'Baxter',
  'Pepper',
  'NAO',
  'Custom',
  'Simulation Only',
] as const;

export const SIMULATORS = [
  'Gazebo',
  'Isaac',
  'Web Viewer',
  'RViz',
  'Foxglove',
  'None',
] as const;

export const ROS_DISTROS = ['humble', 'iron', 'jazzy', 'rolling'] as const;

export const RMW_IMPLEMENTATIONS = [
  'cyclonedds',
  'fastrtps',
  'opensplice',
] as const;

export const LICENSES = [
  'MIT',
  'Apache-2.0',
  'GPL-3.0',
  'BSD-3-Clause',
  'CC-BY-4.0',
  'Proprietary',
] as const;

export const DIFFICULTY_LEVELS = [
  'beginner',
  'intermediate',
  'advanced',
] as const;
