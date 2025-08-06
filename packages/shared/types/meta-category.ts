export interface MetaCategoryHierarchy {
  level: number; // 1 = top level, 5 = most specific
  parent?: string; // Parent meta-category ID
  children: string[]; // Child meta-category IDs
}

export interface MetaCategoryClassification {
  primaryFunction: string;
  complexity: 'basic' | 'intermediate' | 'advanced' | 'expert';
  useCase: string[];
  robotType: string[];
}

export interface MetaCategoryTaxonomy {
  keywords: string[];
  aliases: string[];
  relatedCategories: string[];
}

export interface MetaCategoryConstraints {
  rosVersion: string[];
  hardwareRequirements: string[];
  dependencies: string[];
}

export interface MetaCategoryMetadata {
  createdAt: string;
  updatedAt: string;
  version: string;
  maintainer: string;
}

export interface MetaCategory {
  id: string;
  name: string;
  description: string;
  icon: string;
  color: string;
  scope: 'core' | 'advanced' | 'experimental' | 'deprecated';
  hierarchy: MetaCategoryHierarchy;
  classification: MetaCategoryClassification;
  taxonomy: MetaCategoryTaxonomy;
  constraints?: MetaCategoryConstraints;
  metadata?: MetaCategoryMetadata;
}

// Predefined meta-categories for ROS algorithms
export const ROS_META_CATEGORIES: MetaCategory[] = [
  {
    id: 'actuator',
    name: 'Actuator Control',
    description: 'Control systems for motors, servos, and other actuators',
    icon: 'settings_input_component',
    color: '#2196F3',
    scope: 'core',
    hierarchy: {
      level: 1,
      children: ['motor_control', 'servo_control', 'pneumatic_control', 'hydraulic_control']
    },
    classification: {
      primaryFunction: 'Actuator control and management',
      complexity: 'basic',
      useCase: ['motor control', 'servo positioning', 'actuator feedback'],
      robotType: ['mobile', 'manipulator', 'humanoid', 'aerial']
    },
    taxonomy: {
      keywords: ['actuator', 'motor', 'servo', 'control', 'feedback'],
      aliases: ['motor control', 'servo control'],
      relatedCategories: ['control_advanced', 'manipulation_core']
    }
  },
  {
    id: 'camera',
    name: 'Camera Systems',
    description: 'Camera control, calibration, and image processing',
    icon: 'camera_alt',
    color: '#4CAF50',
    scope: 'core',
    hierarchy: {
      level: 1,
      children: ['camera_calibration', 'image_processing', 'stereo_vision', 'depth_sensing']
    },
    classification: {
      primaryFunction: 'Visual perception and processing',
      complexity: 'intermediate',
      useCase: ['object detection', 'navigation', 'inspection', 'surveillance'],
      robotType: ['mobile', 'manipulator', 'aerial', 'underwater']
    },
    taxonomy: {
      keywords: ['camera', 'vision', 'image', 'calibration', 'stereo'],
      aliases: ['vision', 'imaging'],
      relatedCategories: ['perception_basic', 'sensor_fusion']
    }
  },
  {
    id: 'remote_control',
    name: 'Remote Control',
    description: 'Remote control interfaces and teleoperation',
    icon: 'gamepad',
    color: '#FF9800',
    scope: 'core',
    hierarchy: {
      level: 1,
      children: ['joystick_control', 'keyboard_control', 'mobile_interface', 'vr_control']
    },
    classification: {
      primaryFunction: 'Human-robot interaction and control',
      complexity: 'basic',
      useCase: ['teleoperation', 'manual control', 'testing', 'emergency control'],
      robotType: ['mobile', 'manipulator', 'aerial', 'underwater']
    },
    taxonomy: {
      keywords: ['remote', 'control', 'teleoperation', 'joystick', 'interface'],
      aliases: ['teleoperation', 'manual control'],
      relatedCategories: ['voice_control', 'human_interface']
    }
  },
  {
    id: 'arm_control',
    name: 'Arm Control',
    description: 'Robotic arm control and manipulation',
    icon: 'precision_manufacturing',
    color: '#9C27B0',
    scope: 'core',
    hierarchy: {
      level: 1,
      children: ['inverse_kinematics', 'trajectory_planning', 'gripper_control', 'force_control']
    },
    classification: {
      primaryFunction: 'Manipulator control and coordination',
      complexity: 'advanced',
      useCase: ['pick and place', 'assembly', 'welding', 'inspection'],
      robotType: ['manipulator', 'humanoid', 'mobile_manipulator']
    },
    taxonomy: {
      keywords: ['arm', 'manipulator', 'kinematics', 'trajectory', 'gripper'],
      aliases: ['manipulator control', 'robotic arm'],
      relatedCategories: ['manipulation_core', 'planning_advanced']
    }
  },
  {
    id: 'voice_control',
    name: 'Voice Control',
    description: 'Voice recognition and speech synthesis',
    icon: 'record_voice_over',
    color: '#E91E63',
    scope: 'advanced',
    hierarchy: {
      level: 2,
      children: ['speech_recognition', 'speech_synthesis', 'natural_language', 'voice_commands']
    },
    classification: {
      primaryFunction: 'Voice-based human-robot interaction',
      complexity: 'intermediate',
      useCase: ['voice commands', 'natural language processing', 'accessibility'],
      robotType: ['mobile', 'humanoid', 'service_robot']
    },
    taxonomy: {
      keywords: ['voice', 'speech', 'recognition', 'synthesis', 'nlp'],
      aliases: ['speech control', 'voice interface'],
      relatedCategories: ['remote_control', 'human_interface']
    }
  },
  {
    id: 'mapping',
    name: 'Mapping',
    description: 'Environment mapping and representation',
    icon: 'map',
    color: '#795548',
    scope: 'core',
    hierarchy: {
      level: 1,
      children: ['occupancy_grid', 'feature_map', 'semantic_map', 'topological_map']
    },
    classification: {
      primaryFunction: 'Environment representation and modeling',
      complexity: 'intermediate',
      useCase: ['navigation', 'exploration', 'inspection', 'planning'],
      robotType: ['mobile', 'aerial', 'underwater']
    },
    taxonomy: {
      keywords: ['mapping', 'occupancy', 'grid', 'feature', 'semantic'],
      aliases: ['environment mapping', 'spatial representation'],
      relatedCategories: ['navigation_core', 'localization']
    }
  },
  {
    id: 'localization',
    name: 'Localization',
    description: 'Robot localization and pose estimation',
    icon: 'my_location',
    color: '#607D8B',
    scope: 'core',
    hierarchy: {
      level: 1,
      children: ['amcl', 'ekf_localization', 'particle_filter', 'visual_odometry']
    },
    classification: {
      primaryFunction: 'Position and orientation estimation',
      complexity: 'intermediate',
      useCase: ['navigation', 'tracking', 'positioning', 'odometry'],
      robotType: ['mobile', 'aerial', 'underwater']
    },
    taxonomy: {
      keywords: ['localization', 'pose', 'estimation', 'odometry', 'tracking'],
      aliases: ['pose estimation', 'position tracking'],
      relatedCategories: ['navigation_core', 'sensor_fusion']
    }
  },
  {
    id: 'planning',
    name: 'Planning',
    description: 'Path planning and motion planning',
    icon: 'timeline',
    color: '#3F51B5',
    scope: 'core',
    hierarchy: {
      level: 1,
      children: ['path_planning', 'motion_planning', 'task_planning', 'behavior_planning']
    },
    classification: {
      primaryFunction: 'Motion and task planning',
      complexity: 'advanced',
      useCase: ['navigation', 'manipulation', 'autonomous operation'],
      robotType: ['mobile', 'manipulator', 'humanoid', 'aerial']
    },
    taxonomy: {
      keywords: ['planning', 'path', 'motion', 'trajectory', 'algorithm'],
      aliases: ['motion planning', 'path planning'],
      relatedCategories: ['navigation_core', 'arm_control', 'planning_advanced']
    }
  },
  {
    id: 'person_tracking',
    name: 'Person Tracking',
    description: 'Human detection, tracking, and following',
    icon: 'person_search',
    color: '#FF5722',
    scope: 'advanced',
    hierarchy: {
      level: 2,
      children: ['person_detection', 'person_following', 'gesture_recognition', 'social_navigation']
    },
    classification: {
      primaryFunction: 'Human-robot interaction and safety',
      complexity: 'advanced',
      useCase: ['human following', 'social navigation', 'safety', 'assistance'],
      robotType: ['mobile', 'humanoid', 'service_robot']
    },
    taxonomy: {
      keywords: ['person', 'human', 'tracking', 'detection', 'following'],
      aliases: ['human tracking', 'person following'],
      relatedCategories: ['perception_basic', 'navigation_core']
    }
  },
  {
    id: 'person_recognition',
    name: 'Person Recognition',
    description: 'Face recognition and person identification',
    icon: 'face',
    color: '#00BCD4',
    scope: 'advanced',
    hierarchy: {
      level: 2,
      children: ['face_recognition', 'gait_recognition', 'biometric_identification', 'emotion_detection']
    },
    classification: {
      primaryFunction: 'Person identification and authentication',
      complexity: 'expert',
      useCase: ['security', 'personalization', 'access control', 'social interaction'],
      robotType: ['service_robot', 'humanoid', 'security_robot']
    },
    taxonomy: {
      keywords: ['recognition', 'face', 'identification', 'biometric', 'authentication'],
      aliases: ['face recognition', 'person identification'],
      relatedCategories: ['perception_basic', 'ai_core']
    }
  },
  {
    id: 'character_animation',
    name: 'Character Animation',
    description: 'Robot character animation and expression',
    icon: 'animation',
    color: '#8BC34A',
    scope: 'experimental',
    hierarchy: {
      level: 3,
      children: ['facial_animation', 'body_animation', 'gesture_animation', 'emotion_expression']
    },
    classification: {
      primaryFunction: 'Robot personality and expression',
      complexity: 'expert',
      useCase: ['social interaction', 'entertainment', 'education', 'therapy'],
      robotType: ['humanoid', 'social_robot', 'entertainment_robot']
    },
    taxonomy: {
      keywords: ['animation', 'character', 'expression', 'personality', 'social'],
      aliases: ['robot animation', 'character expression'],
      relatedCategories: ['person_recognition', 'voice_control']
    }
  }
];

export type MetaCategoryId = typeof ROS_META_CATEGORIES[number]['id'];
export type MetaCategoryScope = MetaCategory['scope'];
export type MetaCategoryComplexity = MetaCategoryClassification['complexity']; 