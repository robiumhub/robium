import { ProjectMetadata } from './project-metadata';

export interface ProjectConfig {
  name: string;
  description?: string;

  robotTarget: 'none' | 'turtlebot4' | 'ur5';
  simulation: 'none' | 'gazebo' | 'isaac';
  rmw: 'cyclonedds' | 'fastrtps';
  execution: 'simulator' | 'real' | 'hybrid';
  deployment: 'local' | 'cloud_gpu' | 'edge';
  baseImage: 'ros_humble' | 'cuda_ubuntu2204' | 'jetson_l4t_ros';
  rosVersion: 'humble' | 'iron' | 'jazzy';

  foxglove: boolean;
  teleopKeyboard: boolean;
  teleopJoystick: boolean;
}

export interface CreateProjectRequest {
  name: string;
  description?: string;
  config: ProjectConfig;
}

export interface Project {
  id: string;
  name: string;
  description?: string;
  config: ProjectConfig;
  createdAt: string;
  updatedAt: string;
  ownerId: string;
}
