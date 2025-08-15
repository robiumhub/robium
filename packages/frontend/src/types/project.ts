export type NewProjectForm = {
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
};

export const defaultNewProjectForm: NewProjectForm = {
  name: '',
  description: '',

  robotTarget: 'none',
  simulation: 'none',
  rmw: 'cyclonedds',
  execution: 'simulator',
  deployment: 'local',
  baseImage: 'ros_humble',
  rosVersion: 'humble',

  foxglove: false,
  teleopKeyboard: false,
  teleopJoystick: false,
};

export function validateConstraints(f: NewProjectForm): string[] {
  const errs: string[] = [];
  if (f.simulation === 'isaac' && f.baseImage !== 'cuda_ubuntu2204')
    errs.push('Isaac requires a CUDA base image.');
  if (f.execution === 'real' && f.simulation !== 'none')
    errs.push('Real execution cannot auto-start a simulator.');
  if (f.teleopJoystick && f.deployment === 'cloud_gpu')
    errs.push(
      'Joystick needs host device mapping; unsupported in cloud preset.'
    );
  return errs;
}
