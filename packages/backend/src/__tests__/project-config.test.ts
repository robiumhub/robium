import { schemaValidator } from '@robium/shared';

describe('Project Configuration Schema Validation', () => {
  test('should validate correct project config', () => {
    const validConfig = {
      name: 'Test Project',
      description: 'A test project',
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

    const result = schemaValidator.validateProjectConfig(validConfig);
    expect(result.valid).toBe(true);
    expect(result.errors).toHaveLength(0);
  });

  test('should reject invalid robot target', () => {
    const invalidConfig = {
      name: 'Test Project',
      robotTarget: 'invalid_robot',
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

    const result = schemaValidator.validateProjectConfig(invalidConfig);
    expect(result.valid).toBe(false);
    expect(result.errors.length).toBeGreaterThan(0);
  });

  test('should reject missing required fields', () => {
    const invalidConfig = {
      name: 'Test Project',
      // Missing required fields
      foxglove: false,
    };

    const result = schemaValidator.validateProjectConfig(invalidConfig);
    expect(result.valid).toBe(false);
    expect(result.errors.length).toBeGreaterThan(0);
  });

  test('should accept valid boolean flags', () => {
    const validConfig = {
      name: 'Test Project',
      robotTarget: 'turtlebot4',
      simulation: 'gazebo',
      rmw: 'fastrtps',
      execution: 'real',
      deployment: 'cloud_gpu',
      baseImage: 'cuda_ubuntu2204',
      rosVersion: 'iron',
      foxglove: true,
      teleopKeyboard: true,
      teleopJoystick: true,
    };

    const result = schemaValidator.validateProjectConfig(validConfig);
    expect(result.valid).toBe(true);
    expect(result.errors).toHaveLength(0);
  });
});
