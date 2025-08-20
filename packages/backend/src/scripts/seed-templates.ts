import { Database } from '../utils/database';

async function main() {
  try {
    await Database.connect();
    const db = Database.getDatabase();

    // Insert sample templates
    const insertTemplate = db.prepare(`
      INSERT OR REPLACE INTO projects (
        id, name, description, owner_id, is_active, is_template, 
        tags, config, metadata, template_visibility, template_version,
        template_published_at, created_at, updated_at
      )
      VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    `);

    const sampleTemplates = [
      // Admin templates (owner_id: '1')
      {
        id: '6',
        name: 'Advanced SLAM Template',
        description: 'Complete SLAM system template with mapping and localization',
        ownerId: '1',
        isActive: 1,
        isTemplate: 1,
        tags: JSON.stringify(['slam', 'mapping', 'localization', 'advanced', 'template']),
        config: JSON.stringify({
          ros_version: 'noetic',
          robot_type: 'turtlebot',
          slam_algorithm: 'gmapping',
        }),
        metadata: JSON.stringify({
          useCases: ['mapping', 'localization'],
          capabilities: ['slam', 'path_planning', 'obstacle_avoidance'],
          robots: ['turtlebot3', 'pioneer3'],
          simulators: ['gazebo', 'rviz'],
        }),
        templateVisibility: 'public',
        templateVersion: '2.1.0',
        templatePublishedAt: new Date().toISOString(),
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
      },
      {
        id: '7',
        name: 'Computer Vision Starter',
        description: 'Template for computer vision projects with OpenCV and TensorFlow',
        ownerId: '1',
        isActive: 1,
        isTemplate: 1,
        tags: JSON.stringify(['computer-vision', 'opencv', 'tensorflow', 'template']),
        config: JSON.stringify({
          ros_version: 'noetic',
          cv_library: 'opencv',
          ml_framework: 'tensorflow',
        }),
        metadata: JSON.stringify({
          useCases: ['object_detection', 'image_processing'],
          capabilities: ['image_processing', 'ml_inference', 'camera_control'],
          robots: ['turtlebot3', 'custom_camera'],
          simulators: ['gazebo', 'rviz'],
        }),
        templateVisibility: 'public',
        templateVersion: '1.5.0',
        templatePublishedAt: new Date().toISOString(),
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
      },
      {
        id: '8',
        name: 'Multi-Robot Swarm',
        description: 'Template for coordinating multiple robots in a swarm',
        ownerId: '1',
        isActive: 1,
        isTemplate: 1,
        tags: JSON.stringify(['multi-robot', 'swarm', 'coordination', 'template']),
        config: JSON.stringify({
          ros_version: 'noetic',
          robot_count: 3,
          coordination_type: 'swarm',
        }),
        metadata: JSON.stringify({
          useCases: ['swarm_robotics', 'formation_control'],
          capabilities: ['multi_agent_control', 'formation_control', 'collision_avoidance'],
          robots: ['turtlebot3', 'crazyflie'],
          simulators: ['gazebo', 'rviz'],
        }),
        templateVisibility: 'public',
        templateVersion: '1.2.0',
        templatePublishedAt: new Date().toISOString(),
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
      },
      // User templates (owner_id: '2')
      {
        id: '9',
        name: 'Beginner Line Follower',
        description: 'Simple template for line following robots - perfect for beginners',
        ownerId: '2',
        isActive: 1,
        isTemplate: 1,
        tags: JSON.stringify(['line-follower', 'beginner', 'simple', 'template']),
        config: JSON.stringify({
          ros_version: 'noetic',
          robot_type: 'arduino',
          complexity: 'beginner',
        }),
        metadata: JSON.stringify({
          useCases: ['line_following'],
          capabilities: ['sensor_reading', 'motor_control'],
          robots: ['arduino_robot'],
          simulators: ['arduino_sim'],
        }),
        templateVisibility: 'public',
        templateVersion: '1.0.0',
        templatePublishedAt: new Date().toISOString(),
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
      },
      {
        id: '10',
        name: 'Weather Monitoring Bot',
        description: 'Template for autonomous weather monitoring robots',
        ownerId: '2',
        isActive: 1,
        isTemplate: 1,
        tags: JSON.stringify(['weather', 'monitoring', 'autonomous', 'template']),
        config: JSON.stringify({
          ros_version: 'noetic',
          robot_type: 'custom',
          sensors: ['temperature', 'humidity', 'pressure'],
        }),
        metadata: JSON.stringify({
          useCases: ['weather_monitoring', 'data_collection'],
          capabilities: ['sensor_data_collection', 'autonomous_navigation'],
          robots: ['custom_weather_robot'],
          simulators: ['gazebo'],
        }),
        templateVisibility: 'public',
        templateVersion: '1.1.0',
        templatePublishedAt: new Date().toISOString(),
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
      },
    ];

    // Insert all templates
    for (const template of sampleTemplates) {
      insertTemplate.run(
        template.id,
        template.name,
        template.description,
        template.ownerId,
        template.isActive,
        template.isTemplate,
        template.tags,
        template.config,
        template.metadata,
        template.templateVisibility,
        template.templateVersion,
        template.templatePublishedAt,
        template.createdAt,
        template.updatedAt
      );
    }

    console.log('Sample templates seeded successfully');
  } catch (error) {
    console.error('Failed to seed templates:', error);
    process.exit(1);
  } finally {
    Database.close();
  }
}

main();
