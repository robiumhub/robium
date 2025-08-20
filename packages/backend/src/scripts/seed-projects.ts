import { Database } from '../utils/database';

async function main() {
  try {
    await Database.connect();
    const db = Database.getDatabase();

    // Insert sample projects
    const insertProject = db.prepare(`
      INSERT OR REPLACE INTO projects (
        id, name, description, owner_id, is_active, is_template, 
        tags, config, metadata, created_at, updated_at
      )
      VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    `);

    const sampleProjects = [
      // Admin projects (owner_id: '1')
      {
        id: '1',
        name: 'Advanced Navigation System',
        description: 'A sophisticated navigation system with SLAM and path planning',
        ownerId: '1',
        isActive: 1,
        isTemplate: 0,
        tags: JSON.stringify(['navigation', 'slam', 'advanced', 'robotics']),
        config: JSON.stringify({
          ros_version: 'noetic',
          robot_type: 'turtlebot',
          features: ['slam', 'path_planning'],
        }),
        metadata: JSON.stringify({
          useCases: ['navigation', 'mapping'],
          capabilities: ['slam', 'localization', 'path_planning'],
          robots: ['turtlebot3', 'pioneer3'],
          simulators: ['gazebo', 'rviz'],
        }),
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
      },
      {
        id: '2',
        name: 'Computer Vision Pipeline',
        description: 'Real-time object detection and tracking system',
        ownerId: '1',
        isActive: 1,
        isTemplate: 0,
        tags: JSON.stringify(['computer-vision', 'object-detection', 'tracking']),
        config: JSON.stringify({
          ros_version: 'noetic',
          cv_library: 'opencv',
          ml_framework: 'tensorflow',
        }),
        metadata: JSON.stringify({
          useCases: ['object_detection', 'tracking'],
          capabilities: ['image_processing', 'ml_inference'],
          robots: ['turtlebot3', 'custom_camera'],
          simulators: ['gazebo', 'rviz'],
        }),
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
      },
      {
        id: '3',
        name: 'Multi-Robot Coordination',
        description: 'Coordinated control system for multiple robots',
        ownerId: '1',
        isActive: 1,
        isTemplate: 0,
        tags: JSON.stringify(['multi-robot', 'coordination', 'swarm']),
        config: JSON.stringify({
          ros_version: 'noetic',
          robot_count: 3,
          coordination_type: 'swarm',
        }),
        metadata: JSON.stringify({
          useCases: ['swarm_robotics', 'coordination'],
          capabilities: ['multi_agent_control', 'formation_control'],
          robots: ['turtlebot3', 'crazyflie'],
          simulators: ['gazebo', 'rviz'],
        }),
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
      },
      // User projects (owner_id: '2')
      {
        id: '4',
        name: 'Basic Line Follower',
        description: 'Simple line following robot project for beginners',
        ownerId: '2',
        isActive: 1,
        isTemplate: 0,
        tags: JSON.stringify(['line-follower', 'beginner', 'simple']),
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
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
      },
      {
        id: '5',
        name: 'Weather Station Robot',
        description: 'Autonomous weather monitoring robot',
        ownerId: '2',
        isActive: 1,
        isTemplate: 0,
        tags: JSON.stringify(['weather', 'monitoring', 'autonomous']),
        config: JSON.stringify({
          ros_version: 'noetic',
          robot_type: 'custom',
          sensors: ['temperature', 'humidity', 'pressure'],
        }),
        metadata: JSON.stringify({
          useCases: ['weather_monitoring'],
          capabilities: ['sensor_data_collection', 'autonomous_navigation'],
          robots: ['custom_weather_robot'],
          simulators: ['gazebo'],
        }),
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
      },
    ];

    // Insert all projects
    for (const project of sampleProjects) {
      insertProject.run(
        project.id,
        project.name,
        project.description,
        project.ownerId,
        project.isActive,
        project.isTemplate,
        project.tags,
        project.config,
        project.metadata,
        project.createdAt,
        project.updatedAt
      );
    }

    console.log('Sample projects seeded successfully');
  } catch (error) {
    console.error('Failed to seed projects:', error);
    process.exit(1);
  } finally {
    Database.close();
  }
}

main();
