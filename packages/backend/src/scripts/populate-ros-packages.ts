#!/usr/bin/env ts-node

import { Database } from '../utils/database';

interface ROSPackage {
  name: string;
  version: string;
  description: string;
  category: string;
  type: 'mock' | 'production' | 'custom';
  maintainer_email: string;
  license: string;
  build_dependencies: string[];
  runtime_dependencies: string[];
  test_dependencies: string[];
  build_type: string;
  packages: string[];
  published_topics: string[];
  subscribed_topics: string[];
  advertised_services: string[];
  called_services: string[];
  tags: string[];
  algorithms: string[];
  source_path: string;
  package_xml_path: string;
  cmake_lists_path: string;
}

const mockPackages: ROSPackage[] = [
  {
    name: 'amcl_localization',
    version: '0.0.0',
    description:
      'Mock AMCL (Adaptive Monte Carlo Localization) package for Robium',
    category: 'localization',
    type: 'mock',
    maintainer_email: 'ros@robium.dev',
    license: 'Apache-2.0',
    build_dependencies: ['ament_cmake', 'ament_cmake_python'],
    runtime_dependencies: [
      'rclcpp',
      'std_msgs',
      'geometry_msgs',
      'nav_msgs',
      'tf2',
      'tf2_ros',
    ],
    test_dependencies: ['ament_lint_auto', 'ament_lint_common'],
    build_type: 'ament_cmake',
    packages: ['amcl_localization'],
    published_topics: ['/amcl_localization/package_info', '/amcl_pose'],
    subscribed_topics: [],
    advertised_services: [],
    called_services: [],
    tags: ['localization', 'amcl', 'monte-carlo', 'pose-estimation'],
    algorithms: ['amcl', 'tf2', 'rviz'],
    source_path: 'ros/src/amcl_localization',
    package_xml_path: 'ros/src/amcl_localization/package.xml',
    cmake_lists_path: 'ros/src/amcl_localization/CMakeLists.txt',
  },
  {
    name: 'person_tracking',
    version: '0.0.0',
    description: 'Mock person detection and tracking package for Robium',
    category: 'perception',
    type: 'mock',
    maintainer_email: 'ros@robium.dev',
    license: 'Apache-2.0',
    build_dependencies: ['ament_cmake', 'ament_cmake_python'],
    runtime_dependencies: [
      'rclcpp',
      'std_msgs',
      'geometry_msgs',
      'vision_msgs',
      'sensor_msgs',
    ],
    test_dependencies: ['ament_lint_auto', 'ament_lint_common'],
    build_type: 'ament_cmake',
    packages: ['person_tracking'],
    published_topics: [
      '/person_tracking/package_info',
      '/person_detections',
      '/person_poses',
    ],
    subscribed_topics: [],
    advertised_services: [],
    called_services: [],
    tags: ['perception', 'person-detection', 'tracking', 'computer-vision'],
    algorithms: ['opencv', 'pcl', 'rviz'],
    source_path: 'ros/src/person_tracking',
    package_xml_path: 'ros/src/person_tracking/package.xml',
    cmake_lists_path: 'ros/src/person_tracking/CMakeLists.txt',
  },
  {
    name: 'path_planning',
    version: '0.0.0',
    description: 'Mock path planning and navigation package for Robium',
    category: 'navigation',
    type: 'mock',
    maintainer_email: 'ros@robium.dev',
    license: 'Apache-2.0',
    build_dependencies: ['ament_cmake', 'ament_cmake_python'],
    runtime_dependencies: [
      'rclcpp',
      'std_msgs',
      'geometry_msgs',
      'nav_msgs',
      'tf2',
      'tf2_ros',
    ],
    test_dependencies: ['ament_lint_auto', 'ament_lint_common'],
    build_type: 'ament_cmake',
    packages: ['path_planning'],
    published_topics: [
      '/path_planning/package_info',
      '/planned_path',
      '/cmd_vel',
    ],
    subscribed_topics: [],
    advertised_services: [],
    called_services: [],
    tags: ['navigation', 'path-planning', 'autonomous', 'motion-planning'],
    algorithms: ['nav2', 'tf2', 'rviz'],
    source_path: 'ros/src/path_planning',
    package_xml_path: 'ros/src/path_planning/package.xml',
    cmake_lists_path: 'ros/src/path_planning/CMakeLists.txt',
  },
  {
    name: 'kalman_filter',
    version: '0.0.0',
    description: 'Mock Kalman filter for state estimation in Robium',
    category: 'estimation',
    type: 'mock',
    maintainer_email: 'ros@robium.dev',
    license: 'Apache-2.0',
    build_dependencies: ['ament_cmake', 'ament_cmake_python'],
    runtime_dependencies: [
      'rclcpp',
      'std_msgs',
      'geometry_msgs',
      'nav_msgs',
      'sensor_msgs',
    ],
    test_dependencies: ['ament_lint_auto', 'ament_lint_common'],
    build_type: 'ament_cmake',
    packages: ['kalman_filter'],
    published_topics: [
      '/kalman_filter/package_info',
      '/filtered_odometry',
      '/filtered_pose',
    ],
    subscribed_topics: [],
    advertised_services: [],
    called_services: [],
    tags: ['estimation', 'kalman-filter', 'state-estimation', 'sensor-fusion'],
    algorithms: ['kalman-filter', 'sensor-fusion', 'state-estimation'],
    source_path: 'ros/src/kalman_filter',
    package_xml_path: 'ros/src/kalman_filter/package.xml',
    cmake_lists_path: 'ros/src/kalman_filter/CMakeLists.txt',
  },
];

async function populateROSPackages() {
  try {
    console.log('🚀 Connecting to database...');
    await Database.connect();

    console.log('📦 Populating ROS packages...');

    for (const pkg of mockPackages) {
      console.log(`  - Adding package: ${pkg.name}`);

      const result = (await Database.query(
        `
        INSERT INTO ros_packages (
          name, version, description, category, type, maintainer_email, license,
          build_dependencies, runtime_dependencies, test_dependencies, build_type,
          packages, published_topics, subscribed_topics, advertised_services, called_services,
          tags, algorithms, source_path, package_xml_path, cmake_lists_path
        ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13, $14, $15, $16, $17, $18, $19, $20, $21)
        ON CONFLICT (name) DO UPDATE SET
          version = EXCLUDED.version,
          description = EXCLUDED.description,
          category = EXCLUDED.category,
          type = EXCLUDED.type,
          maintainer_email = EXCLUDED.maintainer_email,
          license = EXCLUDED.license,
          build_dependencies = EXCLUDED.build_dependencies,
          runtime_dependencies = EXCLUDED.runtime_dependencies,
          test_dependencies = EXCLUDED.test_dependencies,
          build_type = EXCLUDED.build_type,
          packages = EXCLUDED.packages,
          published_topics = EXCLUDED.published_topics,
          subscribed_topics = EXCLUDED.subscribed_topics,
          advertised_services = EXCLUDED.advertised_services,
          called_services = EXCLUDED.called_services,
          tags = EXCLUDED.tags,
          algorithms = EXCLUDED.algorithms,
          source_path = EXCLUDED.source_path,
          package_xml_path = EXCLUDED.package_xml_path,
          cmake_lists_path = EXCLUDED.cmake_lists_path,
          updated_at = CURRENT_TIMESTAMP
        RETURNING id, name
      `,
        [
          pkg.name,
          pkg.version,
          pkg.description,
          pkg.category,
          pkg.type,
          pkg.maintainer_email,
          pkg.license,
          JSON.stringify(pkg.build_dependencies),
          JSON.stringify(pkg.runtime_dependencies),
          JSON.stringify(pkg.test_dependencies),
          pkg.build_type,
          JSON.stringify(pkg.packages),
          JSON.stringify(pkg.published_topics),
          JSON.stringify(pkg.subscribed_topics),
          JSON.stringify(pkg.advertised_services),
          JSON.stringify(pkg.called_services),
          JSON.stringify(pkg.tags),
          JSON.stringify(pkg.algorithms),
          pkg.source_path,
          pkg.package_xml_path,
          pkg.cmake_lists_path,
        ]
      )) as { rows: Array<{ id: string; name: string }> };

      console.log(
        `    ✅ Package ${pkg.name} added/updated with ID: ${result.rows[0].id}`
      );
    }

    // Verify the packages were added
    const countResult = (await Database.query(
      'SELECT COUNT(*) as count FROM ros_packages'
    )) as { rows: Array<{ count: string }> };
    console.log(
      `\n📊 Total ROS packages in database: ${countResult.rows[0].count}`
    );

    const packagesResult = (await Database.query(
      'SELECT name, category, type FROM ros_packages ORDER BY name'
    )) as { rows: Array<{ name: string; category: string; type: string }> };
    console.log('\n📋 ROS Packages in database:');
    packagesResult.rows.forEach((row) => {
      console.log(`  - ${row.name} (${row.category}, ${row.type})`);
    });

    console.log('\n✅ ROS packages population completed successfully!');
  } catch (error) {
    console.error('❌ Error populating ROS packages:', error);
    process.exit(1);
  } finally {
    await Database.disconnect();
  }
}

// Run the script
if (require.main === module) {
  populateROSPackages();
}

export { populateROSPackages };
