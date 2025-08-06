import { schemaValidator } from './validation/schema-validator';
import { configValidator } from './validation/config-validator';
import { projectMetadataSchema, ros2PackageSchema, environmentConfigSchema, simulationConfigSchema, projectConfigSchema } from './index';

// Test data for validation
const testProjectMetadata = {
  id: "550e8400-e29b-41d4-a716-446655440000",
  name: "test-project",
  description: "A test robotics project",
  version: "1.0.0",
  author: {
    id: "550e8400-e29b-41d4-a716-446655440001",
    name: "Test User",
    email: "test@example.com"
  },
  createdAt: "2024-01-01T00:00:00Z",
  projectType: "navigation"
};

const testRos2Package = {
  name: "test_package",
  version: "1.0.0",
  taskCategory: "navigation",
  algorithms: [
    {
      id: "550e8400-e29b-41d4-a716-446655440002",
      name: "test_algorithm",
      description: "A test algorithm",
      taskDefinition: "Performs test navigation",
      inputOutput: {
        inputs: [
          {
            name: "input1",
            type: "string",
            description: "Test input"
          }
        ],
        outputs: [
          {
            name: "output1",
            type: "string",
            description: "Test output"
          }
        ]
      }
    }
  ],
  dependencies: {
    ros2_packages: ["rclcpp", "nav_msgs"],
    system_packages: ["libboost-all-dev"]
  }
};

const testEnvironmentConfig = {
  environment: {
    variables: {
      ROS_DOMAIN_ID: {
        value: 0,
        type: "number",
        description: "ROS domain ID"
      }
    }
  },
  resources: {
    cpu: {
      limit: 2
    },
    memory: {
      limit: "4G"
    },
    storage: {
      workspace: "10G"
    }
  },
  network: {
    mode: "bridge"
  }
};

const testSimulationConfig = {
  world: {
    name: "test_world",
    file: "/path/to/world.world"
  },
  robots: [
    {
      name: "test_robot",
      model: "/path/to/robot.urdf",
      pose: {
        position: [0, 0, 0],
        orientation: [0, 0, 0, 1]
      }
    }
  ],
  physics: {
    engine: "ode",
    maxStepSize: 0.001,
    realTimeFactor: 1
  }
};

const testProjectConfig = {
  metadata: testProjectMetadata,
  ros2Packages: [testRos2Package],
  environment: testEnvironmentConfig,
  simulation: testSimulationConfig
};

// Test validation functions
export function testSchemaValidation(): void {
  console.log("Testing schema validation...");

  // Test project metadata validation
  const metadataResult = schemaValidator.validateProjectMetadata(testProjectMetadata);
  console.log("Project metadata validation:", metadataResult.valid ? "PASS" : "FAIL");
  if (!metadataResult.valid) {
    console.log("Errors:", schemaValidator.getErrorMessages(metadataResult));
  }

  // Test ROS2 package validation
  const ros2Result = schemaValidator.validateRos2Package(testRos2Package);
  console.log("ROS2 package validation:", ros2Result.valid ? "PASS" : "FAIL");
  if (!ros2Result.valid) {
    console.log("Errors:", schemaValidator.getErrorMessages(ros2Result));
  }

  // Test environment config validation
  const envResult = schemaValidator.validateEnvironmentConfig(testEnvironmentConfig);
  console.log("Environment config validation:", envResult.valid ? "PASS" : "FAIL");
  if (!envResult.valid) {
    console.log("Errors:", schemaValidator.getErrorMessages(envResult));
  }

  // Test simulation config validation
  const simResult = schemaValidator.validateSimulationConfig(testSimulationConfig);
  console.log("Simulation config validation:", simResult.valid ? "PASS" : "FAIL");
  if (!simResult.valid) {
    console.log("Errors:", schemaValidator.getErrorMessages(simResult));
  }

  // Test complete project config validation
  const projectResult = schemaValidator.validateProjectConfig(testProjectConfig);
  console.log("Project config validation:", projectResult.valid ? "PASS" : "FAIL");
  if (!projectResult.valid) {
    console.log("Errors:", schemaValidator.getErrorMessages(projectResult));
  }

  // Test config validator
  const configResult = configValidator.validateProjectConfiguration(testProjectConfig);
  console.log("Config validator:", configResult.valid ? "PASS" : "FAIL");
  if (!configResult.valid) {
    console.log("Errors:", configValidator.getAllErrors(configResult));
  }

  console.log("Schema validation tests completed!");
}

// Run tests if this file is executed directly
if (require.main === module) {
  testSchemaValidation();
} 