// Import schemas
import projectMetadataSchema from './schemas/project-metadata.schema.json';
import ros2PackageSchema from './schemas/ros2-package.schema.json';
import environmentConfigSchema from './schemas/environment-config.schema.json';
import simulationConfigSchema from './schemas/simulation-config.schema.json';
import projectConfigSchema from './schemas/project-config.schema.json';

// Export schemas
export {
  projectMetadataSchema,
  ros2PackageSchema,
  environmentConfigSchema,
  simulationConfigSchema,
  projectConfigSchema
};

// Export validation utilities (to be implemented)
export * from './validation/schema-validator';
export * from './validation/config-validator';

// Export types (to be implemented)
export * from './types/project-config';
export * from './types/ros2-package';
export * from './types/environment-config';
export * from './types/simulation-config';

// Export utilities (to be implemented)
export * from './utils/schema-loader';
export * from './utils/config-migrator'; 