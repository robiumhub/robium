import { ProjectMetadata } from './project-metadata';

export interface ProjectConfig {
  metadata: ProjectMetadata;
  modules: string[];
} 