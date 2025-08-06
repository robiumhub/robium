export interface ModuleMetadata {
  name: string;
  description: string;
  version: string;
  category: string;
  packages: string[];
  dependencies?: string[];
  tags?: string[];
} 