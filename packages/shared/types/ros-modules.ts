export interface ModuleSpec {
  id: string;
  title: string;
  description: string;
  category:
    | 'robots'
    | 'navigation'
    | 'communication'
    | 'vision'
    | 'gui'
    | 'debug'
    | 'web';
  version: string;
  rosDistros: string[]; // e.g., ['humble', 'foxy', 'noetic']
  dependencies?: string[]; // IDs of other modules this depends on
  apt: string[]; // System packages to install
  pip: string[]; // Python packages to install
  env: Record<string, string>; // Environment variables
  setupCommands: string[]; // Custom setup commands
  udevRules?: string[]; // UDEV rules
  expose?: number[]; // Ports to expose
  files?: Array<{
    path: string;
    contents: string;
  }>;
  bashrcAliases?: string[]; // Bash aliases to add
}

export interface RobotSpec extends ModuleSpec {
  category: 'robots';
  baseImage: string; // e.g., 'osrf/ros:humble-ros-base'
  supportedModules: string[]; // IDs of modules this robot supports
}

export interface DistroSpec {
  id: string;
  name: string;
  version: string;
  baseImages: {
    core: string;
    base: string;
    desktop: string;
    desktopFull: string;
  };
  supportedRobots: string[];
  supportedModules: string[];
}

export interface ROSProjectConfig {
  distro: string;
  robot: string;
  modules: string[];
  customizations: {
    udevRules: string[];
    env: Record<string, string>;
    expose: number[];
    files: Array<{
      path: string;
      contents: string;
    }>;
    bashrcAliases: string[];
  };
  buildProfile: 'development' | 'production';
}

export interface ModuleRegistry {
  distros: DistroSpec[];
  robots: RobotSpec[];
  modules: ModuleSpec[];
}

