import { ApiService } from './api';

export interface AIProjectSuggestion {
  name: string;
  description: string;
  category: string;
  tags: string[];
  algorithms: string[];
  environment: {
    baseImage: string;
    pythonVersion?: string;
    nodeVersion?: string;
    systemDependencies: string[];
    pythonDependencies: string[];
    nodeDependencies: string[];
    environmentVariables: Record<string, string>;
    ports: number[];
  };
  settings: {
    autoSave: boolean;
    enableDebugging: boolean;
    enableLogging: boolean;
    maxMemory: number;
    cpuLimit: number;
    enableGPU: boolean;
    backupFrequency: string;
  };
}

export class AIProjectGenerationService {
  private static instance: AIProjectGenerationService;

  private constructor() {}

  public static getInstance(): AIProjectGenerationService {
    if (!AIProjectGenerationService.instance) {
      AIProjectGenerationService.instance = new AIProjectGenerationService();
    }
    return AIProjectGenerationService.instance;
  }

  /**
   * Generate project suggestions based on user input
   */
  async generateProjectSuggestion(
    userInput: string
  ): Promise<AIProjectSuggestion> {
    try {
      // For now, we'll use a rule-based approach
      // In the future, this could call an actual AI service
      const suggestion = this.analyzeUserInput(userInput);

      // Simulate API call delay
      await new Promise((resolve) => setTimeout(resolve, 2000));

      return suggestion;
    } catch (error) {
      console.error('Error generating project suggestion:', error);
      throw new Error('Failed to generate project suggestion');
    }
  }

  /**
   * Analyze user input and generate project suggestions
   */
  private analyzeUserInput(userInput: string): AIProjectSuggestion {
    const input = userInput.toLowerCase();

    // Default suggestion
    let suggestion: AIProjectSuggestion = {
      name: this.generateProjectName(userInput),
      description: this.generateDescription(userInput),
      category: 'Other',
      tags: this.extractTags(userInput),
      algorithms: [],
      environment: {
        baseImage: 'python:3.11-slim',
        pythonVersion: '3.11',
        systemDependencies: [],
        pythonDependencies: [],
        nodeDependencies: [],
        environmentVariables: {},
        ports: [8080],
      },
      settings: {
        autoSave: true,
        enableDebugging: false,
        enableLogging: true,
        maxMemory: 2048,
        cpuLimit: 2,
        enableGPU: false,
        backupFrequency: 'weekly',
      },
    };

    // Analyze for navigation projects
    if (this.containsNavigationKeywords(input)) {
      suggestion.category = 'Navigation';
      suggestion.algorithms = ['nav2', 'tf2', 'rviz'];
      suggestion.environment.systemDependencies = ['ros2', 'nav2-bringup'];
      suggestion.environment.pythonDependencies = ['nav2-msgs', 'tf2-ros'];
      suggestion.tags.push('navigation', 'autonomous', 'mapping');
    }

    // Analyze for manipulation projects
    if (this.containsManipulationKeywords(input)) {
      suggestion.category = 'Manipulation';
      suggestion.algorithms = ['moveit', 'tf2', 'rviz'];
      suggestion.environment.systemDependencies = ['ros2', 'moveit'];
      suggestion.environment.pythonDependencies = ['moveit-msgs', 'tf2-ros'];
      suggestion.tags.push('manipulation', 'robotic-arm', 'grasping');
    }

    // Analyze for perception projects
    if (this.containsPerceptionKeywords(input)) {
      suggestion.category = 'Perception';
      suggestion.algorithms = ['opencv', 'pcl', 'rviz'];
      suggestion.environment.systemDependencies = ['opencv', 'pcl'];
      suggestion.environment.pythonDependencies = [
        'opencv-python',
        'pcl-python',
      ];
      suggestion.tags.push('perception', 'computer-vision', 'sensors');
    }

    // Analyze for simulation projects
    if (this.containsSimulationKeywords(input)) {
      suggestion.algorithms.push('gazebo');
      suggestion.environment.systemDependencies.push('gazebo');
      suggestion.tags.push('simulation', 'gazebo');
    }

    // Analyze for data recording
    if (this.containsDataKeywords(input)) {
      suggestion.algorithms.push('rosbag');
      suggestion.environment.systemDependencies.push('rosbag');
      suggestion.tags.push('data', 'recording', 'playback');
    }

    // Analyze for mobile robots
    if (this.containsMobileKeywords(input)) {
      suggestion.tags.push('mobile-robot', 'autonomous');
      if (!suggestion.algorithms.includes('nav2')) {
        suggestion.algorithms.push('nav2');
      }
    }

    // Analyze for office/home environments
    if (this.containsOfficeKeywords(input)) {
      suggestion.tags.push('indoor', 'office', 'home');
      suggestion.settings.maxMemory = 4096; // More memory for complex environments
      suggestion.settings.cpuLimit = 4;
    }

    // Analyze for GPU requirements
    if (this.containsGPUKeywords(input)) {
      suggestion.settings.enableGPU = true;
      suggestion.environment.baseImage = 'nvidia/cuda:11.8-devel-ubuntu20.04';
      suggestion.tags.push('gpu', 'cuda');
    }

    return suggestion;
  }

  private generateProjectName(userInput: string): string {
    const words = userInput.split(' ').filter((word) => word.length > 3);
    if (words.length === 0) return 'My Robotics Project';

    const keyWords = words.slice(0, 3);
    return (
      keyWords
        .map((word) => word.charAt(0).toUpperCase() + word.slice(1))
        .join('-') + '-Robot'
    );
  }

  private generateDescription(userInput: string): string {
    return `An autonomous robotics project: ${userInput}. This project is designed to handle the specified requirements with appropriate algorithms and configurations.`;
  }

  private extractTags(userInput: string): string[] {
    const tags: string[] = [];
    const input = userInput.toLowerCase();

    // Extract common robotics terms
    const roboticsTerms = [
      'robot',
      'autonomous',
      'navigation',
      'perception',
      'manipulation',
      'mobile',
      'arm',
      'vision',
      'sensor',
    ];
    roboticsTerms.forEach((term) => {
      if (input.includes(term)) {
        tags.push(term);
      }
    });

    return tags;
  }

  private containsNavigationKeywords(input: string): boolean {
    const keywords = [
      'navigate',
      'navigation',
      'move',
      'autonomous',
      'path',
      'route',
      'mapping',
      'slam',
      'localization',
    ];
    return keywords.some((keyword) => input.includes(keyword));
  }

  private containsManipulationKeywords(input: string): boolean {
    const keywords = [
      'manipulate',
      'grasp',
      'pick',
      'place',
      'arm',
      'hand',
      'gripper',
      'manipulation',
    ];
    return keywords.some((keyword) => input.includes(keyword));
  }

  private containsPerceptionKeywords(input: string): boolean {
    const keywords = [
      'vision',
      'camera',
      'image',
      'detect',
      'recognize',
      'perception',
      'sensor',
      'lidar',
      'depth',
    ];
    return keywords.some((keyword) => input.includes(keyword));
  }

  private containsSimulationKeywords(input: string): boolean {
    const keywords = ['simulate', 'simulation', 'gazebo', 'virtual', 'test'];
    return keywords.some((keyword) => input.includes(keyword));
  }

  private containsDataKeywords(input: string): boolean {
    const keywords = ['record', 'data', 'log', 'playback', 'rosbag'];
    return keywords.some((keyword) => input.includes(keyword));
  }

  private containsMobileKeywords(input: string): boolean {
    const keywords = ['mobile', 'wheeled', 'drone', 'uav', 'ground', 'vehicle'];
    return keywords.some((keyword) => input.includes(keyword));
  }

  private containsOfficeKeywords(input: string): boolean {
    const keywords = ['office', 'home', 'indoor', 'room', 'building', 'house'];
    return keywords.some((keyword) => input.includes(keyword));
  }

  private containsGPUKeywords(input: string): boolean {
    const keywords = [
      'gpu',
      'cuda',
      'deep learning',
      'neural',
      'ai',
      'machine learning',
    ];
    return keywords.some((keyword) => input.includes(keyword));
  }
}

export default AIProjectGenerationService.getInstance();
