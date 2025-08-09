import { ApiService } from './api';

export interface Robot {
  code: string;
  name: string;
  module_count: number | string;
}

export interface RobotsResponse {
  success: boolean;
  data: Robot[];
}

export class RobotsService {
  static async getRobots(): Promise<Robot[]> {
    try {
      const response = await ApiService.get<Robot[]>('/robots');
      return response || [];
    } catch (error) {
      console.error('Failed to fetch robots:', error);
      return [];
    }
  }

  static async getAdminRobots(): Promise<Robot[]> {
    try {
      const response = await ApiService.get<Robot[]>('/admin/robots');
      return response || [];
    } catch (error) {
      console.error('Failed to fetch admin robots:', error);
      return [];
    }
  }

  static async getRobotNames(): Promise<string[]> {
    try {
      const robots = await this.getRobots();
      return robots.map((robot) => robot.code);
    } catch (error) {
      console.error('Failed to fetch robot names:', error);
      return [];
    }
  }

  static getRobotDisplayName(code: string): string {
    const robotNames: Record<string, string> = {
      turtlebot3: 'TurtleBot 3',
      pioneer3at: 'Pioneer 3-AT',
      kobuki: 'Kobuki Base',
      create3: 'iRobot Create 3',
      jackal: 'Clearpath Jackal',
      husky: 'Clearpath Husky',
      clearpath_robots: 'Clearpath Robots',
      pr2: 'Willow Garage PR2',
      fetch: 'Fetch Robotics',
      baxter: 'Rethink Robotics Baxter',
    };

    return (
      robotNames[code] ||
      code.replace(/_/g, ' ').replace(/\b\w/g, (l) => l.toUpperCase())
    );
  }
}
