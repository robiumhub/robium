import * as fs from 'fs';
import * as path from 'path';
import { ProjectConfig } from '../types/project-config';

export async function loadProjectConfig(templateName: string): Promise<ProjectConfig> {
  const templatePath = path.join(__dirname, `../templates/${templateName}.json`);
  const fileContents = await fs.promises.readFile(templatePath, 'utf8');
  return JSON.parse(fileContents);
} 