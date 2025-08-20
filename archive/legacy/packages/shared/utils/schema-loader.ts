import * as fs from 'fs';
import * as path from 'path';

export class SchemaLoader {
  /**
   * Load a schema from file
   */
  static loadSchema(filePath: string): any {
    try {
      const fullPath = path.resolve(filePath);
      const content = fs.readFileSync(fullPath, 'utf8');
      return JSON.parse(content);
    } catch (error) {
      throw new Error(`Failed to load schema from ${filePath}: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  /**
   * Load all schemas from a directory
   */
  static loadSchemasFromDirectory(dirPath: string): Record<string, any> {
    const schemas: Record<string, any> = {};
    
    try {
      const files = fs.readdirSync(dirPath);
      
      for (const file of files) {
        if (file.endsWith('.json')) {
          const schemaName = path.basename(file, '.json');
          const schemaPath = path.join(dirPath, file);
          schemas[schemaName] = this.loadSchema(schemaPath);
        }
      }
    } catch (error) {
      throw new Error(`Failed to load schemas from directory ${dirPath}: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
    
    return schemas;
  }

  /**
   * Validate schema structure
   */
  static validateSchemaStructure(schema: any): boolean {
    return schema && 
           typeof schema === 'object' && 
           schema.$schema && 
           schema.type === 'object';
  }

  /**
   * Get schema version
   */
  static getSchemaVersion(schema: any): string | null {
    return schema?.$schema || null;
  }

  /**
   * Get schema title
   */
  static getSchemaTitle(schema: any): string | null {
    return schema?.title || null;
  }

  /**
   * Get schema description
   */
  static getSchemaDescription(schema: any): string | null {
    return schema?.description || null;
  }
} 