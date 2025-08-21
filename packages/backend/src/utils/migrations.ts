import { Database } from './database';
import { logger } from './logger';

interface Migration {
  id: number;
  name: string;
  up: string;
  down: string;
}

const migrations: Migration[] = [
  {
    id: 1,
    name: 'initial_schema',
    up: `
      CREATE TABLE IF NOT EXISTS users (
        id TEXT PRIMARY KEY,
        email TEXT UNIQUE NOT NULL,
        username TEXT UNIQUE NOT NULL,
        password_hash TEXT NOT NULL,
        role TEXT NOT NULL DEFAULT 'user',
        is_active INTEGER NOT NULL DEFAULT 1,
        created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
        updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
      );

      CREATE TABLE IF NOT EXISTS projects (
        id TEXT PRIMARY KEY,
        name TEXT NOT NULL,
        description TEXT,
        owner_id TEXT NOT NULL,
        is_active INTEGER NOT NULL DEFAULT 1,
        is_template INTEGER NOT NULL DEFAULT 0,
        tags TEXT NOT NULL DEFAULT '[]',
        config TEXT NOT NULL DEFAULT '{}',
        metadata TEXT NOT NULL DEFAULT '{}',
        template_visibility TEXT,
        template_version TEXT,
        template_published_at TEXT,
        created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
        updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
        FOREIGN KEY (owner_id) REFERENCES users(id)
      );

      CREATE INDEX IF NOT EXISTS idx_projects_owner_id ON projects(owner_id);
      CREATE INDEX IF NOT EXISTS idx_projects_is_template ON projects(is_template);
      CREATE INDEX IF NOT EXISTS idx_projects_created_at ON projects(created_at);
    `,
    down: `
      DROP TABLE IF EXISTS projects;
      DROP TABLE IF EXISTS users;
    `,
  },
  {
    id: 2,
    name: 'filter_categories',
    up: `
      CREATE TABLE IF NOT EXISTS filter_categories (
        id TEXT PRIMARY KEY,
        name TEXT NOT NULL,
        display_name TEXT NOT NULL,
        description TEXT,
        type TEXT NOT NULL DEFAULT 'string', -- 'string', 'boolean', 'number'
        is_active INTEGER NOT NULL DEFAULT 1,
        sort_order INTEGER NOT NULL DEFAULT 0,
        created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
        updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
      );

      CREATE TABLE IF NOT EXISTS filter_values (
        id TEXT PRIMARY KEY,
        category_id TEXT NOT NULL,
        value TEXT NOT NULL,
        display_name TEXT NOT NULL,
        description TEXT,
        is_active INTEGER NOT NULL DEFAULT 1,
        sort_order INTEGER NOT NULL DEFAULT 0,
        created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
        updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
        FOREIGN KEY (category_id) REFERENCES filter_categories(id) ON DELETE CASCADE
      );

      CREATE INDEX IF NOT EXISTS idx_filter_values_category_id ON filter_values(category_id);
      CREATE INDEX IF NOT EXISTS idx_filter_values_sort_order ON filter_values(sort_order);
      CREATE INDEX IF NOT EXISTS idx_filter_categories_sort_order ON filter_categories(sort_order);

      -- Insert default filter categories
      INSERT OR REPLACE INTO filter_categories (id, name, display_name, description, type, sort_order) VALUES
        ('use_cases', 'use_cases', 'Use Cases', 'Primary use cases for the project', 'string', 1),
        ('capabilities', 'capabilities', 'Capabilities', 'Technical capabilities provided', 'string', 2),
        ('robots', 'robots', 'Robot Targets', 'Supported robot platforms', 'string', 3),
        ('simulators', 'simulators', 'Simulators', 'Supported simulation environments', 'string', 4),
        ('difficulty', 'difficulty', 'Difficulty Level', 'Project complexity level', 'string', 5),
        ('tags', 'tags', 'Tags', 'Custom tags for categorization', 'string', 6);

      -- Insert default filter values
      INSERT OR REPLACE INTO filter_values (id, category_id, value, display_name, sort_order) VALUES
        -- Use Cases
        ('uc_navigation', 'use_cases', 'navigation', 'Navigation', 1),
        ('uc_mapping', 'use_cases', 'mapping', 'Mapping', 2),
        ('uc_object_detection', 'use_cases', 'object_detection', 'Object Detection', 3),
        ('uc_tracking', 'use_cases', 'tracking', 'Tracking', 4),
        ('uc_swarm_robotics', 'use_cases', 'swarm_robotics', 'Swarm Robotics', 5),
        ('uc_coordination', 'use_cases', 'coordination', 'Coordination', 6),
        ('uc_line_following', 'use_cases', 'line_following', 'Line Following', 7),
        ('uc_weather_monitoring', 'use_cases', 'weather_monitoring', 'Weather Monitoring', 8),
        ('uc_data_collection', 'use_cases', 'data_collection', 'Data Collection', 9),

        -- Capabilities
        ('cap_slam', 'capabilities', 'slam', 'SLAM', 1),
        ('cap_localization', 'capabilities', 'localization', 'Localization', 2),
        ('cap_path_planning', 'capabilities', 'path_planning', 'Path Planning', 3),
        ('cap_image_processing', 'capabilities', 'image_processing', 'Image Processing', 4),
        ('cap_ml_inference', 'capabilities', 'ml_inference', 'ML Inference', 5),
        ('cap_multi_agent_control', 'capabilities', 'multi_agent_control', 'Multi-Agent Control', 6),
        ('cap_formation_control', 'capabilities', 'formation_control', 'Formation Control', 7),
        ('cap_sensor_reading', 'capabilities', 'sensor_reading', 'Sensor Reading', 8),
        ('cap_motor_control', 'capabilities', 'motor_control', 'Motor Control', 9),
        ('cap_sensor_data_collection', 'capabilities', 'sensor_data_collection', 'Sensor Data Collection', 10),
        ('cap_autonomous_navigation', 'capabilities', 'autonomous_navigation', 'Autonomous Navigation', 11),
        ('cap_obstacle_avoidance', 'capabilities', 'obstacle_avoidance', 'Obstacle Avoidance', 12),
        ('cap_camera_control', 'capabilities', 'camera_control', 'Camera Control', 13),

        -- Robots
        ('robot_turtlebot3', 'robots', 'turtlebot3', 'TurtleBot3', 1),
        ('robot_pioneer3', 'robots', 'pioneer3', 'Pioneer3', 2),
        ('robot_custom_camera', 'robots', 'custom_camera', 'Custom Camera', 3),
        ('robot_crazyflie', 'robots', 'crazyflie', 'Crazyflie', 4),
        ('robot_arduino_robot', 'robots', 'arduino_robot', 'Arduino Robot', 5),
        ('robot_custom_weather_robot', 'robots', 'custom_weather_robot', 'Custom Weather Robot', 6),

        -- Simulators
        ('sim_gazebo', 'simulators', 'gazebo', 'Gazebo', 1),
        ('sim_rviz', 'simulators', 'rviz', 'RViz', 2),
        ('sim_arduino_sim', 'simulators', 'arduino_sim', 'Arduino Sim', 3),

        -- Difficulty
        ('diff_beginner', 'difficulty', 'beginner', 'Beginner', 1),
        ('diff_intermediate', 'difficulty', 'intermediate', 'Intermediate', 2),
        ('diff_advanced', 'difficulty', 'advanced', 'Advanced', 3);
    `,
    down: `
      DROP TABLE IF EXISTS filter_values;
      DROP TABLE IF EXISTS filter_categories;
    `,
  },
  {
    id: 3,
    name: 'remove_simulators_from_templates',
    up: `
      -- Deactivate simulators filter category for templates
      UPDATE filter_categories 
      SET is_active = 0, updated_at = CURRENT_TIMESTAMP 
      WHERE id = 'simulators';
      
      -- Deactivate all simulator filter values
      UPDATE filter_values 
      SET is_active = 0, updated_at = CURRENT_TIMESTAMP 
      WHERE category_id = 'simulators';
    `,
    down: `
      -- Reactivate simulators filter category
      UPDATE filter_categories 
      SET is_active = 1, updated_at = CURRENT_TIMESTAMP 
      WHERE id = 'simulators';
      
      -- Reactivate all simulator filter values
      UPDATE filter_values 
      SET is_active = 1, updated_at = CURRENT_TIMESTAMP 
      WHERE category_id = 'simulators';
    `,
  },
  {
    id: 4,
    name: 'remove_difficulty_from_templates',
    up: `
      -- Deactivate difficulty filter category for templates
      UPDATE filter_categories 
      SET is_active = 0, updated_at = CURRENT_TIMESTAMP 
      WHERE id = 'difficulty';
      
      -- Deactivate all difficulty filter values
      UPDATE filter_values 
      SET is_active = 0, updated_at = CURRENT_TIMESTAMP 
      WHERE category_id = 'difficulty';
    `,
    down: `
      -- Reactivate difficulty filter category
      UPDATE filter_categories 
      SET is_active = 1, updated_at = CURRENT_TIMESTAMP 
      WHERE id = 'difficulty';
      
      -- Reactivate all difficulty filter values
      UPDATE filter_values 
      SET is_active = 1, updated_at = CURRENT_TIMESTAMP 
      WHERE category_id = 'difficulty';
    `,
  },
];

export class MigrationManager {
  private db = Database.getDatabase();

  async runPendingMigrations(): Promise<void> {
    try {
      // Create migrations table if it doesn't exist
      await this.createMigrationsTable();

      // Get list of applied migrations
      const appliedMigrations = await this.getAppliedMigrations();

      // Get all migration files
      const migrations = this.getMigrations();

      // Run pending migrations
      for (const migration of migrations) {
        if (!appliedMigrations.includes(migration.name)) {
          await this.runMigration(migration);
        }
      }

      logger.info('All migrations completed successfully');
    } catch (error) {
      logger.error('Migration failed', {
        error: error instanceof Error ? error.message : 'Unknown error',
      });
      throw error;
    }
  }

  private createMigrationsTable(): Promise<void> {
    const sql = `
      CREATE TABLE IF NOT EXISTS migrations (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL UNIQUE,
        applied_at DATETIME DEFAULT CURRENT_TIMESTAMP
      )
    `;
    return new Promise((resolve, reject) => {
      this.db.run(sql, (err) => {
        if (err) reject(err);
        else resolve();
      });
    });
  }

  private getAppliedMigrations(): Promise<string[]> {
    const sql = 'SELECT name FROM migrations ORDER BY applied_at';
    return new Promise((resolve, reject) => {
      this.db.all(sql, (err, rows) => {
        if (err) reject(err);
        else resolve(rows.map((row: any) => row.name));
      });
    });
  }

  private getMigrations(): Array<{ name: string; sql: string }> {
    return [
      {
        name: '001_initial_schema',
        sql: `
          CREATE TABLE IF NOT EXISTS users (
            id TEXT PRIMARY KEY,
            email TEXT UNIQUE NOT NULL,
            username TEXT UNIQUE NOT NULL,
            password_hash TEXT NOT NULL,
            role TEXT CHECK (role IN ('user', 'admin')) DEFAULT 'user',
            is_active INTEGER DEFAULT 1,
            github_token_encrypted TEXT,
            github_username TEXT,
            github_connected INTEGER DEFAULT 0,
            created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
            updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
          );

          CREATE TABLE IF NOT EXISTS projects (
            id TEXT PRIMARY KEY,
            name TEXT NOT NULL,
            description TEXT,
            owner_id TEXT NOT NULL,
            is_active INTEGER DEFAULT 1,
            is_template INTEGER DEFAULT 0,
            type TEXT,
            version TEXT DEFAULT '1.0.0',
            tags TEXT,
            config TEXT NOT NULL,
            metadata TEXT,
            github_repo_owner TEXT,
            github_repo_name TEXT,
            github_repo_url TEXT,
            github_repo_id INTEGER,
            template_published_at DATETIME,
            template_author TEXT,
            template_version TEXT DEFAULT '1.0.0',
            template_visibility TEXT CHECK (template_visibility IN ('public', 'private')) DEFAULT 'public',
            created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
            updated_at DATETIME DEFAULT CURRENT_TIMESTAMP,
            FOREIGN KEY (owner_id) REFERENCES users (id) ON DELETE CASCADE
          );

          CREATE INDEX IF NOT EXISTS idx_users_email ON users (email);
          CREATE INDEX IF NOT EXISTS idx_users_username ON users (username);
          CREATE INDEX IF NOT EXISTS idx_projects_owner_id ON projects (owner_id);
          CREATE INDEX IF NOT EXISTS idx_projects_name ON projects (name);
          CREATE INDEX IF NOT EXISTS idx_projects_is_template ON projects (is_template);
          CREATE INDEX IF NOT EXISTS idx_projects_created_at ON projects (created_at);
          CREATE INDEX IF NOT EXISTS idx_projects_updated_at ON projects (updated_at);
        `,
      },
      {
        name: '002_filter_categories',
        sql: `
          CREATE TABLE IF NOT EXISTS filter_categories (
            id TEXT PRIMARY KEY,
            name TEXT NOT NULL,
            display_name TEXT NOT NULL,
            description TEXT,
            type TEXT NOT NULL DEFAULT 'string',
            is_active INTEGER NOT NULL DEFAULT 1,
            sort_order INTEGER NOT NULL DEFAULT 0,
            created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
          );

          CREATE TABLE IF NOT EXISTS filter_values (
            id TEXT PRIMARY KEY,
            category_id TEXT NOT NULL,
            value TEXT NOT NULL,
            display_name TEXT NOT NULL,
            description TEXT,
            is_active INTEGER NOT NULL DEFAULT 1,
            sort_order INTEGER NOT NULL DEFAULT 0,
            created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            FOREIGN KEY (category_id) REFERENCES filter_categories(id) ON DELETE CASCADE
          );

          CREATE INDEX IF NOT EXISTS idx_filter_values_category_id ON filter_values(category_id);
          CREATE INDEX IF NOT EXISTS idx_filter_values_sort_order ON filter_values(sort_order);
          CREATE INDEX IF NOT EXISTS idx_filter_categories_sort_order ON filter_categories(sort_order);

          -- Insert default filter categories
          INSERT OR REPLACE INTO filter_categories (id, name, display_name, description, type, sort_order) VALUES
            ('use_cases', 'use_cases', 'Use Cases', 'Primary use cases for the project', 'string', 1),
            ('capabilities', 'capabilities', 'Capabilities', 'Technical capabilities provided', 'string', 2),
            ('robots', 'robots', 'Robot Targets', 'Supported robot platforms', 'string', 3),
            ('simulators', 'simulators', 'Simulators', 'Supported simulation environments', 'string', 4),
            ('difficulty', 'difficulty', 'Difficulty Level', 'Project complexity level', 'string', 5),
            ('tags', 'tags', 'Tags', 'Custom tags for categorization', 'string', 6);

          -- Insert default filter values
          INSERT OR REPLACE INTO filter_values (id, category_id, value, display_name, sort_order) VALUES
            -- Use Cases
            ('uc_navigation', 'use_cases', 'navigation', 'Navigation', 1),
            ('uc_mapping', 'use_cases', 'mapping', 'Mapping', 2),
            ('uc_object_detection', 'use_cases', 'object_detection', 'Object Detection', 3),
            ('uc_tracking', 'use_cases', 'tracking', 'Tracking', 4),
            ('uc_swarm_robotics', 'use_cases', 'swarm_robotics', 'Swarm Robotics', 5),
            ('uc_coordination', 'use_cases', 'coordination', 'Coordination', 6),
            ('uc_line_following', 'use_cases', 'line_following', 'Line Following', 7),
            ('uc_weather_monitoring', 'use_cases', 'weather_monitoring', 'Weather Monitoring', 8),
            ('uc_data_collection', 'use_cases', 'data_collection', 'Data Collection', 9),

            -- Capabilities
            ('cap_slam', 'capabilities', 'slam', 'SLAM', 1),
            ('cap_localization', 'capabilities', 'localization', 'Localization', 2),
            ('cap_path_planning', 'capabilities', 'path_planning', 'Path Planning', 3),
            ('cap_image_processing', 'capabilities', 'image_processing', 'Image Processing', 4),
            ('cap_ml_inference', 'capabilities', 'ml_inference', 'ML Inference', 5),
            ('cap_multi_agent_control', 'capabilities', 'multi_agent_control', 'Multi-Agent Control', 6),
            ('cap_formation_control', 'capabilities', 'formation_control', 'Formation Control', 7),
            ('cap_sensor_reading', 'capabilities', 'sensor_reading', 'Sensor Reading', 8),
            ('cap_motor_control', 'capabilities', 'motor_control', 'Motor Control', 9),
            ('cap_sensor_data_collection', 'capabilities', 'sensor_data_collection', 'Sensor Data Collection', 10),
            ('cap_autonomous_navigation', 'capabilities', 'autonomous_navigation', 'Autonomous Navigation', 11),
            ('cap_obstacle_avoidance', 'capabilities', 'obstacle_avoidance', 'Obstacle Avoidance', 12),
            ('cap_camera_control', 'capabilities', 'camera_control', 'Camera Control', 13),

            -- Robots
            ('robot_turtlebot3', 'robots', 'turtlebot3', 'TurtleBot3', 1),
            ('robot_pioneer3', 'robots', 'pioneer3', 'Pioneer3', 2),
            ('robot_custom_camera', 'robots', 'custom_camera', 'Custom Camera', 3),
            ('robot_crazyflie', 'robots', 'crazyflie', 'Crazyflie', 4),
            ('robot_arduino_robot', 'robots', 'arduino_robot', 'Arduino Robot', 5),
            ('robot_custom_weather_robot', 'robots', 'custom_weather_robot', 'Custom Weather Robot', 6),

            -- Simulators
            ('sim_gazebo', 'simulators', 'gazebo', 'Gazebo', 1),
            ('sim_rviz', 'simulators', 'rviz', 'RViz', 2),
            ('sim_arduino_sim', 'simulators', 'arduino_sim', 'Arduino Sim', 3),

            -- Difficulty
            ('diff_beginner', 'difficulty', 'beginner', 'Beginner', 1),
            ('diff_intermediate', 'difficulty', 'intermediate', 'Intermediate', 2),
            ('diff_advanced', 'difficulty', 'advanced', 'Advanced', 3);
        `,
      },
    ];
  }

  private async runMigration(migration: { name: string; sql: string }): Promise<void> {
    return new Promise((resolve, reject) => {
      this.db.serialize(() => {
        this.db.run('BEGIN TRANSACTION');

        // Split SQL by semicolon and execute each statement
        const statements = migration.sql.split(';').filter((stmt) => stmt.trim());

        let completed = 0;
        const total = statements.length + 1; // +1 for the INSERT statement

        const checkComplete = () => {
          completed++;
          if (completed === total) {
            this.db.run('COMMIT', (err) => {
              if (err) reject(err);
              else {
                logger.info(`Migration applied: ${migration.name}`);
                resolve();
              }
            });
          }
        };

        for (const statement of statements) {
          if (statement.trim()) {
            this.db.run(statement, (err) => {
              if (err) reject(err);
              else checkComplete();
            });
          } else {
            checkComplete();
          }
        }

        // Record the migration
        this.db.run('INSERT INTO migrations (name) VALUES (?)', [migration.name], (err) => {
          if (err) reject(err);
          else checkComplete();
        });
      });
    });
  }
}
