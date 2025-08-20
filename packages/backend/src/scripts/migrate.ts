import { MigrationManager } from '../utils/migrations';
import { Database } from '../utils/database';

async function main() {
  const command = process.argv[2] || 'up';

  try {
    await Database.connect();

    const migrationManager = new MigrationManager();

    if (command === 'up') {
      await migrationManager.runPendingMigrations();
      console.log('Migrations completed successfully');
    } else if (command === 'reset') {
      // TODO: Implement reset functionality
      console.log('Reset functionality not implemented yet');
    } else {
      console.log('Unknown command. Use "up" or "reset"');
    }
  } catch (error) {
    console.error('Migration failed:', error);
    process.exit(1);
  } finally {
    Database.close();
  }
}

main();
