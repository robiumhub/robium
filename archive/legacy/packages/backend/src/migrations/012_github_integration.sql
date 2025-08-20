-- UP
-- Add GitHub repository integration columns to projects
-- Note: SQLite doesn't support ADD COLUMN IF NOT EXISTS, so we'll handle this in the application layer
-- The columns are already added in the initializeSqliteSchema function
SELECT 1;

-- DOWN
-- Note: SQLite doesn't support DROP COLUMN easily, so this is a no-op for SQLite
SELECT 1;


