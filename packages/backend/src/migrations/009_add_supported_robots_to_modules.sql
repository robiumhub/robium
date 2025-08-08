-- Migration: Add supported_robots to modules
-- Adds a TEXT[] column to list robot codes supported by a module

-- UP
ALTER TABLE modules
  ADD COLUMN IF NOT EXISTS supported_robots TEXT[] DEFAULT '{}'::text[];

CREATE INDEX IF NOT EXISTS idx_modules_supported_robots ON modules USING GIN (supported_robots);

-- DOWN
DROP INDEX IF EXISTS idx_modules_supported_robots;
ALTER TABLE modules DROP COLUMN IF EXISTS supported_robots;


