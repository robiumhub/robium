-- Migration: Remove status and is_public columns from projects table
-- Date: 2025-08-07
-- Description: Remove the status and is_public fields from projects as they're no longer needed

-- Remove status column from projects table
ALTER TABLE projects DROP COLUMN IF EXISTS status;

-- Remove is_public column from projects table
ALTER TABLE projects DROP COLUMN IF EXISTS is_public;

-- Update migration record
INSERT INTO migrations (id, name) VALUES 
('007_remove_status_and_public_from_projects', 'Remove status and is_public columns from projects table')
ON CONFLICT (id) DO NOTHING; 