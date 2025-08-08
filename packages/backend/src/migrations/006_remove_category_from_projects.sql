-- Migration: Remove category column from projects table
-- Date: 2025-08-07
-- Description: Remove the category field from projects as it's no longer needed

-- Remove category column from projects table
ALTER TABLE projects DROP COLUMN IF EXISTS category;

-- Update migration record
INSERT INTO migrations (id, name) VALUES 
('006_remove_category_from_projects', 'Remove category column from projects table')
ON CONFLICT (id) DO NOTHING; 