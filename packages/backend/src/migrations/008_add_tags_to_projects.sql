-- Migration: Add tags array column to projects table
-- Date: 2025-08-08

-- Add tags column as TEXT[] with empty array default
ALTER TABLE projects
  ADD COLUMN IF NOT EXISTS tags TEXT[] DEFAULT '{}'::text[];

-- Optional index for searching by tag (GIN on text[])
CREATE INDEX IF NOT EXISTS idx_projects_tags ON projects USING GIN (tags);


