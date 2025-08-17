-- UP
-- Add GitHub repository integration columns to projects
ALTER TABLE projects
  ADD COLUMN IF NOT EXISTS github_repo_owner VARCHAR(255),
  ADD COLUMN IF NOT EXISTS github_repo_name VARCHAR(255),
  ADD COLUMN IF NOT EXISTS github_repo_url VARCHAR(500),
  ADD COLUMN IF NOT EXISTS github_repo_id BIGINT;

CREATE INDEX IF NOT EXISTS idx_projects_github_owner ON projects(github_repo_owner);
CREATE INDEX IF NOT EXISTS idx_projects_github_name ON projects(github_repo_name);

-- DOWN
DROP INDEX IF EXISTS idx_projects_github_name;
DROP INDEX IF EXISTS idx_projects_github_owner;
ALTER TABLE projects
  DROP COLUMN IF EXISTS github_repo_id,
  DROP COLUMN IF EXISTS github_repo_url,
  DROP COLUMN IF EXISTS github_repo_name,
  DROP COLUMN IF EXISTS github_repo_owner;


