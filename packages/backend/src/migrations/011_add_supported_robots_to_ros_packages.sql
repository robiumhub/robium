-- Migration: Add supported_robots to ros_packages
-- Adds a TEXT[] column to list robot codes supported by a ROS package

-- UP
ALTER TABLE ros_packages
  ADD COLUMN IF NOT EXISTS supported_robots TEXT[] DEFAULT '{}'::text[];

CREATE INDEX IF NOT EXISTS idx_ros_packages_supported_robots ON ros_packages USING GIN (supported_robots);

-- DOWN
DROP INDEX IF EXISTS idx_ros_packages_supported_robots;
ALTER TABLE ros_packages DROP COLUMN IF EXISTS supported_robots;
