## Robium — Comprehensive Project Description

Robium is a monorepo-based robot development platform that helps teams design, configure, build, and operate robot applications using modern web tooling, strong typing, and containerized runtimes. It integrates a TypeScript/Express backend, a React frontend, shared schemas/types/utilities, and a ROS2 workspace to streamline the full lifecycle from project definition to Dockerized execution.

### Mission and Scope

- Provide an end-to-end workflow for robotics projects, from project configuration to containerized build/run.
- Standardize how robot software is described (metadata, dependencies, supported robots) and validated (JSON Schemas).
- Automate Dockerfile and Docker Compose generation tailored to project selections.
- Offer a web UI for managing projects, modules, and assets with real-time feedback via WebSockets.
- Support local and containerized ROS2 development for portability and reproducibility.

---

## Key Capabilities

- Authentication and RBAC: User login/registration with route-level authorization.
- Project Management: Create and manage project configurations, tags, and supported robots.
- Dockerfile Generation: Template-driven Dockerfiles produced on demand from project data.
- ROS Integration: Optional local or containerized ROS2 (Humble+) workspace management.
- Database-backed State: SQLite schema for users, projects, activity logs, container state, etc.
- WebSocket Server: Real-time log streaming, status updates, and interactive operations.
- Admin & Dashboard: Administrative endpoints and dashboard data for observability.

Status highlights (from current repo):

- Completed: AuthN/AuthZ, project config storage, Dockerfile generation, frontend project wizard, database migrations, view Dockerfile in UI.
- In Progress: Container lifecycle management, advanced algorithm selection, richer execution controls, caching strategies.

---

## Architecture Overview

The repository uses npm workspaces to organize services and shared code:

- `packages/backend`: TypeScript/Express API, WebSocket server, migrations, services.
- `packages/frontend`: React application, routing, pages, charts, and services.
- `packages/shared`: Shared types, JSON Schemas, templates, module metadata, and validation utilities.
- `ros/`: ROS2 workspace (as a submodule-style directory) with Docker build tooling and helper scripts.
- Root-level `docker-compose.yml`: Spins up frontend, backend (with embedded SQLite), and a ROS container target.

### High-Level Data Flow

1. User authenticates in the frontend and creates/edits a project.
2. Backend validates and stores project configuration in SQLite.
3. Dockerfile Generation Service renders a Dockerfile from templates and project data.
4. Optional: Container lifecycle and log streaming run via services + WebSockets.
5. ROS workspace can be built and interacted with locally or within containers.

---

## Backend (packages/backend)

### Stack

- Node.js, TypeScript, Express
- SQLite (via custom `Database` utility)
- WebSockets for realtime features
- Helmet, CORS, Morgan, centralized error handling

### Server Setup

- Health endpoint: `GET /health` (DB connectivity, service status, request ID)
- Middleware: request ID, request timing, security headers, CORS, logging, global error handler
- Migrations auto-run on startup via `MigrationManager`
- WebSocket server bootstrapped alongside HTTP server with heartbeat and graceful shutdown

### Primary Routes (selected)

- `POST /auth/*` (authentication)
- `GET/POST /admin/*` (admin operations)
- `GET /modules` (list of modules and related metadata)
- `GET/POST /projects` (manage projects, metadata, tags)
- `GET /dockerfiles/:projectId` (view generated Dockerfile)
- `POST /dockerfiles/:projectId/generate` (trigger generation)
- `GET /ros-packages` (ROS package data)
- `GET /robots` (robot models and supported capabilities)
- `GET /dashboard` (dashboard data)

### Services (selected)

- `DockerfileGenerationService`: Core generator using template files (Handlebars-style), modules, and project inputs.
- `ContainerLifecycleService`: Orchestrates starting/stopping containers and tracking state.
- `DockerService`: Low-level Docker interactions.
- `TemplateEngine`: Templating abstraction used by generation services.
- `EnvironmentVariableService`: Centralized environment handling.
- `ValidationService`: Input and schema validation.
- `CachingService`: Cache strategies for expensive operations.
- `LogStreamingService`: Stream logs/events over WebSockets.
- `WorkspaceMountingService`: Controls host/container mounts for project assets.
- `AutomatedCleanupService`: Background cleanup of stale resources.

### Database & Migrations

Migrations define tables and indexes for users, projects, modules, ros-packages, activity logs, and related entities. Current files include:

- `001_initial_schema.sql`
- `002_project_configuration_schema.sql`
- `003_ros_packages_schema.sql`
- `004_modules_schema.sql`
- `005_projects_schema.sql`
- `006_remove_category_from_projects.sql`
- `007_remove_status_and_public_from_projects.sql`
- `008_add_tags_to_projects.sql`
- `009_add_supported_robots_to_modules.sql`

### Error Handling & Observability

- Structured logging via `logger`
- Unified error pipeline: not found handler, global error handler, crash safety for unhandled rejections/exceptions
- Request correlation with IDs and timing metrics

---

## Frontend (packages/frontend)

### Stack & Structure

- React 18 + TypeScript
- Pages for Admin, Dashboard, Projects, Modules, Robots, Execution Environment, etc.
- Components for forms, charts, status indicators, overlays, toasts, and UI primitives.
- Contexts for Auth, Error, and Navigation.
- Services for API consumption and project generation.
- Design system and tokens for consistent styling.

### Highlights

- Project Creation Wizard and configuration editors
- Role-based routes and protected routes
- Charts for analytics (e.g., robot categories, trends, types)
- Error boundaries and accessibility helpers

---

## Shared Package (packages/shared)

### Purpose

- Shared TypeScript types across backend and frontend
- JSON Schemas for strongly validated configuration
- Project templates and module definitions
- Validation utilities and schema loader

### Notable Contents

- Schemas: `project-config.schema.json`, `environment-config.schema.json`, `project-metadata.schema.json`, `ros2-package.schema.json`, `simulation-config.schema.json`
- Types: project config, environment config, metadata, ROS module definitions
- Templates: sample project templates (navigation, manipulation, perception, custom)
- Modules library: curated module JSON files (e.g., `navigation/nav2-core.json`, `vision/depthai-oakd.json`, `robots/turtlebot4.json`, `communication/rmw-cyclonedds.json`)

---

## ROS Workspace (ros/)

- Dockerfile and Compose definitions for building a ROS2 workspace in containers
- `ros` helper script with commands (setup, build, help, etc.)
- `meta/core.vcs.yaml` for upstream ROS package dependencies (via vcstool)
- Supports both local builds and containerized workflows for consistency

Typical containerized usage:

```bash
cd ros
docker compose up --build -d
docker compose exec ros-workspace bash
./ros build
source install/setup.bash
```

---

## Development Environment

- Root `docker-compose.yml` to run:
  - Frontend (React) on port 3000
  - Backend (Express) on port 8000 (exposed as 8001 in compose mapping)
  - SQLite database (embedded in backend)
  - ROS container build target
- NPM scripts and shell helpers under `scripts/` for starting, cleaning, and ROS bootstrapping
- Backend auto-runs DB migrations on start

Local quick start (see README for details):

```bash
./scripts/start-dev.sh   # Launch dev servers
./scripts/cleanup.sh     # Stop servers and cleanup
```

---

## Typical User Flow

1. Sign up / sign in via frontend.
2. Create a project and select modules, robots, and configurations.
3. Validate configuration against shared schemas.
4. Generate and review a Dockerfile tailored to the project.
5. (Planned) Launch/stop containers, stream logs, and interact with runtime via the UI.
6. Iterate on project settings; regenerate Dockerfiles or rebuild as needed.

---

## Security & Reliability

- Helmet, CORS, and input validation on the server
- Parameterized database access and migration-based schema evolution
- Centralized error handling with structured logs
- WebSocket heartbeat and connection limits

---

## Extensibility Guide

- Add a new module:

  - Define metadata in `packages/shared/modules/<category>/<name>.json`.
  - Update templates or generation rules if build steps differ.
  - Expose data via `/modules` and integrate selection in the frontend UI.

- Add a new project template:

  - Create a new template JSON under `packages/shared/templates/`.
  - Ensure schema alignment and update frontend wizard if needed.

- Extend Dockerfile generation:

  - Update `templates/dockerfile` and partials used by `DockerfileGenerationService` and `TemplateEngine`.
  - Add service logic for new build contexts (e.g., GPU support, custom base images).

- Add API endpoints:
  - Implement route, validation, and service logic in `packages/backend/src`.
  - Follow existing error handling and logging conventions.

---

## Current Roadmap (Indicative)

- Container lifecycle management from the UI (create/start/stop/restart/cleanup)
- Expanded execution controls and telemetry streaming
- Advanced algorithm selection and pipeline composition
- Caching strategies and performance tuning
- Richer dashboards and analytics
- Deeper ROS workspace automation and presets

---

## Repository Structure (Abbreviated)

```
robium/
├── packages/
│   ├── backend/   # Express API, services, migrations, WebSockets
│   ├── frontend/  # React UI, pages, services, contexts
│   └── shared/    # Types, schemas, templates, modules, validation
├── ros/           # ROS2 workspace, Dockerfile, compose, helper script
├── docs/          # Documentation (this file, rules, guides)
├── scripts/       # Dev and bootstrap scripts
└── docker-compose.yml
```

---

## Getting Started

See `README.md` for prerequisites, setup steps, and quick commands. Key URLs after startup:

- Frontend: `http://localhost:3000`
- Backend API: `http://localhost:8000` (exposed as 8001 in docker-compose)

---

## License

Robium is licensed under the MIT License (see `README.md`).
