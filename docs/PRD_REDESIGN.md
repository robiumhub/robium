# Robium — Redesign PRD (Final)

## 1. Overview

Robium is a focused web platform to create and manage robotics project configurations and generate Dockerfiles from templates, using a simple, reliable stack built around SQLite, Express, and React. This redesign delivers an MVP centered on Authentication, Projects, Templates, Dockerfile generation, modern filtering UX, and GitHub publishing.

Primary objectives:

- Simplify around Projects and Templates (single table with `is_template`).
- Modern, dynamic UI with excellent filtering and discoverability.
- GitHub publishing at project create/clone (user PAT or admin token fallback).
- SQLite-only backend with consistent APIs and predictable DX.

## 2. In Scope (MVP)

- Authentication: signup/login/JWT/me/change-password/logout
- Projects: create/list/get/update-settings/delete/clone; admin-only convert-to-template
- Templates: list/search/filter (subset of projects with `is_template = 1`)
- Dockerfiles: generate on demand from project config; view; cache on disk
- GitHub integration (MVP): create repo and push initial files during create/clone
- Modern filtering UX (Hugging Face–style chip sidebar) for Projects and Templates
- SQLite DB, migrations on startup, uniform API responses

Out of scope (now): ROS packages/modules, datasets, execution workspace, WebSocket features, container lifecycle.

## 3. Architecture

- Monorepo (npm workspaces)
  - `packages/backend`: Express + TypeScript + SQLite (better-sqlite3)
  - `packages/frontend`: React + TS + Tailwind + shadcn/ui (Radix) + Framer Motion
  - `packages/shared`: shared types and template helpers
- Dev proxy: frontend → `/api` (backend on 8000)
- Response shape: `{ success, data?, message?, error? }`

## 4. Frontend Stack & Design System

MUI design.

## 5. User Flows

1. User registers or logs in.
2. (Optional) User connects GitHub in Settings (store encrypted PAT). If absent and user is admin, server uses admin token/org.
3. User creates a project or clones a template.
   - Option: "Publish to GitHub" (default on if connected); set repo name/visibility.
   - Backend creates repo and pushes starter files (README, .gitignore, Dockerfile, scaffold).
4. User updates project settings (config fields).
5. User generates and views Dockerfile; when publishing is enabled, commit updated Dockerfile.
6. Admin (optional) checks counts endpoint.

## 6. Filtering UX (Projects & Templates)

- Desktop: left sidebar, sticky, chip-based sections with counts; no dropdowns.
- Mobile: bottom drawer with same content.
- Sections (multi-select chips where applicable):
  - Quick: New, Official, Verified, Requires GPU (booleans if present).
  - Use Cases: chips with counts.
  - Capabilities: chips with counts.
  - Robots: chips with counts.
  - Simulators: chips with counts.
  - Tags: user tags + freeform add; suggestions; chips with counts.
- Above results: Active filters summary row with removable chips and "Clear all".
- Accessibility: keyboard navigation, ARIA labels, visible focus states.

## 7. Routes & Pages

- `/login` (public), `/register` (public)
- `/projects` (auth): list with sidebar filters, create, details, delete, clone, convert-to-template (admin)
- `/templates` (auth): list with sidebar filters, launch (clone)
- `/settings` (auth): profile, change password, connect GitHub (PAT)
- `/profile` (auth)
- `/admin` (auth, admin-only; optional counts)
- `*` NotFound

## 8. API (JSON)

All responses: success → `{ success: true, data }`, errors → `{ success: false, error }`.

Auth `/api/auth`

- POST `/signup` { username, email, password } → { user, token, expiresIn }
- POST `/login` { email, password } → { user, token, expiresIn }
- GET `/me` → { user }
- POST `/change-password` { currentPassword, newPassword }
- POST `/refresh` → { token, expiresIn }
- POST `/logout`

Projects `/api/projects`

- GET `/` `?search=&is_template=&tags[]=...&use_cases[]=...&capabilities[]=...&robots[]=...&simulators[]=...&sort_by=&sort_order=` → { projects[] }
- POST `/` { name, description?, config, github?: { createRepo?: boolean, visibility?: 'private'|'public', repoName?: string } } → { project, githubRepo? }
- GET `/:id` → { project }
- PUT `/:id/settings` { config, metadata? } → { project }
- POST `/:id/clone` { name?, github?: { createRepo?, visibility?, repoName? } } → { clonedProject, githubRepo? }
- POST `/:id/convert-to-template` { visibility?: 'public'|'private', version?: string } → { project } (admin only)
- DELETE `/:id`
- GET `/templates` same as list but filters `is_template=1` on server
- Facets: GET `/facets` and `/templates/facets` → { use_case_counts, capability_counts, robot_counts, simulator_counts, tag_counts }

Dockerfiles `/api/dockerfiles`

- GET `/:projectId` → { content, path, size, warnings, errors }
- POST `/:projectId/generate` { options? } → same shape as GET
- DELETE `/:projectId` → { message }

Integrations: GitHub `/api/integrations/github`

- POST `/connect` { token } → { connected: true, username }
- DELETE `/disconnect` → { connected: false }
- GET `/status` → { connected: boolean, username?: string }

Admin `/api/admin` (optional)

- GET `/dashboard` → { totalUsers, totalProjects, templates }

Notes:

- Filters are SQLite-safe (`LOWER(name) LIKE ?`).
- Facets computed in-app over the filtered result set (cacheable per search key).

## 9. Database (SQLite)

Tables

- `users`
  - id TEXT (uuid), email TEXT UNIQUE, username TEXT UNIQUE, password_hash TEXT,
    role TEXT ('user'|'admin'), is_active INTEGER, created_at, updated_at
  - github_token_encrypted TEXT NULL, github_username TEXT NULL, github_connected INTEGER DEFAULT 0
- `projects`
  - id TEXT (uuid), name TEXT, description TEXT, owner_id TEXT (FK users),
    is_active INTEGER, is_template INTEGER DEFAULT 0, type TEXT, version TEXT,
    tags TEXT (JSON string), config TEXT (JSON string), metadata TEXT (JSON string),
    created_at, updated_at
  - github_repo_owner TEXT NULL, github_repo_name TEXT NULL, github_repo_url TEXT NULL, github_repo_id INTEGER NULL
  - template_published_at DATETIME NULL, template_author TEXT NULL,
    template_version TEXT DEFAULT '1.0.0',
    template_visibility TEXT CHECK (template_visibility IN ('public','private')) DEFAULT 'public'

Metadata fields used for filtering (stored as JSON text in `projects.metadata`)

- use_cases: string[]; capabilities: string[]; robots: string[]; simulators: string[]

Indexes

- users(email), users(username)
- projects(owner_id), projects(name), projects(is_template), projects(created_at), projects(updated_at)

## 10. GitHub Integration (MVP)

- Server token (admin): `GITHUB_TOKEN` (optionally `GITHUB_FORK_ORG`)
- User connection: PAT stored encrypted with `APP_ENCRYPTION_KEY` (symmetric), never logged; redacted in errors
- Create Repo: authenticated user (if connected) or admin token/org; wait until repo is available
- Commit Files: README.md, .gitignore, Dockerfile, basic scaffold (via GitHub contents API)
- Update Commits: Dockerfile updates on regenerate (if publishing enabled)
- Timeouts, rate limits, and retry with backoff

## 11. Security & Validation

- JWT bearer auth; route guards; admin checks
- Joi validation middleware; input sanitization; Helmet; CORS
- Token handling: encrypt user PATs, redact logs, never echo tokens to client

## 12. Configuration

- `GITHUB_TOKEN` (admin)
- `GITHUB_FORK_ORG` (optional)
- `APP_ENCRYPTION_KEY` (user token encryption)
- `PORT`, `CORS_ORIGIN` for server

## 13. Non-Functional

- Migrations run on startup; idempotent
- Structured logging; request IDs; global error handler
- SQLite-only; no Postgres operators; queries normalized
- UI performance: facet caching and incremental rendering

## 16. Docker & Docker Compose

- Goal: Run frontend and backend in Docker; provide a docker-compose setup for local dev and simple prod.

- Compose services (dev):

```yaml
version: '3.8'
services:
  backend:
    build:
      context: .
      dockerfile: ./packages/backend/Dockerfile
    environment:
      - NODE_ENV=development
      - PORT=8000
      - CORS_ORIGIN=http://localhost:3000
      - GITHUB_TOKEN=${GITHUB_TOKEN-}
      - GITHUB_FORK_ORG=${GITHUB_FORK_ORG-}
      - APP_ENCRYPTION_KEY=${APP_ENCRYPTION_KEY-}
    ports:
      - '8000:8000'
    volumes:
      # mount only source to preserve node_modules inside the image
      - ./packages/backend/src:/app/packages/backend/src
      - ./packages/backend/generated:/app/packages/backend/generated
    restart: unless-stopped

  frontend:
    build:
      context: .
      dockerfile: ./packages/frontend/Dockerfile
    environment:
      - NODE_ENV=development
      - BACKEND_URL=http://backend:8000
    ports:
      - '3000:3000'
    volumes:
      - ./packages/frontend/src:/app/packages/frontend/src
      - ./packages/frontend/public:/app/packages/frontend/public
    depends_on:
      - backend
    restart: unless-stopped

volumes:
  # named volume could be used if we externalize SQLite DB path
  # robium-sqlite:
```

- Dockerfiles (baseline):

  - `packages/backend/Dockerfile`
    - Base: `node:18-alpine`
    - WORKDIR `/app/packages/backend`
    - Copy package.json/lock → `npm ci`
    - Copy source → build (tsc) → expose 8000 → `CMD ["node","dist/index.js"]`
  - `packages/frontend/Dockerfile`
    - Base: `node:18-alpine`
    - WORKDIR `/app/packages/frontend`
    - Copy package.json/lock → `npm ci`
    - Copy src/public → expose 3000 → `CMD ["npm","start"]`

- SQLite persistence:

  - Default DB path under `packages/backend/generated/robium.db` (already in repo). Mount that path for durability in dev.

- Dev workflow:

```bash
docker compose up --build
# open http://localhost:3000 (frontend) and http://localhost:8000/health (backend)
```

- Prod notes (simple):
  - Build images with `NODE_ENV=production` and run backend with compiled output.
  - Frontend can be built and served by the backend (static) or via a separate static server; backend already serves build when `NODE_ENV=production`.

## 14. Milestones

1. Backend core: Auth, Projects, Dockerfiles (SQLite), GitHub publish, consistent responses
2. Frontend core: Auth pages, Projects (list/create/details), Templates (list), Dockerfile view, Settings → Connect GitHub
3. Admin counts (optional), tests, docs, polish
4. Future: containers/WS, advanced catalog features, OAuth GitHub App (replace PAT storage)

## 16. Developer Scripts & Tooling (Convenience)

- Root-level npm scripts (workspaces aware):

  - Development
    - `npm run dev` → run frontend and backend in parallel
    - `npm run dev:frontend` → run frontend only
    - `npm run dev:backend` → run backend only
  - Build
    - `npm run build` → build all workspaces
    - `npm run build:frontend` → build frontend
    - `npm run build:backend` → build backend
  - Start/Run
    - `npm run start:backend` → start compiled backend server
  - Clean
    - `npm run clean` → remove build artifacts and generated files
  - Database/Seeds (SQLite):
    - `npm run db:migrate` → run migrations (startup also runs migrations)
    - `npm run seed:users` → seed sample users
    - `npm run seed:projects` → seed sample projects
    - `npm run seed:templates` → seed sample templates
    - `npm run seed:all` → seed users + projects + templates
    - `npm run db:reset` → destructive reset of SQLite DB + rerun migrations

- Suggested root package.json entries:

```json
{
  "scripts": {
    "dev": "npm-run-all --parallel dev:frontend dev:backend",
    "dev:frontend": "npm run dev --workspace=packages/frontend",
    "dev:backend": "npm run dev --workspace=packages/backend",
    "build": "npm-run-all build:frontend build:backend",
    "build:frontend": "npm run build --workspace=packages/frontend",
    "build:backend": "npm run build --workspace=packages/backend",
    "start:backend": "npm run start --workspace=packages/backend",
    "clean": "rimraf packages/*/dist packages/*/build packages/backend/generated && rimraf node_modules packages/*/node_modules",
    "db:migrate": "npm run migrate --workspace=packages/backend",
    "seed:users": "npm run seed:users --workspace=packages/backend",
    "seed:projects": "npm run seed:projects --workspace=packages/backend",
    "seed:templates": "npm run seed:templates --workspace=packages/backend",
    "seed:all": "npm run seed:all --workspace=packages/backend",
    "db:reset": "npm run db:reset --workspace=packages/backend"
  }
}
```

- Backend workspace scripts (map to existing script files under `packages/backend/src/scripts/`):

  - `migrate`: `ts-node src/scripts/migrate.ts up`
  - `seed:users`: `ts-node src/scripts/seed-users.ts`
  - `seed:projects`: `ts-node src/scripts/populate-projects.ts`
  - `seed:templates`: `ts-node src/scripts/seed-sample-data.ts --templates`
  - `seed:all`: `ts-node src/scripts/seed-all-sample-data.ts`
  - `db:reset`: `ts-node src/scripts/migrate.ts reset && ts-node src/scripts/migrate.ts up`

- One-liners for common tasks:

```bash
# Start dev environment (both apps)
npm run dev

# Build and run backend
npm run build && npm run start:backend

# Seed everything
npm run db:migrate && npm run seed:all

# Clean and reinstall
npm run clean && npm install && npm run dev
```

## 15. Acceptance Criteria

- Register/login, persist JWT, fetch `/me`
- Create project, edit settings, clone template, delete
- List templates and clone them
- Generate Dockerfile and view content
- When "Publish to GitHub" is enabled: new repo is created and initialized; project persists `github_*` fields
- Projects and Templates pages feature a left sidebar with chip-based filters (use cases, capabilities, robots, simulators, tags), facet counts, and keyboard accessibility
- All endpoints use uniform response shape
- App boots clean; SQLite migrations apply; no Postgres-only code
