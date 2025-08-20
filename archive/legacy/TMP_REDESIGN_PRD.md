# Robium (Reimplementation) — Project Description

## Vision

A focused web platform to create/manage robotics project configurations and generate Dockerfiles from templates, with a minimal, reliable stack (SQLite + Express + React). Start small (Auth, Projects, Templates, Dockerfiles), keep DX high, and expand later.

## In Scope (MVP)

- Authentication: signup/login/jwt/me/change-password/logout
- Projects: create/list/get/update-settings/delete/clone; admin-only convert-to-template
- Templates: browse/search public templates (subset of projects with is_template=1)
- Dockerfiles: generate on demand from project config; cache and view
- Admin (minimal): counts-only dashboard endpoint (if desired)
- SQLite, consistent API responses
- GitHub integration (MVP): create a new repo and push initial files (Dockerfile + starter source) during project create/clone. If user connected GitHub, use their token; otherwise, for admin users use server org/user via admin token; for non-admins without GitHub, allow skipping

## Architecture

- Monorepo (npm workspaces)
  - packages/backend: Express + TypeScript + SQLite (better-sqlite3)
  - packages/frontend: React + TS + Tailwind + shadcn/ui (Radix UI) + Framer Motion
  - packages/shared: types, simple schemas, and template helpers
- Dev proxy: frontend → /api (8000)
- Single response shape: { success, data?, message?, error? }

## Tech Stack

- Backend: Node 18+, Express, better-sqlite3, JWT, Joi, Helmet, CORS, Morgan
- Frontend (default): React 18, React Router, Tailwind CSS, shadcn/ui (Radix primitives), Framer Motion, Axios, Lucide icons, Recharts or VisX (charts), optional react-three-fiber (subtle 3D)
- Shared: TS types; simple JSON-based templates
- Infra: Dockerfile generation to /generated/<projectId>/

## Brand, Design System, and UI Stack

- Default pick (recommended for AI/robotics “modern + dynamic”):
  - Tailwind CSS + shadcn/ui (Radix) + Framer Motion
  - Why: clean, current look; accessible primitives; fast theming; motion-rich UX
  - Look/feel: dark-first, minimal, high contrast with electric accent (cyan/azure or lime)
  - Motion: Framer Motion for micro-interactions (fade/slide on reveal; hover scale ≤1.02)
  - Optional: subtle 3D via react-three-fiber where it adds value (hero, previews)
- Alternative path (if you want MUI ecosystem/DataGrid):
  - MUI Core + MUI Joy + Framer Motion
  - Theming: dark-first, custom radii (12–16px), high-contrast palette, tighter shadows/typography
  - Avoid stock look: override shadows/radii/typography, prefer Joy components where possible
- Starter tokens (works for either stack):
  - Radius: 12–16px; Spacing: 8/12/16/24
  - Colors: neutral gray base; electric accent (cyan/azure or lime)
  - Shadows: low-blur/high-spread for depth in dark UIs
  - States: subtle spring transitions, 150–250ms interaction timings

## User Flows

1. Visitor registers or logs in
2. (Optional, recommended) User connects GitHub in Settings
   - If user connects: we store an encrypted GitHub token; future creates/clones publish to their GitHub
   - If not connected and user is admin: server will use admin token/org for publishing
   - If not connected and user is not admin: they can still create locally, or connect GitHub later and publish on demand
3. User creates a project from scratch or clones a template
   - Option: "Publish to GitHub" (default on if connected). User can set repo name/visibility
   - On submit, backend creates repo and pushes initial files (README, .gitignore, Dockerfile, basic scaffold)
4. User updates project settings (basic config fields)
5. User generates and views the Dockerfile (auto-updates GitHub via commit if publishing was enabled)
6. Admin (optional) checks counts endpoint

## Frontend

- Global providers: Theme, Auth, Error, Navigation, Toast, Accessibility
- Routes
  - /login (public)
  - /register (public)
  - /projects (auth)
    - /projects/new (auth)
    - /projects/:projectId (auth)
  - /templates (auth)
  - /settings (auth)
  - /profile (auth)
  - /admin (auth, admin-only; optional)
  - - (NotFound)
- Navigation (top bar + mobile drawer): Projects, Templates; Profile menu: Profile, Settings, Admin (if admin), Logout
- Pages (MVP details)
  - Login/Register: email/password + username for register
  - Projects: list, filter by name; create form; details with config editor (simple JSON form); actions: clone, delete; button to generate Dockerfile and view
  - Templates: searchable list of is_template=1 projects; “Clone” into user space
  - Profile/Settings: update profile fields, change password; Connect GitHub (paste PAT for MVP) and view connection status
  - Admin (optional): show counts for users/projects/templates
- API client: unified error handling; token injection; 401 redirect

### Filtering UX (Projects & Templates)

- Left sidebar (desktop): sticky, non-scrolling block with chip-style filters (no dropdowns). Mobile: bottom drawer.
- Visual style: Hugging Face–like pill chips with compact icons and counts; multi-select; active-state highlighting.
- Sections (both pages; some may be empty on Projects depending on data):
  - Quick: New, Official, Verified, Requires GPU (boolean chips)
  - Use Cases: multi-select chips with counts
  - Capabilities: multi-select chips with counts
  - Robot Types/Targets: multi-select chips with counts
  - Simulators: multi-select chips with counts
  - Difficulty: beginner/intermediate/advanced chips
  - Licenses: chips
  - Tags: existing tag chips + freeform add; auto-suggest tags
- Active filters summary row above results with removable chips and “Clear all”.
- Accessibility: full keyboard navigation; ARIA labels; focus rings.
- Performance: facet counts reflect current search term; update instantly on chip toggle.

## API (all JSON, 2xx → { success: true, data }, errors → { success: false, error })

- Auth (/api/auth)
  - POST /signup { username, email, password } → { user, token, expiresIn }
  - POST /login { email, password } → { user, token, expiresIn }
  - GET /me → { user }
  - POST /change-password { currentPassword, newPassword }
  - POST /refresh → { token, expiresIn }
  - POST /logout
- Projects (/api/projects)
  - GET /?search=&type=&is_template= → { projects[] }
  - POST / { name, description?, config, github?: { createRepo?: boolean, visibility?: 'private'|'public', repoName?: string } } → { project, githubRepo? }
  - GET /:id → { project }
  - PUT /:id/settings { config, metadata? } → { project }
  - POST /:id/clone { name?, github?: { createRepo?: boolean, visibility?: 'private'|'public', repoName?: string } } → { clonedProject, githubRepo? }
  - DELETE /:id
  - GET /templates?search=&category?&type? → { templates[] } (server filters is_template=1; SQLite-safe LIKE)
  - POST /:id/convert-to-template { visibility?: 'public'|'private', version?: string } → { project } (admin only)
  - Filtering params (applies to both / and /templates):
    - use_cases[]=, capabilities[]=, robot_targets[]=, simulators[]=, ros_distros[]=, rmw[]=, difficulty[]=, licenses[]=, tags[]=, requires_gpu, official_only, verified_only, sort_by, sort_order
  - Facets:
    - GET /facets?search=&...same filters (except the facet being computed) → { use_case_counts, capability_counts, robot_target_counts, simulator_counts, difficulty_counts, license_counts, tag_counts }
    - GET /templates/facets?search=&... → same shape, computed over templates only
- Dockerfiles (/api/dockerfiles)
  - GET /:projectId → { content, path, size, warnings, errors }
  - POST /:projectId/generate { options? } → { ...same as GET }
  - DELETE /:projectId → { message }
- Integrations: GitHub (/api/integrations/github)
  - POST /connect { token } → { connected: true } (MVP: store user PAT encrypted)
  - DELETE /disconnect → { connected: false }
  - GET /status → { connected: boolean, username?: string }
- Admin (optional) (/api/admin)
  - GET /dashboard → { totalUsers, totalProjects, templates }

## Database (SQLite)

- users
  - id TEXT (uuid), email TEXT unique, username TEXT unique, password_hash TEXT, role TEXT ('user'|'admin'), is_active INTEGER, created_at, updated_at
  - github_token_encrypted TEXT (nullable), github_username TEXT (nullable), github_connected INTEGER DEFAULT 0
- projects
  - id TEXT (uuid), name TEXT, description TEXT, owner_id TEXT (FK users), is_active INTEGER, is_template INTEGER DEFAULT 0, type TEXT, version TEXT, tags TEXT (JSON string), config TEXT (JSON string), metadata TEXT (JSON string), created_at, updated_at
  - github_repo_owner TEXT (nullable), github_repo_name TEXT (nullable), github_repo_url TEXT (nullable), github_repo_id INTEGER (nullable)
  - template_published_at DATETIME (nullable), template_author TEXT (nullable), template_version TEXT DEFAULT '1.0.0', template_visibility TEXT CHECK (template_visibility IN ('public','private')) DEFAULT 'public'
- metadata fields used for filtering (stored as JSON text):
  - use_cases: string[]; capabilities: string[]; robots: string[]; simulators: string[]
- project_configurations (optional for future; MVP stores config in projects.config)
- sessions (optional, for refresh tokens later)
- admin_audit_trails (optional)
- indexes on users(email,username), projects(owner_id, name, is_template)
  - recommended: projects(created_at), projects(updated_at), projects(is_template)

Notes:

- JSON stored as TEXT; app parses/serializes.
- All queries SQLite-safe (LOWER(..) LIKE ?, no ILIKE/JSONB/arrays).
- Facet counts: computed in application layer over the filtered result set for MVP (load rows then aggregate), then cached per search key; future optimization can precompute or add FTS indexes.

## Dockerfile Generation (MVP)

- Input: minimal project config { baseImage, workdir, systemDependencies[], env{}, ports[], command }
- Output: generated Dockerfile under /generated/<projectId>/Dockerfile
- Options: includeComments, optimize, securityScan (simple string checks)
- If missing config, generate sensible defaults (python:3.11-slim, /app, bash)

## Security/Validation

- JWT bearer auth; 401 handling; role guard for admin route
- Joi validation on inputs; global sanitization middleware
- Helmet, CORS (configurable), audit logs later
- GitHub tokens: stored encrypted at rest (MVP: symmetric encryption with app key); never logged; redact in errors

## Non-Functional

- Migrations run on startup; idempotent
- Structured logging; request IDs; global error handler
- Timeouts on axios; 60s backend timeouts reasonable
- UI stack baseline: Tailwind + shadcn/ui + Framer Motion (MUI path documented above if chosen)
- GitHub API: rate-limit aware (retry with backoff), timeouts, configurable org fallback for admin

## Configuration

- Environment variables
  - GITHUB_TOKEN: server/admin token for publishing under admin account or org (required for admin path)
  - GITHUB_FORK_ORG: optional GitHub org to create/fork repos into for admin
  - APP_ENCRYPTION_KEY: symmetric key to encrypt/decrypt user GitHub tokens at rest (MVP)

## Milestones

1. Core backend: Auth, Projects, Dockerfiles (SQLite), GitHub integration (create repo + push starter), consistent responses
2. Frontend: Auth pages, Projects list/create/details (with "Publish to GitHub"), Templates list, Dockerfile view, Settings → Connect GitHub
3. Admin counts (optional), polishing, tests, docs
4. Future: Containers/WS, ROS features, datasets, workspace, OAuth GitHub app (replace PAT storage)

## Acceptance Criteria (MVP)

- Can register/login, persist JWT, fetch /me
- Can create project, edit settings, clone template, delete
- Can list templates and clone them
- Can generate Dockerfile and view its content
- When "Publish to GitHub" is enabled, a new repo is created and populated with README, .gitignore, Dockerfile, and starter files; repo info is persisted on the project
- Projects and Templates pages provide a left sidebar with chip-based filters (no dropdowns) including Use Cases, Capabilities, Robot Types, Simulators, Difficulty, License, Tags, and quick boolean chips; facet counts update with search; filters are keyboard accessible
- All endpoints return uniform { success, data } or { success: false, error }
- No Postgres-only code; app boots clean; migrations apply; SQLite file persists
