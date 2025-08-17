export interface ProjectFileSpec {
  path: string;
  content: string;
}

export class ProjectScaffoldService {
  static generateScaffold(projectName: string): ProjectFileSpec[] {
    const safeName = projectName
      .toLowerCase()
      .replace(/[^a-z0-9-_]+/g, '-')
      .slice(0, 100);

    const dockerfile = `# Dev-friendly base image
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
    bash ca-certificates curl git sudo build-essential \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Default command keeps the container running for multi-terminal dev
CMD ["bash"]
`;

    const dockerCompose = `version: '3.9'
services:
  app:
    build:
      context: .
      network: host
    container_name: ${safeName}-dev
    working_dir: /workspace
    command: tail -f /dev/null
    volumes:
      - ./:/workspace
    ports:
      - "8080:8080"
    dns:
      - 8.8.8.8
      - 1.1.1.1
`;

    const dockerignore = `node_modules
.git
.gitignore
*.log
dist
build
.cache
.venv
__pycache__
.DS_Store
`;

    const gitignore = `# Dependencies & build artifacts
node_modules/
dist/
build/
.cache/
.venv/
__pycache__/
.DS_Store
*.log
`;

    const readme = `# ${projectName}

Dev-ready containerized workspace for ${projectName}.

## Quickstart (preferred)

Use the helper scripts:

1) Start dev container (detached):

\`\`\`
bash scripts/dev-start.sh
\`\`\`

2) Open a shell (repeat for multiple terminals):

\`\`\`
bash scripts/dev-shell.sh
\`\`\`

3) Stop the environment:

\`\`\`
bash scripts/dev-stop.sh
\`\`\`

Changes in this repo are live-mounted at /workspace inside the container.

## Alternative (docker compose directly)

\`\`\`
docker compose up -d
docker compose exec app bash
docker compose down
\`\`\`
`;

    const devStart = `#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"
docker compose up -d
`;

    const devShell = `#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"
docker compose exec app bash
`;

    const devStop = `#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"
docker compose down
`;

    const placeholderSrc = `// Your source code lives here. This directory is bind-mounted into the container.
`;

    return [
      { path: 'Dockerfile', content: dockerfile },
      { path: 'docker-compose.yml', content: dockerCompose },
      { path: '.dockerignore', content: dockerignore },
      { path: '.gitignore', content: gitignore },
      { path: 'README.md', content: readme },
      { path: 'scripts/dev-start.sh', content: devStart },
      { path: 'scripts/dev-shell.sh', content: devShell },
      { path: 'scripts/dev-stop.sh', content: devStop },
      { path: 'src/.keep', content: placeholderSrc },
    ];
  }
}

export const projectScaffoldService = ProjectScaffoldService;


