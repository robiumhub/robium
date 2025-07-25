# Overview  
Robium is a web-based robotics development studio IDE that enables users to create, simulate, and run robotics applications by connecting modular ROS2 components. It targets robotics developers, educators, and hobbyists who need a streamlined, intelligent environment for building ROS2-based applications. Robium solves the complexity of setting up, wiring, and debugging ROS2 projects by providing containerized execution environments and a robust project development workflow. Its value lies in accelerating robotics development, reducing errors, and making advanced robotics accessible to a broader audience.

# Core Features  
- **Project Development Environment Setup**
  - Monorepo structure with clear separation for frontend, backend, and shared code.
  - Docker Compose for orchestrating multi-service development (frontend, backend, ROS2, etc.).
  - TypeScript for type safety in frontend and backend codebases.
  - ESLint and Prettier for code quality and consistent formatting.
  - Environment variable management with .env files and templates.
  - Pre-commit hooks for linting, formatting, and tests.
  - Continuous Integration (CI) for automated testing and linting on every push/PR.
  - Editor/IDE configuration for consistent developer experience (e.g., .vscode, .editorconfig).
  - Comprehensive documentation and onboarding guides.
- **Global Colcon Workspace and Unified Base Image**
  - All ROS algorithms/packages are maintained in a single global colcon workspace for integration testing and validation.
  - The Docker base image is built to support all packages in the global workspace, ensuring compatibility and reducing per-project build complexity.
  - Even if user projects require only a subset of packages, the base image and global workspace ensure all packages are always available and buildable.
- **ROS Algorithm Suite and Task Categories**
  - A curated suite of ROS algorithms/packages organized by meta-categories and task categories (e.g., actuator, camera, remote control, arm control, voice control, mapping, localization, planning, person tracking, person recognition, character animation, etc.).
  - Each category contains multiple solution alternatives (algorithms/packages), each with a clear task definition, input/output relations, and a list of ROS packages used.
  - All packages are hosted in a global catkin workspace for integration testing and validation.
  - When a user creates a project and selects tasks, only the required packages are copied to their workspace, ensuring minimal and relevant project environments.
  - Emphasizes modularity, clear task definitions, IO relations, and flexible solution selection for robotics development.
- **Containerized Project Environment with Persistent Workspace and Strict Isolation**
  - Each project runs in its own isolated Docker container, built FROM a custom base image with ROS 2 Humble pre-installed.
  - Each project has a dedicated workspace and source files stored persistently on the host filesystem, mounted into the container as a volume.
  - Projects are portable and can be cloned or run locally.
  - Comprehensive container management system with automated lifecycle and resource isolation.
  - Persistent storage enables versioning, backup, user access, and robust development workflows.
  - **Strict isolation:** Each project container is strictly isolated and cannot access the volumes, files, or data of any other project. This is enforced via Docker volume scoping, network isolation, and security policies.
  - **UI-Driven Configuration:** All project, container, and workspace configurations—including isolation, volumes, dependencies, and environment variables—are manageable and configurable from the Robium web UI.
- **Project Configuration Management**
  - Standardized configuration schema for project metadata, dependencies, and environment settings.
  - Automated Dockerfile and docker-compose generation from project configs.
- **Project Versioning and Sharing**
  - Built-in version control and project sharing (no real-time collaboration in MVP).
- **Execution and Debugging**
  - One-click execution launches all nodes, with per-node logs, live RViz, and rosbag playback.
- **User Authentication and Project Ownership**
  - Users can sign up and sign in to the platform.
  - Each user has their own list of projects, which are private by default.
  - Users can create as many projects as they want.
  - Admin users can see and manage all projects; regular users can only see and manage their own projects.
- **Minimal Backend**
  - Provides component metadata, rosbag hosting, LLM proxy, and WebSocket sync.
  - Centralized container and project management with comprehensive REST/WebSocket APIs.
- **Simulator Support**
  - Embedded Gazebo with predefined worlds; supports object spawning and sensor emulation via ROS interfaces.

# User Experience  
- **Personas**: Robotics developers, educators, students, hobbyists.
- **Key Flows**:
  1. Create new project → initializes Docker/ROS2 workspace with custom base image.
  2. Configure project settings → define dependencies, environment variables, simulation settings.
  3. Add and configure tasks and algorithms via UI forms, code, or LLM assistant.
  4. Inspect/edit component code, adjust parameters.
  5. Click "Run" → launch nodes, view logs, RViz, Gazebo.
  6. Share/export project or run locally.
- **UI/UX Considerations**:
  - Intuitive project and task configuration forms.
  - Clear visualization of component/task relationships.
  - Embedded terminals, RViz, and Gazebo views.
  - Guided wiring and parameter editing with LLM suggestions.
  - Project configuration editor with validation and suggestions.
  - **All configuration and management actions (project settings, container options, workspace setup, environment variables, and isolation policies) are accessible and editable through the Robium web UI.**
  - **User authentication and project management:** Users sign up/sign in, see only their own projects, and can create/manage as many projects as they want. Admins can see and manage all projects.
</context>
<PRD>
# Technical Architecture  
- **Development Environment**: Monorepo structure with Docker Compose for multi-service orchestration, TypeScript for frontend/backend, ESLint/Prettier, .env management, pre-commit hooks, CI, and editor config. All services and tooling are containerized for reproducibility and onboarding.
- **Global Colcon Workspace and Base Image**: All ROS packages are maintained in a single global colcon workspace. The Docker base image is built from this workspace and includes all packages, ensuring that every project container can support any combination of packages as needed. This approach guarantees compatibility, simplifies user project builds, and enables robust integration testing.
- **Component Registry and Algorithm Suite**: Central metadata store for ROS2 components and a curated suite of ROS algorithms/packages, organized by meta-categories and task categories. Each algorithm/package entry includes task definition, input/output relations, and a list of ROS packages used. All packages are maintained in a global catkin workspace for integration testing and validation. When a user creates a project and selects tasks, only the required packages are copied to their workspace for efficient, tailored project builds.
- **Frontend**: Web-based UI (React or similar) for project and task configuration, code inspection, parameter editing, and terminal output. Provides full UI for configuring project settings, container options, workspace volumes, dependencies, and environment variables. Includes user authentication (sign in/sign up), user-specific project lists, and admin dashboard for managing all projects. 
- **Execution Environment**: Each project runs in its own Docker container, which is built FROM a custom base Docker image with ROS 2 Humble pre-installed. This base image is maintained by Robium and ensures all projects use a consistent, up-to-date ROS 2 environment. Each project container adds its own colcon workspace and dependencies on top of this base image. The project’s workspace and source files are stored persistently on the host filesystem and mounted into the container as a volume (e.g., /workspace), enabling versioning, backup, and direct user access. **Strict isolation is enforced:** Project containers cannot access the volumes, files, or data of any other project, using Docker volume scoping and network isolation.
- **Container Management System**: Automated creation, naming, and teardown of Docker containers for each user/project with unique naming conventions (robium_{user_id}_{project_id}). Includes resource limits, network isolation, and scheduled cleanup of idle containers.
- **Project Configuration System**: Standardized JSON/YAML schema for project metadata, ROS2 dependencies, environment variables, and simulation settings. Automated generation of Dockerfiles and docker-compose files from project configurations.
- **Simulator/Visualizer**: Embedded RViz and Gazebo with rosbag support.
- **Backend Services**: The MVP will include a minimal backend (Node.js or Python/FastAPI) for metadata APIs, user/project management, and WebSocket sync. This will evolve into a more comprehensive MCP (Mission Control Plane) server in the future. **Implements user authentication, project ownership, and role-based access control (admin vs. regular user).**
- **Data Models**: ROS2 component metadata, project definitions, user settings, version snapshots, container registry, and configuration schemas.
- **APIs/Integrations**: REST/GraphQL for metadata, WebSocket for live sync, LLM API proxy, Docker API for container management, container lifecycle APIs, project configuration APIs.
- **Infrastructure**: Local and cloud deployment, Docker-based isolation, persistent storage for projects and bags. The base Docker image is stored in a registry and updated as needed for new ROS 2 releases or global tools.
- **Security Framework**: Container security hardening with non-root users, least privilege capabilities, network isolation, resource limits, and automated vulnerability scanning. **Strict project isolation is enforced at the container and volume level to prevent cross-project access.**

# Development Roadmap  
The development will prioritize getting a core, usable product for the MVP, focusing on the essential features that allow a user to create, configure, and run a basic robotics project.

- **MVP Requirements**:
  - **Phase 1: Foundation & Core Backend**
    - Project development environment setup (monorepo, Docker Compose, TypeScript, linting, .env, pre-commit hooks, CI, editor config, documentation).
    - Minimal backend for metadata, session management, and WebSocket sync.
    - User authentication and project management (user/project DB models, signup/login APIs, project ownership).
    - Project configuration schema definition.
  - **Phase 2: ROS Assets & Containerization**
    - Global colcon workspace for all ROS packages, with a unified Docker base image supporting all packages.
    - ROS algorithm suite and task categories with metadata and modular selection.
    - Container management system with automated lifecycle and resource isolation.
    - Automated Dockerfile generation from the project configuration schema.
  - **Phase 3: Frontend & User Interaction**
    - Containerized execution with per-project Docker environments, built FROM the shared base image.
    - Execution and debugging through a web terminal.
    - UI for user authentication, project creation/selection, and task configuration.
  - **Phase 4: Finishing Touches**
    - Project versioning and sharing (simple, non-collaborative).
    - Automated maintenance procedures for base image updates and security patches.
- **Future Enhancements**:
  - LLM integration and intelligent task curation (LLM assistant with access to metadata, task curation, solution suggestions, and project a structure guidance).
  - MCP server for centralized container and project management with comprehensive REST/WebSocket APIs, authentication, resource tracking, and maintenance automation.
  - Advanced security and resource isolation framework (non-root users, least privilege, advanced network isolation, vulnerability scanning, audit logging, etc.).
  - Multi-user collaboration and real-time editing.
  - Rviz and gazebo support
  - Component marketplace/library.
  - User roles, permissions, analytics, and learning-based recommendations.

# Logical Dependency Chain
The project will be built in phases, ensuring a logical progression from backend fundamentals to a feature-complete user-facing application.

- **Phase 1: Core Infrastructure and Services**
  - **Goal:** Establish the developer environment and the foundational backend services.
  - 1. **Setup Development Environment:** Configure the monorepo, Docker Compose, linters, and CI/CD pipeline.
  - 2. **Minimal Backend & User Auth:** Implement the core backend server, define data models (User, Project), and set up APIs for user signup, login, and session management.
  - 3. **Project Configuration Schema:** Define the JSON/YAML structure for project configurations that will drive the rest of the application.

- **Phase 2: ROS Assets and Containerization**
  - **Goal:** Build the core robotics assets and the system for running them in isolation.
  - 1. **Build Global ROS Workspace & Base Image:** Create the comprehensive colcon workspace and bake it into a unified Docker base image.
  - 2. **Develop ROS Algorithm Suite:** Curate and document the initial set of ROS tasks and components.
  - 3. **Implement Container Management System:** Develop the backend logic to create, start, stop, and isolate project containers using the base image.
  - 4. **Automate Docker Generation:** Create the service that generates project-specific Dockerfiles from their configuration files.

- **Phase 3: Frontend Implementation and Integration**
  - **Goal:** Build the user interface and connect it to the backend services.
  - 1. **Implement UI Shell & Auth:** Create the main application layout and the login/signup flows.
  - 2. **Project Management UI:** Build the dashboard for users to see, create, and manage their projects.
  - 3. **Project Configuration UI:** Develop the interface for users to add tasks from the algorithm suite and configure their project.
  - 4. **Execution and Debugging UI:** Integrate the web terminal and wire up the "Run" functionality to the container management system.

- **Phase 4: MVP Polish and Deployment**
  - **Goal:** Add final features for the MVP and prepare for initial deployment.
  - 1. **Implement Project Versioning/Sharing:** Add basic functionality to export or share a project.
  - 2. **Integrate Simulator/Visualizer:** Add basic support for RViz and Gazebo.
  - 3. **Establish Maintenance Procedures:** Document and automate the process for updating the base image and dependencies.

- **Guiding Principle:** Each feature is atomic and can be improved iteratively. The primary goal is to establish a usable create-configure-run loop as early as possible.

# Risks and Mitigations  
- **Technical Challenges**: Integrating ROS2, Docker, and web technologies; mitigated by leveraging existing open-source tools and clear interface boundaries.
- **Container Security**: Ensuring proper isolation and security; mitigated by implementing security hardening, resource limits, and automated vulnerability scanning.
- **Resource Management**: Managing multiple user containers efficiently; mitigated by automated lifecycle management, resource limits, and scheduled cleanup.
- **MVP Scope Creep**: Strictly limit features to MVP list; defer advanced features to future phases.
- **Resource Constraints**: Prioritize core flows; use modular architecture for incremental delivery.
- **LLM Integration**: Ensuring useful, accurate suggestions; mitigated by schema-driven guidance and fallback documentation.
- **Maintenance Overhead**: Keeping base images and security patches updated; mitigated by automated maintenance procedures and notification systems.

# Appendix  
- Research: Review of existing robotics IDEs, ROS2 best practices, and LLM integration patterns.
- Technical Specs: ROS2 Foxy/Humble, Docker, React, FastAPI/Node.js, RViz, Gazebo, WebSocket, REST/GraphQL, OpenAI/Anthropic LLM APIs.
- Container Management: Docker API, container orchestration, resource isolation, security hardening, automated lifecycle management.
- Project Configuration: JSON/YAML schemas, Dockerfile generation, environment management, dependency tracking.
- Security Framework: Container security best practices, network isolation, resource limits, vulnerability scanning, audit logging. 
- **Development Environment**: Monorepo structure, Docker Compose, TypeScript, linting, .env management, pre-commit hooks, CI, editor config, and documentation for robust and reproducible development workflows. 
- **Algorithm Suite and Task Categories**: Meta-categories, task categories, and modular solution alternatives for robotics tasks. Each solution includes task definition, IO relations, and ROS package dependencies. Global catkin workspace for integration testing; user workspaces only include selected packages. 
- **LLM Integration and MCP Server**: LLM assistant for intelligent task curation and MCP server for centralized management are planned for future phases.
- **Advanced Security and Resource Isolation**: Advanced security features and resource isolation (beyond strict container/volume isolation) are planned for future phases. 
- **Global Colcon Workspace and Base Image**: All ROS packages are maintained and tested in a single global colcon workspace. The Docker base image is built from this workspace and supports all packages, ensuring compatibility and simplifying project builds. 