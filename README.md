# Robium

A web-based robotics development studio IDE that enables users to create, simulate, and run robotics applications by connecting modular ROS2 components.

## Getting Started

### Prerequisites

- [Docker](https://www.docker.com/get-started)
- [Node.js](https://nodejs.org/en/) (v18 or higher)
- [npm](https://www.npmjs.com/)

### Installation

1.  **Clone the repository:**

    ```sh
    git clone git@github.com:mdemirst/robium.git
    cd robium
    ```

2.  **Install dependencies:**

    ```sh
    npm install
    ```

3.  **Set up environment variables:**

    Create a `.env` file in `packages/frontend` and `packages/backend`. You can use the `.env.example` files (once created) as a template.

4.  **Run the development environment:**

    ```sh
    docker-compose up --build
    ```

    - Frontend will be available at [http://localhost:3000](http://localhost:3000)
    - Backend will be available at [http://localhost:8000](http://localhost:8000)

## Development Workflow

This is a monorepo using npm workspaces. Code is organized into `packages`:

- `packages/frontend`: The React web application.
- `packages/backend`: The Node.js backend server.
- `packages/shared`: Shared code between frontend and backend.

### Useful Scripts

- **Linting:**

  ```sh
  npm run lint
  ```

- **Formatting:**
  ```sh
  npm run format
  ```

### Pre-commit Hooks

This project uses `husky` and `lint-staged` to automatically lint and format your code before you commit.

### CI/CD

A GitHub Actions workflow is configured to run on every push and pull request to the `main` branch. It will install dependencies, lint the code, and run the build.
