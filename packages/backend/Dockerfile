FROM node:18-alpine

# Set working directory
WORKDIR /app

# Copy root package.json and package-lock.json for monorepo install
COPY package.json package-lock.json ./
COPY packages/backend/package.json ./packages/backend/

# Install all dependencies (including workspaces)
RUN npm install

# Copy only the backend source code
COPY packages/backend ./packages/backend

WORKDIR /app/packages/backend

# Expose port and start command will be defined later
EXPOSE 8000
CMD ["npm", "run", "dev"] 