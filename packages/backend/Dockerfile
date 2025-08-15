FROM node:18-alpine

# Set working directory
WORKDIR /app/packages/backend

# Copy only the backend package.json first (omit package-lock to allow resolving local file deps)
COPY packages/backend/package.json ./

# Copy the shared package so local file dependency can be installed
COPY packages/shared /app/packages/shared

# Install shared package dependencies
RUN cd /app/packages/shared && npm install

# Install dependencies locally without workspace context
RUN npm install

# Copy the backend source code
COPY packages/backend/src ./src
COPY packages/backend/jest.config.js packages/backend/tsconfig.json ./

EXPOSE 8000
CMD ["npm", "run", "dev"] 