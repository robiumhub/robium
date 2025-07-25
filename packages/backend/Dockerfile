FROM node:18-alpine

# Set working directory
WORKDIR /app/packages/backend

# Copy only the backend package.json first
COPY packages/backend/package.json packages/backend/package-lock.json* ./

# Install dependencies locally without workspace context
RUN npm install

# Copy the backend source code
COPY packages/backend/src ./src
COPY packages/backend/jest.config.js packages/backend/tsconfig.json ./

EXPOSE 8000
CMD ["npm", "run", "dev"] 