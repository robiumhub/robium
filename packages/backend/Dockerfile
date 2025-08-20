FROM node:18-alpine

# Set working directory
WORKDIR /app/packages/backend

# Copy package files
COPY package*.json ./
COPY tsconfig.json ./

# Install dependencies
RUN npm ci --only=production

# Copy source code
COPY src ./src

# Build the application
RUN npm run build

# Expose port
EXPOSE 8000

# Start the application
CMD ["node", "dist/index.js"]
