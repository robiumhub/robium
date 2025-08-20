FROM node:18-alpine

# Set working directory
WORKDIR /app/packages/frontend

# Copy package files
COPY package*.json ./
COPY tsconfig.json ./

# Install dependencies
RUN npm ci

# Copy source code and public files
COPY src ./src
COPY public ./public

# Expose port
EXPOSE 3000

# Start the development server
CMD ["npm", "start"]
