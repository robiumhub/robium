FROM node:18-alpine

# Set working directory
WORKDIR /app/packages/frontend

# Copy only the frontend package.json first
COPY packages/frontend/package.json packages/frontend/package-lock.json* ./

# Install dependencies locally without workspace context
RUN npm install

# Copy the frontend source code
COPY packages/frontend/src ./src
COPY packages/frontend/public ./public
COPY packages/frontend/tsconfig.json ./

EXPOSE 3000
CMD ["npm", "start"] 