FROM node:18-alpine

# Set working directory
WORKDIR /app

# Copy package.json and package-lock.json
COPY package.json ./
# COPY package-lock.json ./

# Install dependencies
RUN npm install

# Copy application source
COPY . .

# Expose port and start command will be defined later
# EXPOSE 3000
# CMD ["npm", "start"] 