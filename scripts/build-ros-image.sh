#!/bin/bash

set -e

VERSION=$(node -p "require('./package.json').version")
IMAGE_NAME="localhost:5000/robium-ros"
IMAGE_TAG="$IMAGE_NAME:$VERSION"

echo "Building ROS image with tag: $IMAGE_TAG"

docker build -t $IMAGE_TAG -f ros/Dockerfile .

echo "Pushing ROS image to local registry"

docker push $IMAGE_TAG 