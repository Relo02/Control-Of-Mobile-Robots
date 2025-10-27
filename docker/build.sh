#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Docker image name
IMAGE_NAME="ros2-humble"
TAG="latest"
FULL_IMAGE_NAME="${IMAGE_NAME}:${TAG}"

echo -e "${YELLOW}Building ROS2 Humble Docker image...${NC}"
echo -e "${YELLOW}Image name: ${FULL_IMAGE_NAME}${NC}"

# Build the Docker image
docker build -t ${FULL_IMAGE_NAME} .

# Check if build was successful
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Docker image built successfully!${NC}"
    echo -e "${GREEN}Image: ${FULL_IMAGE_NAME}${NC}"
    
    # Show image size
    echo -e "${YELLOW}Image size:${NC}"
    docker images ${IMAGE_NAME}
else
    echo -e "${RED}✗ Failed to build Docker image${NC}"
    exit 1
fi
