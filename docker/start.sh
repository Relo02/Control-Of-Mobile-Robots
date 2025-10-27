#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Docker image name
IMAGE_NAME="ros2-humble:latest"
CONTAINER_NAME="ros2-humble-container"

# Default options
REMOVE_AFTER_EXIT=true
MOUNT_WORKSPACE=true
ENABLE_GUI=false
PRIVILEGED=false

# Function to show usage
show_usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -p, --persistent    Don't remove container after exit"
    echo "  -n, --no-mount      Don't mount current directory as workspace"
    echo "  -g, --gui           Enable GUI support (X11 forwarding)"
    echo "  --privileged        Run container in privileged mode"
    echo "  -h, --help          Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                  # Run with default settings"
    echo "  $0 -p -g            # Run persistent container with GUI support"
    echo "  $0 --no-mount       # Run without mounting workspace"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--persistent)
            REMOVE_AFTER_EXIT=false
            shift
            ;;
        -n|--no-mount)
            MOUNT_WORKSPACE=false
            shift
            ;;
        -g|--gui)
            ENABLE_GUI=true
            shift
            ;;
        --privileged)
            PRIVILEGED=true
            shift
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            show_usage
            exit 1
            ;;
    esac
done

# Check if Docker image exists
if ! docker image inspect ${IMAGE_NAME} > /dev/null 2>&1; then
    echo -e "${RED}✗ Docker image '${IMAGE_NAME}' not found${NC}"
    echo -e "${YELLOW}Please run './build.sh' first to build the image${NC}"
    exit 1
fi

# Build docker run command
DOCKER_CMD="docker run -it"

# Add remove flag if specified
if [ "$REMOVE_AFTER_EXIT" = true ]; then
    DOCKER_CMD="$DOCKER_CMD --rm"
else
    DOCKER_CMD="$DOCKER_CMD --name $CONTAINER_NAME"
fi

# Add privileged mode if specified
if [ "$PRIVILEGED" = true ]; then
    DOCKER_CMD="$DOCKER_CMD --privileged"
fi

# Add workspace mount if specified
if [ "$MOUNT_WORKSPACE" = true ]; then
    DOCKER_CMD="$DOCKER_CMD -v $(pwd):/workspace"
fi

# Add GUI support if specified
if [ "$ENABLE_GUI" = true ]; then
    DOCKER_CMD="$DOCKER_CMD -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
    # Allow X11 forwarding
    xhost +local:docker > /dev/null 2>&1
fi

# Add network host for ROS2 communication
DOCKER_CMD="$DOCKER_CMD --network host"

# Add the image name
DOCKER_CMD="$DOCKER_CMD $IMAGE_NAME"

echo -e "${YELLOW}Starting ROS2 Humble container...${NC}"
echo -e "${BLUE}Container settings:${NC}"
echo -e "  Remove after exit: ${REMOVE_AFTER_EXIT}"
echo -e "  Mount workspace: ${MOUNT_WORKSPACE}"
echo -e "  GUI support: ${ENABLE_GUI}"
echo -e "  Privileged mode: ${PRIVILEGED}"
echo ""

# Run the container
echo -e "${GREEN}Running: ${DOCKER_CMD}${NC}"
echo ""
eval $DOCKER_CMD

# Cleanup X11 permissions if GUI was enabled
if [ "$ENABLE_GUI" = true ]; then
    xhost -local:docker > /dev/null 2>&1
fi

echo -e "${GREEN}Container exited${NC}"
