#!/bin/bash

# Build script for cobot-autonomy container
# Usage: ./build.sh [--rosv ROS_VERSION] [IMAGE_NAME]

set -e

# Show help if requested
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    echo "Usage: ./build.sh [--rosv ROS_VERSION] [IMAGE_NAME]"
    echo ""
    echo "Arguments:"
    echo "  --rosv ROS_VERSION    ROS version to build (noetic, humble, foxy) [default: noetic]"
    echo "  IMAGE_NAME            Docker image name [default: cobot-autonomy]"
    echo ""
    echo "Examples:"
    echo "  ./build.sh                                    # Build with ROS Noetic"
    echo "  ./build.sh --rosv noetic                      # Build with ROS Noetic"
    echo "  ./build.sh --rosv humble                      # Build with ROS Humble"
    echo "  ./build.sh --rosv foxy                        # Build with ROS Foxy"
    echo "  ./build.sh --rosv noetic my-image             # Build with custom image name"
    exit 0
fi

# Default values
ROS_VERSION="noetic"
IMAGE_NAME="cobot-autonomy"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --rosv)
      ROS_VERSION="$2"
      shift 2
      ;;
    --*)
      echo "ERROR: Unknown flag $1"
      echo "Use './build.sh --help' for usage information"
      exit 1
      ;;
    *)
      IMAGE_NAME="$1"
      shift
      ;;
  esac
done

# Validate ROS version
if [[ "$ROS_VERSION" != "noetic" && "$ROS_VERSION" != "humble" && "$ROS_VERSION" != "foxy" ]]; then
    echo "ERROR: ROS version must be noetic, humble, or foxy, got: $ROS_VERSION"
    echo "Use './build.sh --help' for usage information"
    exit 1
fi

echo "Building cobot-autonomy container..."
echo "ROS Version: $ROS_VERSION"
echo "Image Name: $IMAGE_NAME"

# Build the Docker image
docker build \
    --build-arg BUILD_ROSV=$ROS_VERSION \
    -t $IMAGE_NAME \
    -f Dockerfile \
    .

echo -e "\033[32m✓ Build completed successfully!\033[0m"
echo -e "\033[36mTo build and run the container (only need to run once):\033[0m"
echo -e "  \033[1m./container.sh --rosv $ROS_VERSION --name <container_name> <image_name>\033[0m"
echo -e "\033[36mOnce built, use:\033[0m"
echo -e "  \033[1mdocker start <container_name> && docker attach <container_name>\033[0m"