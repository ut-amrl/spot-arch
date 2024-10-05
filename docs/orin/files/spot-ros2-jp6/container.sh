# Usage: ./container.sh [--name CONTAINER_NAME] [--flag1=bla1 --flag2=bla2 ...] [IMAGE_NAME]
#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Default image name if none is provided
DEFAULT_IMAGE_NAME="spot-ros2-jp6:${USER}"
DEFAULT_CONTAINER_NAME="spot-ros2-jp6-${USER}"

# Initialize variables for options and image name
FLAGS=""
IMAGE_NAME=""
CONTAINER_NAME=""

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --name) # Check for --name flag
      CONTAINER_NAME="$2"
      shift 2 # Shift past the flag and its value
      ;;
    --*) # Any other argument starting with "--" is treated as a flag
      FLAGS="$FLAGS $1"
      shift # Move to next argument
      ;;
    *)  # Anything else is treated as the image name
      IMAGE_NAME="$1"
      shift
      ;;
  esac
done

# If no image name is provided, use the default image name
IMAGE_NAME=${IMAGE_NAME:-$DEFAULT_IMAGE_NAME}
echo "Using image: $IMAGE_NAME"

# If no --name flag is provided, use a default container name
CONTAINER_NAME=${CONTAINER_NAME:-$DEFAULT_CONTAINER_NAME}
echo "Using container name: $CONTAINER_NAME"

echo "Additional flags: $FLAGS"

# Run the Docker container with the provided or default image name and flags
docker run -it \
    --name $CONTAINER_NAME \
    --hostname orin \
    --runtime nvidia \
    --network host \
    --ipc host \
    --workdir /root \
    --group-add dialout \
    --privileged \
    -e PULSE_SERVER=unix:/run/user/0/pulse/native \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    -e HOST_UID=${HOST_UID} \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v ${HOME}/.Xauthority:/root/.Xauthority:rw \
    -v /dev/dri:/dev/dri:ro \
    -v /run/user/${HOST_UID}/pulse/native:/run/user/0/pulse/native:rw \
    -v ${HOME}/.gitconfig:/root/.gitconfig:rw \
    -v /tmp:/tmp \
    -u 0:0 \
    $FLAGS \
    $IMAGE_NAME
