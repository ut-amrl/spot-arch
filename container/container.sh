# Usage: ./container.sh [--rosv ROS_VERSION] [--name CONTAINER_NAME] [--flag1=bla1 --flag2=bla2 ...] [IMAGE_NAME]
#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Show help if requested
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    echo "Usage: ./container.sh [--rosv ROS_VERSION] [--name CONTAINER_NAME] [--flag1=bla1 --flag2=bla2 ...] [IMAGE_NAME]"
    echo ""
    echo "Arguments:"
    echo "  --rosv ROS_VERSION    ROS version to use (noetic, humble, foxy) [default: noetic]"
    echo "  --name CONTAINER_NAME Container name [default: cobot-autonomy]"
    echo "  IMAGE_NAME            Docker image name [default: cobot-autonomy]"
    echo ""
    echo "Examples:"
    echo "  ./container.sh                                    # Run with default settings"
    echo "  ./container.sh --rosv noetic                      # Run with ROS Noetic"
    echo "  ./container.sh --rosv humble --name my-container  # Run with custom name"
    echo "  ./container.sh --rosv foxy my-image               # Run with custom image"
    echo "  ./container.sh --rosv noetic -- -v ~/my-dir:/workspace    # Run with additional Docker flags"
    exit 0
fi

# Default values
ROS_VERSION="noetic"
CONTAINER_NAME="cobot-autonomy"
IMAGE_NAME="cobot-autonomy"
FLAGS=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --rosv)
      ROS_VERSION="$2"
      shift 2
      ;;
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

# Validate ROS version
if [[ "$ROS_VERSION" != "noetic" && "$ROS_VERSION" != "humble" && "$ROS_VERSION" != "foxy" ]]; then
    echo "ERROR: ROS version must be noetic, humble, or foxy, got: $ROS_VERSION"
    echo "Use './container.sh --help' for usage information"
    exit 1
fi

echo -e "\033[36mUsing ROS version:\033[0m \033[1m$ROS_VERSION\033[0m"
echo -e "\033[36mUsing image:\033[0m \033[1m$IMAGE_NAME\033[0m"
echo -e "\033[36mUsing container name:\033[0m \033[1m$CONTAINER_NAME\033[0m"
echo -e "\033[36mAdditional flags:\033[0m \033[1m$FLAGS\033[0m"
echo -e "\033[36mHost UID:\033[0m \033[1m$HOST_UID\033[0m"

# Get current directory for mounting
CURRENT_DIR=$(pwd)

# Check if environment files exist (created by user's bashrc)
if [ ! -f "/tmp/.display_env_$HOST_UID" ]; then
    echo -e "\033[33m⚠ WARNING: Display environment file not found. Add bashrc configuration as described in README.md\033[0m"
fi
if [ ! -f "/tmp/.ssh_auth_sock_$HOST_UID" ]; then
    echo -e "\033[33m⚠ WARNING: SSH auth sock file not found. Add bashrc configuration as described in README.md\033[0m"
fi

# Build mount flags conditionally
MOUNT_FLAGS=""
if [ -f "${HOME}/.gitconfig" ]; then
    MOUNT_FLAGS="$MOUNT_FLAGS -v ${HOME}/.gitconfig:/root/.gitconfig:rw"
fi
if [ -f "${HOME}/.vimrc" ]; then
    MOUNT_FLAGS="$MOUNT_FLAGS -v ${HOME}/.vimrc:/root/.vimrc:rw"
fi

# Run the Docker container with sophisticated configuration
docker run -it \
    --name $CONTAINER_NAME \
    --hostname $HOSTNAME \
    --workdir /root \
    --group-add dialout \
    --privileged \
    --network host \
    --ipc host \
    --pid host \
    -e ROS_IP=$(ip -4 addr show dev wg0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}') \
    -e HOST_UID=$HOST_UID \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v /tmp/.display_env_$HOST_UID:/tmp/.display_env_$HOST_UID:ro \
    -v /tmp/.ssh_auth_sock_$HOST_UID:/tmp/.ssh_auth_sock_$HOST_UID:ro \
    -v ${HOME}/.Xauthority:/root/.Xauthority:rw \
    -v /dev/dri:/dev/dri:ro \
    -v /etc/passwd:/etc/passwd:ro \
    -v /etc/group:/etc/group:ro \
    --user 0:0 \
    $MOUNT_FLAGS \
    $FLAGS \
    $IMAGE_NAME