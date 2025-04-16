# Usage: ./container.sh <image-name> <container-name> [--force] [--other-flags]
#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Initialize variables for options and image name
if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <image-name> <container-name> [--force] [--other-flags]"
  exit 1
fi

IMAGE_NAME="$1"
CONTAINER_NAME="$2"
FORCE_DELETE="0"
FLAGS=""

shift 2 # Move past required args

while [[ $# -gt 0 ]]; do
  case $1 in
    --force)
      FORCE_DELETE=1
      shift
      ;;
    --*)
      FLAGS="$FLAGS $1"
      shift
      ;;
    *)
      echo "Unknown argument: $1"
      exit 1
      ;;
  esac
done

echo "Using image: $IMAGE_NAME"
echo "Using container name: $CONTAINER_NAME"

if [[ "$FORCE_DELETE" == "1" ]]; then
  yes | docker rm --force $CONTAINER_NAME
fi

echo "Additional flags: $FLAGS"

# Run the Docker container with the provided or default image name and flags
set -x
docker run -d \
    -e PATH="/opt/nvidia/nsight-systems/2024.2.2/bin:$PATH" \
    --name $CONTAINER_NAME \
    --hostname orin \
    --runtime nvidia \
    --gpus all \
    --network host \
    --ipc host \
    --cgroupns host \
    --workdir /root \
    --group-add dialout \
    --privileged \
    -e PULSE_SERVER=unix:/run/user/0/pulse/native \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    -e ROS_MASTER_URI=http://10.1.0.3:11311 \
    -e ROS_IP=10.1.0.3 \
    -e HOST_UID=${HOST_UID} \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v ${HOME}/.Xauthority:/root/.Xauthority:rw \
    -v ${HOME}/.ssh:/root/.ssh:rw \
    -v ${HOME}/robot:/root/robot:rw \
    -v /dev/dri:/dev/dri:ro \
    -v /sys/fs/cgroup:/sys/fs/cgroup:rw \
    -v /run/user/${HOST_UID}/pulse/native:/run/user/0/pulse/native:rw \
    -v ${HOME}/.gitconfig:/root/.gitconfig:rw \
    -v /tmp:/tmp \
    -v /opt/nvidia/nsight-systems/:/opt/nvidia/nsight-systems/ \
    -v /usr/local/cuda/bin/ncu:/usr/local/bin/ncu \
    -u 0:0 \
    $FLAGS \
    $IMAGE_NAME \
    sleep infinity