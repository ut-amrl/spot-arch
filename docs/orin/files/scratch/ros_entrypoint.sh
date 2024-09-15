#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

#rosdep update
#git lfs install 2>/dev/null

# Source the catkin workspace setup if it exists
if [[ -e /root/catkin_ws/devel/setup.bash ]]; then
	source /root/catkin_ws/devel/setup.bash
fi

# Repos
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/amrl_msgs
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/k4a_ros
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/spot_autonomy
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/graph_navigation
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/amrl_maps

# Add the Python path for the amrl_msgs package if it exists
if rospack find amrl_msgs &> /dev/null; then
	export PYTHONPATH="$(rospack find amrl_msgs)/src:${PYTHONPATH}"
fi

exec "$@"
