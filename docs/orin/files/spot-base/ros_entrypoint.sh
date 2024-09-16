#!/bin/bash
set -e

# Dynamically set SSH_AUTH_SOCK if it's available in the mounted /tmp directory
if [ -n "$(find /tmp -type s -name 'agent.*' 2>/dev/null)" ]; then
  export SSH_AUTH_SOCK=$(find /tmp -type s -name 'agent.*' 2>/dev/null)
fi

# Check if the initialization has already been done (using a marker file)
if [ ! -f /initialized ]; then
	echo "Running one-time setup for essential repos..."
	source "/opt/ros/$ROS_DISTRO/setup.bash"
	# Clone and set up repositories
	mkdir -p /root/catkin_ws/src
	cd /root/catkin_ws/src

	# Clone vectornav and set imu_output_rate in params file
	git clone git@github.com:dawonn/vectornav.git --recursive
	sed -i 's/imu_output_rate:.*/imu_output_rate: 200/' /root/catkin_ws/src/vectornav/params/vn200.yaml

	# Clone spot_ros repo
	git clone git@github.com:ut-amrl/spot_ros.git --recursive

	# Build the catkin workspace
	cd /root/catkin_ws
	catkin_make || true
	catkin_make || true

	# Source the catkin workspace setup
	source /root/catkin_ws/devel/setup.bash

	# Set up ~/ut-amrl directory and clone the other repositories
	mkdir -p /root/ut-amrl
	cd /root/ut-amrl

	git clone git@github.com:ut-amrl/amrl_msgs.git --recursive
	git clone git@github.com:ut-amrl/k4a_ros.git --recursive
	git clone git@github.com:ut-amrl/spot_autonomy.git --recursive

	# Create symbolic links
	ln -s /root/ut-amrl/spot_autonomy/graph_navigation /root/ut-amrl/graph_navigation
	ln -s /root/ut-amrl/spot_autonomy/webviz /root/ut-amrl/webviz
	ln -s /root/ut-amrl/spot_autonomy/maps /root/ut-amrl/amrl_maps

	# Copy and rename the launch file
	cp /root/ut-amrl/spot_autonomy/launch/start_clearpath_spot.launch.example /root/ut-amrl/spot_autonomy/launch/start_clearpath_spot.launch

	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/amrl_msgs
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/k4a_ros
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/spot_autonomy
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/graph_navigation
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/amrl_maps

	# Build the amrl_msgs, k4a_ros, and spot_autonomy projects in order
	cd /root/ut-amrl/amrl_msgs
	make -j$(nproc) || true

	cd /root/ut-amrl/k4a_ros
	make -j$(nproc) || true

	cd /root/ut-amrl/spot_autonomy
	make -j$(nproc) || true

	# Add the Python path for the amrl_msgs package if it exists
	if rospack find amrl_msgs &> /dev/null; then
		export PYTHONPATH="$(rospack find amrl_msgs)/src:${PYTHONPATH}"
	fi

	# Create the marker file to indicate that initialization has been done
	touch /initialized
	cd /root

	# add to .bashrc
	cat >> /root/.bashrc <<- "END"
	source "/opt/ros/$ROS_DISTRO/setup.bash"

	# Source the catkin workspace setup if it exists
	if [[ -e /root/catkin_ws/devel/setup.bash ]]; then
		source /root/catkin_ws/devel/setup.bash
	fi

	# >>> conda initialize >>>
	# !! Contents within this block are managed by 'conda init' !!
	__conda_setup="$('/opt/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
	if [ $? -eq 0 ]; then
		eval "$__conda_setup"
	else
		if [ -f "/opt/miniconda3/etc/profile.d/conda.sh" ]; then
			. "/opt/miniconda3/etc/profile.d/conda.sh"
		else
			export PATH="/opt/miniconda3/bin:$PATH"
		fi
	fi
	unset __conda_setup
	# <<< conda initialize <<<

	# Conda bash completion
	CONDA_ROOT=/opt/miniconda3
	if [[ -r $CONDA_ROOT/etc/profile.d/bash_completion.sh ]]; then
		source $CONDA_ROOT/etc/profile.d/bash_completion.sh
	else
		echo "WARNING: could not find conda-bash-completion setup script"
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

	END
else
	: # do nothing
fi

exec "$@"
