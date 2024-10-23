#!/bin/bash
set -e

# Dynamically set SSH_AUTH_SOCK if it's available in the mounted /tmp directory
if [ -n "$(find /tmp -type s -name 'agent.*' 2>/dev/null)" ]; then
  export SSH_AUTH_SOCK=$(find /tmp -type s -name 'agent.*' 2>/dev/null)
fi

export DISPLAY=$(cat /tmp/.display_env_$HOST_UID)

# Check if the initialization has already been done (using a marker file)
if [ ! -f /initialized ]; then
	echo "Running one-time setup for essential repos..."
	source "/opt/ros/$ROS_DISTRO/setup.bash"
	# Clone and set up repositories
	mkdir -p /root/catkin_ws/src
	cd /root/catkin_ws/src

	# Clone spot_ros repo
	git clone --recursive --branch ldos_sim git@github.com:ut-amrl/spot_ros.git

	# Build the catkin workspace
	cd /root/catkin_ws
	catkin_make || true
	catkin_make || true

	# Source the catkin workspace setup
	source /root/catkin_ws/devel/setup.bash

	# Set up ~/ut-amrl directory and clone the other repositories
	mkdir -p /root/ut-amrl
	cd /root/ut-amrl

	git clone --recursive git@github.com:ut-amrl/ldos_sim.git

	# Create symbolic links
	ln -s /root/ut-amrl/ldos_sim/third_party/amrl_maps /root/ut-amrl/amrl_maps
	ln -s /root/ut-amrl/ldos_sim/third_party/amrl_msgs /root/ut-amrl/amrl_msgs
	ln -s /root/ut-amrl/ldos_sim/third_party/graph_navigation /root/ut-amrl/graph_navigation
	ln -s /root/ut-amrl/ldos_sim/third_party/ut_automata /root/ut-amrl/ut_automata
	ln -s /root/ut-amrl/ldos_sim/third_party/webviz /root/ut-amrl/webviz

	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/ldos_sim
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/amrl_maps
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/amrl_msgs
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/graph_navigation
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/ut_automata
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/webviz

	cd /root/ut-amrl/amrl_msgs
	make -j$(nproc) || true
	cd /root/ut-amrl/graph_navigation
	make -j$(nproc) || true
	cd /root/ut-amrl/ut_automata
	make -j$(nproc) || true
	cd /root/ut-amrl/webviz
	make -j$(nproc) || true

	# Add the Python path for the amrl_msgs package if it exists
	if rospack find amrl_msgs &> /dev/null; then
		export PYTHONPATH="$(rospack find amrl_msgs)/src:${PYTHONPATH}"
	fi

	# Create the marker file to indicate that initialization has been done
	touch /initialized
	cd /root

	# conda hooks
	mkdir -p /root/.conda/hooks/
	touch /root/.conda/hooks/post-create.sh
	cat >> /root/.conda/hooks/post-create.sh <<- "END"
	#!/bin/bash

	# Get the path of the newly created environment
	env_path=$CONDA_PREFIX

	# Create the activate.d and deactivate.d directories
	mkdir -p $env_path/etc/conda/activate.d
	mkdir -p $env_path/etc/conda/deactivate.d

	# Add activation script to set up LD_LIBRARY_PATH and Torch_DIR
	cat <<EOL > $env_path/etc/conda/activate.d/env_vars.sh
	export LD_LIBRARY_PATH=\$CONDA_PREFIX/lib/python3.8/site-packages/torch:\$LD_LIBRARY_PATH
	export LD_LIBRARY_PATH=\$CONDA_PREFIX/lib/python3.8/site-packages/torch/lib:\$LD_LIBRARY_PATH
	export Torch_DIR=\$CONDA_PREFIX/lib/python*/site-packages/torch/share/cmake/Torch
	EOL

	# Add deactivation script to restore the system-wide paths
	cat <<EOL > $env_path/etc/conda/deactivate.d/env_vars.sh
	export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH/\$CONDA_PREFIX\/lib\/python3.8\/site-packages\/torch:/}
	export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH/\$CONDA_PREFIX\/lib\/python3.8\/site-packages\/torch\/lib:/}
	export Torch_DIR=/usr/local/lib/python3.8/dist-packages/torch/share/cmake/Torch
	EOL
	END
	chmod +x /root/.conda/hooks/post-create.sh

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
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/ldos_sim
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/amrl_maps
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/amrl_msgs
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/graph_navigation
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/ut_automata
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/webviz

	# Add the Python path for the amrl_msgs package if it exists
	if rospack find amrl_msgs &> /dev/null; then
		export PYTHONPATH="$(rospack find amrl_msgs)/src:${PYTHONPATH}"
	fi

	conda_create_hook() {
		# Create the conda environment
		conda create "$@"
		
		# Extract the environment name from the arguments (assumes --name or -n is used)
		env_name=""
		for i in "$@"; do
			if [[ $i == "--name" || $i == "-n" ]]; then
				shift
				env_name="$1"
				break
			fi
			shift
		done
		
		conda activate "$env_name"
		bash ~/.conda/hooks/post-create.sh
		conda deactivate
	}

	alias conda-create="conda_create_hook"

	gitall() {
		if [ -z "$1" ]; then
			git add . && git commit -m "changes"
		else
			git add . && git commit -m "$1"
		fi
	}
	alias expros='export ROS_PACKAGE_PATH=$(pwd):$ROS_PACKAGE_PATH'
	alias listpycache='find . -type d -name "__pycache__" -print'
	alias cleanpycache='listpycache 2>/dev/null | xargs rm -rf 2>/dev/null'

	END
else
	: # do nothing
fi

exec "$@"
