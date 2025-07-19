#!/bin/bash
set -e

# Configure dynamic environment variables from host files
SSH_AUTH_SOCK_FILE="/tmp/.ssh_auth_sock_$HOST_UID"
DISPLAY_ENV_FILE="/tmp/.display_env_$HOST_UID"

# Set SSH_AUTH_SOCK from host file (for SSH agent forwarding)
if [ -f "$SSH_AUTH_SOCK_FILE" ]; then
    export SSH_AUTH_SOCK=$(cat "$SSH_AUTH_SOCK_FILE")
    echo "SSH_AUTH_SOCK set to: $SSH_AUTH_SOCK"
else
    echo "WARNING: SSH_AUTH_SOCK file not found. SSH agent forwarding may not work."
fi

# Set DISPLAY from host file (for X11 forwarding)
if [ -f "$DISPLAY_ENV_FILE" ]; then
    export DISPLAY=$(cat "$DISPLAY_ENV_FILE")
    echo "DISPLAY set to: $DISPLAY"
else
    echo "WARNING: DISPLAY file not found. GUI applications may not work."
fi

# Check if the initialization has already been done (using a marker file)
if [ ! -f /initialized ]; then
	echo -e "\033[36m🔄 Running one-time setup...\033[0m"
	
	# weird pip install and cache bugfix (dec16 2024)
	for file in /etc/xdg/pip/pip.conf /etc/pip.conf /usr/pip.conf /root/.config/pip/pip.conf /root/.pip/pip.conf; do [ -f "$file" ] && sed -i 's/^\s*\(no-cache-dir\s*=.*\)/# \1/' "$file" || true; done
	
	source "/opt/ros/$ROS_DISTRO/setup.bash"

	# Setup cobot-autonomy
	mkdir -p /root/ut-amrl
	cd /root/ut-amrl
	git clone --recursive https://github.com/ut-amrl/cobot_autonomy.git
	# TODO: complete this
	cd /root

	# Create the marker file to indicate that initialization has been done
	touch /initialized
	cd /root

	# Configure .bashrc with ROS, conda, and development environment
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

	# Development environment configuration
	export PATH=$PATH:/usr/local/go/bin
	
	# Environment variables - hardcoded defaults
	export TF_CPP_MIN_LOG_LEVEL='2'
	export PYTHONWARNINGS="ignore"
	export PYTHONNOUSERSITE=1  # so that in conda, it will not use .local packages

	# Custom git and development aliases
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
	alias sudo=''

	END
else
	: # Initialization already completed, do nothing
fi

exec "$@"