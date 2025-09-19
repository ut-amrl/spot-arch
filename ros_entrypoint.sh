#!/bin/bash
set -e

# Check if the initialization has already been done (using a marker file)
if [ ! -f /initialized ]; then
	echo "Running one-time setup for essential repos..."
	source "/opt/ros/$ROS_DISTRO/setup.bash"
	# # Clone and set up repositories
	# TODO: add stuff

	# Create the marker file to indicate that initialization has been done
	touch /initialized
	cd /root

	# add to .bashrc
	cat >> /root/.bashrc <<- "END"
	source "/opt/ros/$ROS_DISTRO/setup.bash"

	# # Source the catkin workspace setup if it exists
	# if [[ -e /root/catkin_ws/devel/setup.bash ]]; then
	# 	source /root/catkin_ws/devel/setup.bash
	# fi

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
	alias sudo=''

	END
else
	: # do nothing
fi

exec "$@"
