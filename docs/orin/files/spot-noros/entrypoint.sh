#!/bin/bash
set -e

# Dynamically set SSH_AUTH_SOCK and DISPLAY
export SSH_AUTH_SOCK=$(cat /tmp/.ssh_auth_sock_$HOST_UID)
export DISPLAY=$(cat /tmp/.display_env_$HOST_UID)

# Check if the initialization has already been done (using a marker file)
if [ ! -f /initialized ]; then
	echo "Running one-time setup for essential tools..."

	# weird pip install and cache bugfix (dec16 2024)
	for file in /etc/xdg/pip/pip.conf /etc/pip.conf /usr/pip.conf /root/.config/pip/pip.conf /root/.pip/pip.conf; do [ -f "$file" ] && sed -i 's/^\s*\(no-cache-dir\s*=.*\)/# \1/' "$file" || true; done
	
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
	python_version=$(python -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")

	# Create the activate.d and deactivate.d directories
	mkdir -p $env_path/etc/conda/activate.d
	mkdir -p $env_path/etc/conda/deactivate.d

	# Add activation script to set up LD_LIBRARY_PATH and Torch_DIR
	cat <<EOL > $env_path/etc/conda/activate.d/env_vars.sh
	export LD_LIBRARY_PATH=\$CONDA_PREFIX/lib/python${python_version}/site-packages/torch:\$LD_LIBRARY_PATH
	export LD_LIBRARY_PATH=\$CONDA_PREFIX/lib/python${python_version}/site-packages/torch/lib:\$LD_LIBRARY_PATH
	export Torch_DIR=\$CONDA_PREFIX/lib/python${python_version}/site-packages/torch/share/cmake/Torch
	EOL

	# Add deactivation script to restore the system-wide paths
	cat <<EOL > $env_path/etc/conda/deactivate.d/env_vars.sh
	export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH/\$CONDA_PREFIX\/lib\/python${python_version}\/site-packages\/torch:/}
	export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH/\$CONDA_PREFIX\/lib\/python${python_version}\/site-packages\/torch\/lib:/}
	export Torch_DIR=/usr/local/lib/python${python_version}/dist-packages/torch/share/cmake/Torch
	EOL
	END
	chmod +x /root/.conda/hooks/post-create.sh

	# add to .bashrc
	cat >> /root/.bashrc <<- "END"
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

	gitall() {
		if [ -z "$1" ]; then
			git add . && git commit -m "changes"
		else
			git add . && git commit -m "$1"
		fi
	}

	alias listpycache='find . -type d -name "__pycache__" -print'
	alias cleanpycache='listpycache 2>/dev/null | xargs rm -rf 2>/dev/null'

	alias conda-create="conda_create_hook"
	alias sudo=''

	END
else
	: # do nothing
fi

exec "$@"