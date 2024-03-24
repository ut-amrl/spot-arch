# Sudo Steps
### Dual booting Ubuntu 20.04 with Existing ubuntu 20.04 (for alienware)
- Get a live bootable usb drive with Ubuntu 20.04 (use balena etcher)
- Partitioning will happen in the installation process
- Insert the bootable USB you've created and restart your computer. During the boot process, press F12 to access the boot menu and select the USB drive.
- Select the "Install Ubuntu" option from the menu.
- Follow the installation process. Do select 'Normal Install', 'Allow updates during install' and 'Install third-party software for graphics and Wi-Fi hardware and additional media formats' option.
- When you reach the "Installation type" screen, select the "Something else" option and click "Continue".
- Partitioning:
    - Follow steps [here](https://chat.openai.com/share/d2e58736-7ed1-45c2-a120-e3d218b7e8a4) <span style="color:red">(TODO: write partitioning steps)</span>
- Follow all prompts upto the end of the installation process. It will ask you to restart your computer. Do so and remove the USB drive when prompted.

### Setting up the fresh Ubuntu 20.04
#### Basic installs
- Run the following commands in the terminal:
    - `sudo apt update`
    - `sudo apt upgrade`
    - `sudo apt install linux-headers-$(uname -r) build-essential gcc g++ make`
    - `sudo apt install python3-dev python3-pip git vim emacs curl wget gdb cmake htop tmux screen nano mesa-utils ppa-purge`
    - `sudo pip3 install virtualenv`

#### Nvidia drivers ([reference](https://askubuntu.com/questions/206283/how-can-i-uninstall-a-nvidia-driver-completely))
- Remove the existing nvidia drivers CLEANLY and FULLY by running the following commands in the terminal:
    - `sudo apt-get remove --purge '^nvidia-.*'`
    - `sudo apt-get install ubuntu-desktop`
    - `sudo rm /etc/X11/xorg.conf`
    - `echo 'nouveau' | sudo tee -a /etc/modules`

- Installing the supported driver version:
    - Run `lspci | grep -i nvidia` on terminal to get your nvidia gpu model
    - Go to [nvidia's website](https://www.nvidia.com/download/index.aspx) and get the correct driver version for your gpu. For instance, it could be `535.154.05` for a Linux 64-bit machine running a GeForce RTX 2060 GPU.
    - **(needs physical access)** Go to 'Software & Updates' -> 'Additional Drivers' and select the 'Using NVIDIA driver metapackage from nvidia-driver-535 (proprietary)' option, if your version is `535.154.05`. Click 'Apply Changes' and restart your computer.
      	* If you do not see your required version listed there (e.g., `550`), do this: `sudo add-apt-repository ppa:graphics-drivers/ppa && sudo apt-get update`.
    - NOTE: after this, nvidia-smi should work and show the driver version. It may show CUDA runtime version as well.
        * <span style="color:gray; font-size:smaller;">ChatGPT: _"The output of nvidia-smi is showing a CUDA version because the NVIDIA driver comes with a bundled version of the CUDA runtime, which allows running CUDA applications. However, the CUDA toolkit, which includes the compilers, debugging tools, and other resources you need for CUDA development, is a separate installation. The CUDA runtime version shown in nvidia-smi indicates the highest CUDA version that is compatible with the installed NVIDIA driver. Even if you haven't explicitly installed the CUDA toolkit, the driver still supports running applications built with that CUDA version. If you want to use CUDA for development, you would need to install the CUDA toolkit separately, which typically includes the installation of the /usr/local/cuda directory. If you plan to work with CUDA, you can download the appropriate version of the toolkit from NVIDIA's website that matches the CUDA version shown in nvidia-smi."_</span>

#### CUDA Toolkit
- Go to [nvidia's website](https://developer.nvidia.com/cuda-toolkit-archive) and download the desired version of CUDA Toolkit for your system. For instance, you could do 11.6 for a Linux 64-bit machine running Ubuntu 20.04.
    - Use deb (local) method
- Reboot.
- Check nvidia-smi - you should see CUDA (and likely the driver version as well) version updated.

#### cuDNN installation ([reference](https://docs.nvidia.com/deeplearning/cudnn/installation/linux.html#package-manager-local-installation))
- Go to [nvidia's website](https://developer.nvidia.com/cudnn) and select the appropriate options for your system. Use deb (local) method. For instance, for Ubuntu 20.04 and CUDA 11.6, execute following commands:
    - `wget https://developer.download.nvidia.com/compute/cudnn/9.0.0/local_installers/cudnn-local-repo-ubuntu2004-9.0.0_1.0-1_amd64.deb`
    - `sudo dpkg -i cudnn-local-repo-ubuntu2004-9.0.0_1.0-1_amd64.deb`
    - `sudo cp /var/cudnn-local-repo-ubuntu2004-9.0.0/cudnn-*-keyring.gpg /usr/share/keyrings/`
    - `sudo apt-get update`
    - `sudo apt-get -y install cudnn9-cuda-11`
- Automating addition of appropriate cuda paths for all user accounts:
    - `sudo vi /etc/profile.d/misc.sh` and add the following lines:
        ```
        #!/bin/sh

        # To prevent running twice
        if [ -n "$_MISC_SH_LOADED" ]; then
            return 0
        fi
        _MISC_SH_LOADED=1

        # CUDA
        export CUDA_HOME=/usr/local/cuda
        export PATH=$PATH:$CUDA_HOME/bin
        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CUDA_HOME/lib64
        ```
    - `sudo chmod +x /etc/profile.d/misc.sh`
    - Enter sudo mode `sudo -i`.
    - `sudo echo 'if [ -f /etc/profile.d/misc.sh ]; then . /etc/profile.d/misc.sh; fi' >> /etc/bash.bashrc`
- Reboot.
- Test `nvidia-smi` and `nvcc -V` - you should see both versions matching.

#### Additional steps including quality of life improvements
- `sudo apt-get install libfreeimage3 libfreeimage-dev`
- `sudo apt autoremove && sudo apt autoclean && sudo apt clean`
- `sudo apt install net-tools`
- `sudo apt install acpi quota`
- `sudo apt-get install python3-tk`
- Install git-lfs:
    - `curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash`
    - `sudo apt-get install git-lfs`
- Setup libtorch:
    - Go to [pytorch's website](https://pytorch.org/get-started/locally/) and select appropriate options for your system. For instance, choose Linux, LibTorch, C++, and the CUDA version. Then download the `cxx11 ABI` version zip. Since, 11.6 is not available, this is the appropriate [link](https://download.pytorch.org/libtorch/cu116/libtorch-cxx11-abi-shared-with-deps-1.13.1%2Bcu116.zip) (from the archives). You can run the following commands in the terminal:
        - `wget https://download.pytorch.org/libtorch/cu116/libtorch-cxx11-abi-shared-with-deps-1.13.1%2Bcu116.zip`
        - `unzip libtorch-cxx11-abi-shared-with-deps-1.13.1+cu116.zip`
        - `sudo mv libtorch /opt/`
- `sudo apt update && sudo apt upgrade`
- `sudo apt autoremove && sudo apt autoclean && sudo apt clean`
- Setup SSH server:
    - `sudo apt install openssh-server -y`
    - `sudo systemctl status ssh` to check if it's running. If not, run `sudo systemctl start ssh` to start it.
    - `sudo systemctl enable ssh` to enable it on startup.
    - Security settings for ssh:
        - `sudo vi /etc/ssh/sshd_config` and change/uncomment the following lines:
            ```
            PasswordAuthentication no
            PubkeyAuthentication yes
            PermitRootLogin prohibit-password
            PermitEmptyPasswords no
            ```
        - `sudo systemctl restart ssh`
- Setup AMRL wireguard vpn ([reference](https://github.com/ut-amrl/amrl-documentation/blob/master/computers/wireguard-vpn.md))
    - `sudo apt-get install wireguard wireguard-tools`
    - Enter sudo mode (i.e., executing commands as root user): `sudo -i`
    - `cd /etc/wireguard`
    - `wg genkey | tee wg-private.key | wg pubkey > wg-public.key`
    - Create `/etc/wireguard/wg0.conf` file and add the contents as shown in the reference link.
    - `sudo wg-quick up wg0`
    - `sudo systemctl enable wg-quick@wg0`
    - Reboot.
    - `sudo systemctl status wg-quick@wg0` to check if it's running.
    - Get it registered with the [AMRL server](https://github.com/ut-amrl/amrl-documentation/blob/master/computers/wireguard-vpn.md#adding-clients-to-server).
- Setup power settings so that laptop won't turn off:
    - `sudo vi /etc/systemd/logind.conf` and change the following lines:
        ```
        HandleLidSwitch=ignore
        HandleLidSwitchExternalPower=ignore
        HandleLidSwitchDocked=ignore
        ```
    - `sudo systemctl restart systemd-logind`
    - `sudo vi /etc/profile.d/disable-blank-screen.sh` and add the following lines:
        ```
        #!/bin/sh
        gsettings set org.gnome.desktop.session idle-delay 0
        gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'
        gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing'
        ```
    - `sudo chmod +x /etc/profile.d/disable-blank-screen.sh`
- Configure network interfaces for the sensors: Assign static IPs to the ethernet interfaces for convenience. An example file is provided [here](examples/02-spotgxp-and-velodyne.yaml). Place it in `/etc/netplan` with correct permissions (`644`). Then execute the following commands:
    - `sudo systemctl start systemd-networkd`
    - `sudo systemctl enable systemd-networkd`
    - `sudo netplan apply`
- Refer [here](https://www.cyberciti.biz/howto/quick-tip-display-banner-message-before-openssh-authentication/) if want to add a custom ssh banner message for giving some important information.
- To be able to set user quotas:
    - `sudo vi /etc/fstab` and add `usrquota` to the root partition
        - for instance, change `UUID=401615fc-572a-4c30-9c0d-d62dd13db87d /               ext4    errors=remount-ro 0       1` to `UUID=401615fc-572a-4c30-9c0d-d62dd13db87d /               ext4    errors=remount-ro,usrquota 0       1`
    - `sudo reboot`
    - `sudo quotacheck -cugm /` to initialize the quota files
    - `sudo quotaon -v /` to turn on the quotas
    - `sudo reboot`
- Installing a shared anaconda3 to preserve storage (root access to modify base env, users can create their own envs):
    - `sudo apt-get install libgl1-mesa-glx libegl1-mesa libxrandr2 libxrandr2 libxss1 libxcursor1 libxcomposite1 libasound2 libxi6 libxtst6`
    - `curl -O https://repo.anaconda.com/archive/Anaconda3-2022.10-Linux-x86_64.sh`
    - Enter sudo mode (i.e., executing commands as root user): `sudo -i`
    - `bash Anaconda3-2022.10-Linux-x86_64.sh`
        - Install to `/opt/anaconda3`
        - Do not initialize conda in `.bashrc`, i.e., do not accept `conda init` option.
    - `sudo chown -R root:root /opt/anaconda3`
    - `sudo chmod -R 755 /opt/anaconda3`
    - Add conda initialization (required for conda to be accessible in `sudo -i` mode):
        - `sudo vi /etc/profile.d/misc.sh` and append the following lines:
            ```
            # Anaconda3
            # >>> conda initialize >>>
            # !! Contents within this block are managed by 'conda init' !!
            __conda_setup="$('/opt/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
            if [ $? -eq 0 ]; then
                eval "$__conda_setup"
            else
                if [ -f "/opt/anaconda3/etc/profile.d/conda.sh" ]; then
                    . "/opt/anaconda3/etc/profile.d/conda.sh"
                else
                    export PATH="/opt/anaconda3/bin:$PATH"
                fi
            fi
            unset __conda_setup
            # <<< conda initialize <<<
            ```
        - Reboot.
    - This will place your anaconda3 bin path before the system python path in your `$PATH` variable. So any compile or build commands will use the anaconda3 python by default, even if you aren't explicitly in the `base` conda environment. So, install the needed packages in the `base` environment.
        - Enter sudo mode (i.e., executing commands as root user): `sudo -i`
        - `conda activate`. Now any pip installs will place packages in the shared `base` environment. Else, it would have put it in `home/user/.local/bin` directory. And doing `sudo pip3 install` would have put it in system-wide python installation directory.
        - `pip install rospkg empy==3.3.4`
        - `pip install virtualenv protobuf==3.20.1 cython bosdyn-client==3.1.1 bosdyn-mission==3.1.1 bosdyn-api==3.1.1 bosdyn-core==3.1.1` - some additional things needed for other stuff.
        - For conda bash completion, do `conda install -c conda-forge conda-bash-completion`. 
        - Reboot.

#### ROS Noetic installation ([reference](http://wiki.ros.org/noetic/Installation/Ubuntu))
- Setup your sources.list:
    - `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
- Set up your keys:
    - `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
- Installation:
    - `sudo apt update`
    - `sudo apt install ros-noetic-desktop-full`
- Environment setup:
    - Add `source /opt/ros/noetic/setup.bash` at the start of your `/etc/profile.d/misc.sh` file.
- Dependencies for building packages:
    - `sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential`
    - `sudo rosdep init`
    - `rosdep update`
- `sudo apt-get install python-is-python3`
- `sudo apt-get install ros-noetic-ros-numpy`
- `sudo pip3 install rospkg`

#### Some more installs to support AMRL repositories
- `sudo apt-get update && sudo apt-get install -y apt-utils curl python-is-python3 python3-catkin-tools python3-pip software-properties-common`
- `sudo apt-get update && sudo apt-get install -y libgtest-dev libgoogle-glog-dev cmake build-essential` (https://github.com/ut-amrl/amrl_shared_lib)
- `sudo apt-get update && sudo apt-get install -y libgoogle-glog-dev libgflags-dev liblua5.1-0-dev` (https://github.com/ut-amrl/graph_navigation)
- `sudo apt-get update && sudo apt-get install -y liblua5.1-dev libeigen3-dev libjpeg8-dev libgoogle-perftools-dev libsuitesparse-dev libblas-dev liblapack-dev libopenmpi-dev libgoogle-glog-dev libgflags-dev libceres-dev libtbb-dev libncurses5-dev libpopt-dev` (https://github.com/ut-amrl/enml)
- `sudo apt-get update && sudo apt-get install -y cmake qt5-default libqt5websockets5-dev` (https://github.com/ut-amrl/webviz and https://github.com/ut-amrl/robofleet_client)
- `sudo apt-get update && sudo apt-get install -y build-essential clang-12 clang-format cmake g++ gdb git nano valgrind vim`
- `sudo apt-get update && sudo apt-get install -y iputils-ping less mesa-utils net-tools rsync tmux tree unzip usbutils zip zsh`
- (https://github.com/ut-amrl/k4a_ros)
    - `sudo apt-get update && sudo apt-get install -y liblua5.1-0-dev libgflags-dev libgoogle-glog-dev libgoogle-perftools-dev cimg-dev`
    - `curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -`
    - `sudo apt-add-repository "deb https://packages.microsoft.com/ubuntu/18.04/prod bionic main"`
    - `sudo apt-get update && sudo sh -c 'ACCEPT_EULA=Y apt-get install -y k4a-tools libk4a1.4 libk4a1.4-dev'`
    - `wget https://raw.githubusercontent.com/ut-amrl/k4a_ros/master/99-k4a.rules` and add `99-k4a.rules` file to `/etc/udev/rules.d/` to allow access to the kinect device and reboot to apply the changes. Alternatively, `sudo udevadm control --reload-rules && sudo udevadm trigger` to apply the changes without rebooting.
- `sudo apt-get update && sudo apt-get install -y qt5-default libqt5websockets5-dev libgoogle-glog-dev libgflags-dev` (https://github.com/ut-amrl/spot_autonomy)
- (https://github.com/ut-amrl/spot_ros)
    - `sudo apt-get update && sudo apt-get install -y ros-noetic-twist-mux ros-noetic-interactive-marker-twist-server ros-noetic-velodyne-pointcloud`
    - `sudo pip3 install protobuf==3.20.1` for system-wide protobuf installation.
    - `sudo pip3 install cython empy bosdyn-client==3.1.1 bosdyn-mission==3.1.1 bosdyn-api==3.1.1 bosdyn-core==3.1.1` for system-wide bosdyn installation.
- `sudo apt-get update && sudo apt-get install -y alsa-base alsa-utils pulseaudio joystick ffmpeg espeak mpg123 pavucontrol`

<!-- <span style="color:red">(This has to be done for each user account)</span> -->
# User Account Steps
- From a sudo account, add a new user account: `sudo adduser <username>` and follow the prompts. Once created:
    - Run `sudo -u <username> xdg-user-dirs-update` to create the default directories for the new user.
    - `sudo -u <username> mkdir /home/<username>/.ssh`
    - `sudo -u <username> chmod 700 /home/<username>/.ssh`
    - `sudo -u <username> touch /home/<username>/.ssh/authorized_keys`
    - Add ssh key(s) to the `authorized_keys` file.
    - `sudo setquota -u <username> 41943040 52428800 0 0 /` to set the limits for the user (for instance, 40 GB soft limit and 50 GB hard limit).
    - Give sudo permissions to the user if needed: `sudo usermod -aG sudo <username>`.
    - `sudo chown <username>:<username> /home/<username>/.ssh && sudo chown <username>:<username> /home/<username>/.ssh/*`
- Login into the user account.
- Run `git lfs install` to initialize git-lfs for the user account.
- `rosdep update` to update the rosdep database.    
- `vi ~/.vimrc` and add the following lines:
    ```
    set number
    set tabstop=4
    set shiftwidth=4
    set expandtab
    set autoindent
    ```
- `vi ~/.gitconfig` and add the following lines if not there already:
    ```
    [user]
    	name = AMRL User
    	email = amrl_user@gmail.com
    [credential]
    	helper = store
    [remote "origin"]
    	prune = true
    [filter "lfs"]
    	clean = git-lfs clean -- %f
    	smudge = git-lfs smudge -- %f
    	process = git-lfs filter-process
    	required = true
    ```
- Create a catkin workspace:
    - `mkdir -p ~/catkin_ws/src`
    - `cd ~/catkin_ws/`
    - `catkin_make`
    - `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`
- Add the following lines to `~/.bashrc`:
    ```
    # Anaconda3
    # >>> conda initialize >>>
    # !! Contents within this block are managed by 'conda init' !!
    __conda_setup="$('/opt/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
    if [ $? -eq 0 ]; then
        eval "$__conda_setup"
    else
        if [ -f "/opt/anaconda3/etc/profile.d/conda.sh" ]; then
            . "/opt/anaconda3/etc/profile.d/conda.sh"
        else
            export PATH="/opt/anaconda3/bin:$PATH"
        fi
    fi
    unset __conda_setup
    # <<< conda initialize <<<

    # Conda bash completion
    CONDA_ROOT=/opt/anaconda3   # <- set to your Anaconda/Miniconda installation directory
    if [[ -r $CONDA_ROOT/etc/profile.d/bash_completion.sh ]]; then
        source $CONDA_ROOT/etc/profile.d/bash_completion.sh
    else
        echo "WARNING: could not find conda-bash-completion setup script"
    fi
    ```
- `vi ~/.condarc` and add the following lines:
    ```
    auto_activate_base: false
    ```
- Reboot.
- <span style="color:gray">[INFO] After this, the behaviour of conda is as follows:
    - In normal user shell:
        - conda completion should be working
        - default python is system python
        - conda activate will access the system-wide base conda environment
        - conda deactivate will deactivate the conda environment and revert to system python
    - In `sudo -i` shell:
        - conda completion should be working
        - default python is conda base python (even if you aren't explicitly in the `base` conda environment)
        - conda deactivate will revert to system python. You can do `conda activate` to go back to conda base python.
    - _**Since all packages were also installed in the system-wide `base` conda environment, you can use the `base` environment for building and compiling, including `catkin_make`.**_
</span>

### Setup AMRL repositories
- Clone the following repositories in the `~/catkin_ws/src` directory:
    - `cd ~/catkin_ws/src`
    - Vectornav:
        - `git clone git@github.com:dawonn/vectornav.git --recursive`
        - `vi vectornav/params/vn200.yaml` and change the `imu_output_rate` to 200 Hz.
    - Spot ROS:
        - `git clone git@github.com:ut-amrl/spot_ros.git --recursive`
    - `cd ~/catkin_ws`
    - `catkin_make; catkin_make` (since it sometimes fails to make it properly the first time, possibly some inter-dependency issues)
- Clone the following repositories in the `~/ut-amrl` directory:
    - `mkdir ~/ut-amrl`
    - **AMRL msgs**:
        - `cd ~/ut-amrl`
        - `git clone git@github.com:ut-amrl/amrl_msgs.git --recursive`
        - Add the following line to `~/.bashrc`:
            ```
            export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ut-amrl/amrl_msgs
            ```
        - `cd ~/ut-amrl/amrl_msgs`
        - `source ~/.bashrc && make -j$(nproc)`
    - **K4A ROS**:
        - `cd ~/ut-amrl`
        - `git clone git@github.com:ut-amrl/k4a_ros.git --recursive`
        - Add the following line to `~/.bashrc`:
            ```
            export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ut-amrl/k4a_ros
            ```
        - `cd ~/ut-amrl/k4a_ros`
        - `source ~/.bashrc && make -j$(nproc)`
    - **Spot Autonomy**:
        - `cd ~/ut-amrl`
        - `git clone git@github.com:ut-amrl/spot_autonomy.git --recursive`
        - Create symlinks for convenience:
            - `ln -s ~/ut-amrl/spot_autonomy/graph_navigation ~/ut-amrl/graph_navigation`
            - `ln -s ~/ut-amrl/spot_autonomy/webviz ~/ut-amrl/webviz`
            - `ln -s ~/ut-amrl/spot_autonomy/maps ~/ut-amrl/amrl_maps`
        - Add the following lines to `~/.bashrc`:
            ```
            export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ut-amrl/spot_autonomy
            export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ut-amrl/graph_navigation
            export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ut-amrl/amrl_maps
            ```
        - `cd ~/ut-amrl/spot_autonomy`
        - `source ~/.bashrc && make -j$(nproc)`
            - _If you get an error saying `undefined reference to `uuid_generate@UUID_1.0'`, then do this [reference](https://github.com/uzh-rpg/rpg_esim/issues/7):_
                - `sudo apt-get install uuid-dev`
                - `sudo mkdir /opt/anaconda3/libuuid`
                - `sudo mv /opt/anaconda3/lib/libuuid* /opt/anaconda3/libuuid/`
        - `cp launch/start_clearpath_spot.launch.example launch/start_clearpath_spot.launch`
        - Add the BD Spot login credentials to the `start_clearpath_spot.launch` file.

# Good practices and Tips
- Always use ssh-agent forwarding for shared accounts. DO NOT add your keys to the shared account.
- Example files for the `.bashrc`, `.vimrc`, `.gitconfig`, `/etc/profile.d/misc.sh`, `/etc/profile.d/disable-blank-screen.sh`, `/etc/ssh/custom_info` and `.condarc` files are provided [here](examples/).
- You might need to add a rules file like [this](examples/99-ttyusb.rules) to `/etc/udev/rules.d/` to allow access to the usb ports (e.g., without that you might see an error when running vectornav).
- Creating private user and shared anaconda3 environments ([reference](https://stackoverflow.com/questions/37926940/how-to-specify-new-environment-location-for-conda-create) and `conda create --help`):
    - User (private) environment:
        - By default, all environments are created in `~/.conda/envs/` directory, i.e., prefix is `~/.conda/envs/env_name`.
    - Shared environment:
        - Use the prefix option when doing `conda create --prefix /opt/anaconda3/envs/env_name` with full path to the environment.
    - Note, there are some subtleties with if conda can detect the custom location's environment or not. If not, you can use `conda activate /opt/anaconda3/envs/env_name` to activate the environment.
- Turning on and off ssh server and wireguard vpn client properly:
    - SSH sever:
        - Turn on: `sudo systemctl start ssh && sudo systemctl enable ssh` and then Reboot.
        - Turn off: `sudo systemctl stop ssh && sudo systemctl disable ssh` and then Reboot.
    - Wireguard vpn client:
        - Turn on: `sudo wg-quick up wg0 && sudo systemctl enable wg-quick@wg0` and then Reboot.
        - Turn off: `sudo wg-quick down wg0 && sudo systemctl stop wg-quick@wg0 && sudo systemctl disable wg-quick@wg0` and then Reboot.
- Secure `.ssh` folder on a shared account:
    - `cd` to the home directory of the shared user account
    - Ensure only sudo (root) can add/delete files in .ssh:
        - `sudo chown root:root .ssh`
        - `sudo chmod 755 .ssh`
    - Set authorized_keys readable by shared user account, writable only by sudo
        - `sudo chown root:root .ssh/authorized_keys`
        - `sudo chmod 644 .ssh/authorized_keys`
    - Set known_hosts to have read/write access for everyone
        - `sudo chown root:root .ssh/known_hosts`
        - `sudo chmod 666 .ssh/known_hosts`
- User quotas are assigned in terms of number of blocks. The block size is `1024 bytes`. For instance, to set a 40 GB soft limit and a 50 GB hard limit for a user:
    - `sudo setquota -u <username> 41943040 52428800 0 0 /` to set the limits for a user.
    (For group quotas to work, you can set the group ownership of the files to the shared group and then set the quotas for the group)
