# Flashing Ubuntu 22.04 on ROG NUC 970
- Get a live bootable usb drive with Ubuntu 22.04 (use balena etcher)
- Partitioning will happen in the installation process
- Insert the bootable USB you've created and reboot. During the boot process, press F10 to access the boot menu and select the USB drive.
- Select the "Install Ubuntu" option from the menu.
- Follow the installation process and onscreen prompts. Do select 'Normal Install', 'Allow updates during install' and 'Install third-party software for graphics and Wi-Fi hardware and additional media formats' option. Enter a password for secure boot.
- When you reach the "Installation type" screen, select the erase option, as you do not need dual boot.
- Follow all prompts upto the end of the installation process. It will ask you to restart your computer. Do so and remove the USB drive when prompted.
- During restart, it will prompt you for MOK key. Select that and enter your password when prompted. It will ask you to reboot again.

# Post-setup
## basic packages
- sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean
- sudo apt install linux-headers-$(uname -r) alsa-base alsa-utils apt-utils build-essential clang-12 clang-format cmake curl emacs espeak ffmpeg firefox g++ gcc gdb git htop iputils-ping joystick less libfreeimage-dev libfreeimage3 libopenblas-dev make mesa-utils mpg123 nano net-tools pavucontrol ppa-purge pulseaudio python-is-python3 python3-dev python3-pip python3-tk rsync screen software-properties-common tmux tree unzip usbutils valgrind vim wget zip zsh
- sudo apt install libfreeimage3 libfreeimage-dev net-tools acpi quota python3-tk
- sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean

## git-lfs
- curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash && sudo apt-get install git-lfs

## ssh server
- sudo apt install openssh-server -y && sudo systemctl start ssh && sudo systemctl enable ssh
- Security settings for ssh:
    - `sudo vi /etc/ssh/sshd_config` and change/uncomment the following lines:
        ```
        PasswordAuthentication no
        PubkeyAuthentication yes
        PermitRootLogin prohibit-password
        PermitEmptyPasswords no
        ```
    - `sudo systemctl restart ssh`

## wireguard ([reference](https://github.com/ut-amrl/amrl-documentation/blob/master/computers/wireguard-vpn.md))
- sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean
- sudo apt-get install wireguard wireguard-tools
- sudo -i
- cd /etc/wireguard && wg genkey | tee wg-private.key | wg pubkey > wg-public.key && touch wg0.conf
- add the contents to wg0.conf
- sudo wg-quick up wg0 && sudo systemctl enable wg-quick@wg0
- (to stop the vpn cleanly and completely: sudo wg-quick down wg0 && sudo systemctl stop wg-quick@wg0 && sudo systemctl disable wg-quick@wg0)

## Nvidia drivers
- Remove the existing nvidia drivers CLEANLY and FULLY by running the following commands in the terminal:
    * sudo apt-get remove --purge '^nvidia-.*'
    * sudo apt-get install ubuntu-desktop
    * sudo rm /etc/X11/xorg.conf
    * echo 'nouveau' | sudo tee -a /etc/modules
    * sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean
- Run `lspci | grep -i nvidia` on terminal to get your nvidia gpu model
    * you might need to do `sudo update-pciids` to update the pci ids
- Go to [nvidia's website](https://www.nvidia.com/download/index.aspx) and get the correct driver version for your gpu. For instance, it could be `550.120` for a Linux 64-bit machine running a GeForce RTX 4070 Max-Q / Mobile GPU (this is a part of the 4070 laptop series).
- **(needs physical access)** Go to 'Software & Updates' -> 'Additional Drivers' and select the 'Using NVIDIA driver metapackage from nvidia-driver-xxx (proprietary)' option. Click 'Apply Changes' and restart.
    * If you do not see your required version listed there, do this: `sudo add-apt-repository ppa:graphics-drivers/ppa && sudo apt-get update`.

## CUDA Toolkit
- check `uname -r` for kernel version. Based on the output, CUDA 12.4 seems supported (CUDA 11 is not).
- Go to [nvidia's website](https://developer.nvidia.com/cuda-toolkit-archive) and download the desired version of CUDA Toolkit for your system. For instance, you could do 12.4 for a Linux 64-bit machine running Ubuntu 22.04. Use deb (local) method. You do not need to install the drivers part again (i.e., the thing we did before).
- sudo reboot

## cuDNN installation ([reference](https://docs.nvidia.com/deeplearning/cudnn/installation/linux.html#package-manager-local-installation))
- Go to [nvidia's website](https://developer.nvidia.com/cudnn) and select the appropriate options for your system. Use deb (local) method.
- sudo reboot

## netplan
- sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean
- sudo apt install netplan.io
- copy over netplan config files (permissions 600) and place in /etc/netplan
    * sudo -i and then cd /etc/netplan && wget -O 01-network-manager-all.yaml https://raw.githubusercontent.com/ut-amrl/spot-arch/refs/heads/orin/docs/orin/config/netplan/01-network-manager-all.yaml && wget -O 02-spotgxp-and-velodyne.yaml https://raw.githubusercontent.com/ut-amrl/spot-arch/refs/heads/orin/docs/orin/config/netplan/02-spotgxp-and-velodyne.yaml && chmod 600 *.yaml
    * have to connect actual devices (one by one) and do ip a to get the eth connection names, modify the netplan config files accordingly
- sudo systemctl disable systemd-networkd-wait-online.service && sudo systemctl mask systemd-networkd-wait-online.service
- sudo systemctl start systemd-networkd && sudo systemctl enable systemd-networkd && sudo netplan apply

## docker install
- first do a clean uninstall:
    - docker system prune -a --volumes; sudo systemctl disable --now docker.service docker.socket; sudo rm /var/run/docker.sock; dockerd-rootless-setuptool.sh uninstall; /usr/bin/rootlesskit rm -rf ~/.local/share/docker
    - for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done
    - sudo apt-get purge docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin docker-ce-rootless-extras
    - sudo rm -rf /var/lib/docker && sudo rm -rf /var/lib/containerd && sudo rm -rf /etc/docker && rm -rf ~/.config/docker
    - sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean
- sudo apt-get update && sudo apt-get install ca-certificates curl
- sudo install -m 0755 -d /etc/apt/keyrings
- sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
- sudo chmod a+r /etc/apt/keyrings/docker.asc
- echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
- sudo apt-get update && sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin docker-ce-rootless-extras
- sudo groupadd docker
- sudo usermod -aG docker $USER

## nvidia container toolkit setup with docker [link](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean
- run `nvidia-ctk --version`, if not installed:
    - curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    - sudo apt-get update && sudo apt-get install nvidia-container-toolkit
        * you might need to do sudo apt --fix-broken install
        * re run sudo apt-get install -y nvidia-container-toolkit
    - sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean
- sudo nvidia-ctk runtime configure --runtime=docker
- sudo systemctl restart docker

## miscellaneous
- Setup libtorch:
    - Go to [pytorch's website](https://pytorch.org/get-started/locally/) and select appropriate options for your system. For instance, choose Linux, LibTorch, C++, and the CUDA version. Then download the `cxx11 ABI` version zip. You can run the following commands in the terminal:
        - `wget https://download.pytorch.org/libtorch/cu124/libtorch-cxx11-abi-shared-with-deps-2.4.1%2Bcu124.zip`
        - `unzip libtorch<TAB>`
        - `sudo mv libtorch /opt/`
- Setup power settings so that it won't turn off:
    - `sudo vi /etc/systemd/logind.conf` and change the following lines:
        ```
        HandleLidSwitch=ignore
        HandleLidSwitchExternalPower=ignore
        HandleLidSwitchDocked=ignore
        ```
    - `sudo systemctl restart systemd-logind`
    - `cd ~ && wget -O disable-blank-screen.sh https://raw.githubusercontent.com/ut-amrl/spot-arch/refs/heads/orin/docs/orin/files/disable-blank-screen.sh && chmod +x disable-blank-screen.sh && sudo mv disable-blank-screen.sh /etc/profile.d/`
- To be able to set user quotas:
    - sudo apt update && sudo apt install quota
    - `sudo vi /etc/fstab` and add `usrquota` to the root partition
        - for instance, change `/dev/root            /                     ext4           defaults                                     0 1` to `/dev/root            /                     ext4           defaults,usrquota                                     0 1`
        - for instance, change `UUID=2f7e2445-2139-4dff-b351-738af9ec49a4 /               ext4    errors=remount-ro 0       1` to `UUID=2f7e2445-2139-4dff-b351-738af9ec49a4 /               ext4    errors=remount-ro,usrquota 0       1`
    - `sudo reboot`
    - `sudo quotacheck -cugm /` to initialize the quota files
    - `sudo quotaon -v /` to turn on the quotas
    - `sudo reboot`
- add to /etc/bash.bashrc:
    ```    
    # Function to prompt for confirmation before running risky docker remove commands
    docker() {
        # List of docker commands that should trigger the prompt
        risky_commands=("rm" "rmi" "prune")

        # Check if the first/second argument is in the list of risky commands
        if [[ " ${risky_commands[@]} " =~ " $1 " || " ${risky_commands[@]} " =~ " $2 " ]]; then
            # Red color output
            RED='\033[0;31m'
            # No color (reset)
            NC='\033[0m'

            echo -e "${RED}Are you sure you want to do this? Make sure you do NOT remove other users' containers/images. (y/n)${NC}"
            read -r confirmation

            if [[ "$confirmation" == "y" || "$confirmation" == "yes" ]]; then
                # If confirmed, run the original docker command
                command docker "$@"
            else
                echo "Aborted."
            fi
        else
            # For all other docker commands, run normally
            command docker "$@"
        fi
    }

    # CUDA
    export CUDA_HOME=/usr/local/cuda
    export PATH=$PATH:$CUDA_HOME/bin
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CUDA_HOME/lib64
    ```

# User Account creation
- From a sudo account, add a new user account: `sudo adduser <username>` and follow the prompts. Once created:
    - Run `sudo -u <username> xdg-user-dirs-update` to create the default directories for the new user.
    - `sudo -u <username> mkdir /home/<username>/.ssh`
    - `sudo -u <username> chmod 700 /home/<username>/.ssh`
    - `sudo -u <username> touch /home/<username>/.ssh/authorized_keys`
    - `sudo -i`
    - Add ssh key(s) to the `authorized_keys` file.
    - `sudo setquota -u <username> 50G 60G 0 0 /` to set the limits for the user (for instance, 50 GB soft limit and 60 GB hard limit).
    - Give sudo permissions to the user if needed: `sudo usermod -aG sudo <username>`.
    - `sudo chown <username>:<username> /home/<username>/.ssh && sudo chown <username>:<username> /home/<username>/.ssh/*`
    - `sudo usermod -aG docker <username>` to add the user to the docker group.
- share spot entry and lab password with the user on UT stache
