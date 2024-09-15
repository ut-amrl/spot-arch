# Flashing the Orin (with SSD boot)
- first flash the orin from default emmc (ie, no ssd) following steps in these resources:
    - set these things on your laptop before starting process to avoid any errors:
        * sudo bash -c "echo -1 > /sys/module/usbcore/parameters/autosuspend" and then check that cat /sys/module/usbcore/parameters/autosuspend shows -1. This is temporary and will reset after reboot
        * sudo ufw disable. Once flashing is done, you can re-enable it with sudo ufw enable
    - note you should install components, ie, leave all by default selected components including the linux image, sdk and runtime components, in jetpack sdk
    - note you'd need to have orin connected to your laptop through usb c to usb a connection using the provided cable, while also have a monitor, keyboard and mouse connected to the orin for oem config that comes right after flashing
    - https://www.youtube.com/watch?v=DKI1k_aP0Qk
    - https://www.youtube.com/watch?v=Ucg5Zqm9ZMk
    - https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/two_ways_to_set_up_software.html
- then add ssd as an external drive following the steps in https://www.youtube.com/watch?v=DKI1k_aP0Qk
- then once ssd is mounted properly, you can flash it from ssd following steps in https://www.youtube.com/watch?v=DKI1k_aP0Qk. Basically steps remain same, just choose NVMe SSD as the target instead of emmc
- other useful resources:
    - https://docs.nvidia.com/jetson/jetpack/introduction/index.html
    - https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html

# Post-setup
- sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean
- sudo apt install python3-pip
- sudo pip3 install -U jetson-stats
- sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean

## basic packages
- sudo apt install build-essential gcc g++ make python3-dev python3-pip git vim emacs curl wget gdb cmake htop tmux screen nano mesa-utils ppa-purge software-properties-common
- sudo apt install python-is-python3 clang-12 clang-format valgrind iputils-ping less mesa-utils net-tools rsync tree unzip usbutils zip zsh htop
- sudo apt-get install libfreeimage3 libfreeimage-dev libopenblas-dev python3-tk ffmpeg espeak mpg123
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

## wireguard
- (update: either do the following, or check out user-space wireguard-go installation https://github.com/WireGuard/wireguard-go)
- building wireguard supported custom kernel:
    - sudo apt-get install libssl-dev
    - follow this [this, read by archive.is](https://medium.com/@ebinzacharias/enabling-wireguard-on-nvidia-jetson-devices-0887e833cb41), while referring to [this](https://wiki.gentoo.org/wiki/WireGuard#Kernel_5.6_and_higher) if needed
- sudo -i
- cd /etc/wireguard && wg genkey | tee wg-private.key | wg pubkey > wg-public.key && touch /etc/wireguard/wg0.conf
- fill in the contents of wg0.conf (and copy over the keys here) in /etc/wireguard
- sudo wg-quick up wg0 && sudo systemctl enable wg-quick@wg0

## netplan
- sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean
- sudo apt install netplan.io
- copy over netplan config files (permissions 644) and place in /etc/netplan
    * have to connect actual devices (one by one) and do ip a to get the eth connection names, modify the netplan config files accordingly
- sudo systemctl start systemd-networkd && sudo systemctl enable systemd-networkd && sudo netplan apply

## docker install
- for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done
- sudo apt-get purge docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin docker-ce-rootless-extras
- sudo rm -rf /var/lib/docker
- sudo rm -rf /var/lib/containerd
- sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean
- sudo apt-get update
- sudo apt-get install ca-certificates curl
- sudo install -m 0755 -d /etc/apt/keyrings
- sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
- sudo chmod a+r /etc/apt/keyrings/docker.asc
- echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
- sudo apt-get update
- sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
- sudo groupadd docker
- sudo usermod -aG docker $USER

## nvidia container toolkit setup with docker [link](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean
- curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
- sudo apt-get update
- sudo apt install apt-utils
- sudo apt-get install -y nvidia-container-toolkit
    - you might need to do sudo apt --fix-broken install
    - re run sudo apt-get install -y nvidia-container-toolkit
- sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean
- sudo nvidia-ctk runtime configure --runtime=docker
- sudo systemctl restart docker

## miscellaneous
- Setup power settings so that it won't turn off:
    - `sudo vi /etc/systemd/logind.conf` and change the following lines:
        ```
        HandleLidSwitch=ignore
        HandleLidSwitchExternalPower=ignore
        HandleLidSwitchDocked=ignore
        ```
    - `sudo systemctl restart systemd-logind`
- To be able to set user quotas:
    - sudo apt update && sudo apt install quota
    - `sudo vi /etc/fstab` and add `usrquota` to the root partition
        - for instance, change `UUID=401615fc-572a-4c30-9c0d-d62dd13db87d /               ext4    errors=remount-ro 0       1` to `UUID=401615fc-572a-4c30-9c0d-d62dd13db87d /               ext4    errors=remount-ro,usrquota 0       1`
    - `sudo reboot`
    - `sudo quotacheck -cugm /` to initialize the quota files
    - `sudo quotaon -v /` to turn on the quotas
    - `sudo reboot`
- make orin high performance by sudo nvpmodel -m 0 and sudo jetson_clocks


# User setup
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
- set the host uid on each user by adding export HOST_UID=$(id -u) to user acc .bashrc
- git lfs install
- setup ~/.vimrc and ~/.gitconfig as in `files/`
- set up your [ssh forwarding](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/using-ssh-agent-forwarding)

## building docker container
- spot-base is the main docker image. scratch contains an equivalent setup where pytorch is built from source manually, instead of using images provided by nvidia (see `files/`)
- docker build -t <image_name> . in the folder containing appropriate Dockerfile (say spot-base image)
- docker compose up -d && docker attach spot-base in the folder containing appropriate docker-compose.yaml
- add amrl credentials to spot launch file
- docker start spot-base && docker attach spot-base whenever needed to re-enter the container

# TODOs
- install cuda 11.8 toolkit so that you can use 11.8 when needed as well
    * https://developer.nvidia.com/blog/simplifying-cuda-upgrades-for-nvidia-jetson-users/
    * https://docs.nvidia.com/cuda/cuda-for-tegra-appnote/index.html#cuda-upgradable-package-for-jetson
- add miniconda to the image
- add conda bash completion to the image

# miscellaneous resources
* https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html
* https://www.youtube.com/watch?v=LUxyNyCl4ro
* https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/index.html
* https://developer.nvidia.com/embedded/learn/get-started-jetson-agx-orin-devkit
* https://github.com/dusty-nv/jetson-containers
* https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
* https://github.com/pytorch/vision/blob/main/CONTRIBUTING.md#development-installation
* opencv with cuda: https://www.youtube.com/watch?v=UiZaM-Wbc6A
* cmake build from source: https://forums.developer.nvidia.com/t/how-does-jetson-nono-update-cmake-to-3-18/182786/4, https://forums.developer.nvidia.com/t/halide-install-in-jetson-orin/219134/4
* installing missing dev headers and stuff on l4t-cuda, l4t-tensorrt runtime images:
    - https://forums.developer.nvidia.com/t/how-to-install-cuda-cudnn-and-tensorrt-on-jetpack-4-6-jetson-agx-xavier/235014
    - https://forums.developer.nvidia.com/t/how-to-install-tensorrt-on-top-of-a-l4t-base-image/200768
    - https://forums.developer.nvidia.com/t/how-to-install-cuda-cudnn-tensorrt-by-deb/296759
    - https://repo.download.nvidia.cn/jetson/#Jetpack%205.1.2
* to mount volumes for cuda, cudnn etc support from host to container, do something like this in compose.yaml:
```
volumes:
    - /usr/local/cuda:/usr/local/cuda
    - /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra
    - /usr/lib/aarch64-linux-gnu/libcudnn.so.8:/usr/lib/aarch64-linux-gnu/libcudnn.so.8
    - /usr/include/opencv4:/usr/include/opencv4
    - /usr/lib/aarch64-linux-gnu/libopencv_core.so:/usr/lib/aarch64-linux-gnu/libopencv_core.so
    - /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so:/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so
    - /usr/lib/aarch64-linux-gnu/libopencv_highgui.so:/usr/lib/aarch64-linux-gnu/libopencv_highgui.so
    - /usr/src/tensorrt:/usr/src/tensorrt
    - /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api
    - /usr/share/opencv4:/usr/share/opencv4
    - /opt/nvidia/vpi:/opt/nvidia/vpi
```