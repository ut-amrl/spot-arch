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
- if emmc and nvme both have os installed, you might need to adjust the boot order following the section 2 in this link [link](https://www.linuxbabe.com/command-line/how-to-use-linux-efibootmgr-examples)

# Post-setup
- sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean
- sudo apt install python3-pip firefox
- sudo pip3 install --no-cache-dir -v -U jetson-stats
- sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean

## basic packages
- sudo apt install build-essential gcc g++ make python3-dev python3-pip git vim emacs curl wget gdb cmake htop tmux screen nano mesa-utils ppa-purge software-properties-common
- sudo apt install apt-utils python-is-python3 clang-12 clang-format valgrind iputils-ping less mesa-utils net-tools rsync tree unzip usbutils zip zsh htop
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
(update: either do the following, or check out user-space wireguard-go installation https://github.com/WireGuard/wireguard-go) (update 2: on jetpack 6, kernel 5.15, modifying kernel config DID NOT work. So, had to use wireguard-go)
- sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean
- sudo apt update && sudo apt install wireguard-tools
- cd ~/.ssh && touch authorized_keys and then add your public key to the file
- sudo -i
- cd /etc/wireguard && wg genkey | tee wg-private.key | wg pubkey > wg-public.key && touch wg0.conf and paste the contents of the wireguard config file (see config folder) and modify the keys accordingly
- wget -O go1.20.9.linux-arm64.tar.gz https://go.dev/dl/go1.20.9.linux-arm64.tar.gz
- sudo tar -C /usr/local -xzf go1.20.9.linux-arm64.tar.gz && rm go1.20.9.linux-arm64.tar.gz
- export PATH=$PATH:/usr/local/go/bin && go version
- export PATH=$PATH:/usr/bin && git --version
- git clone https://git.zx2c4.com/wireguard-go && cd wireguard-go && make -j$(nproc)
- NOTE: modify 10.1.0.3 to your assigned IP below and in the service file
- cd /etc/wireguard/wireguard-go && ./wireguard-go wg0 && sudo ip address add dev wg0 10.1.0.3/32 && sudo wg setconf wg0 /etc/wireguard/wg0.conf && sudo ip link set up dev wg0 && sudo ip route add 10.0.0.0/24 dev wg0 && sudo ip route add 10.1.0.0/16 dev wg0 && sudo ip route add 10.2.0.0/16 dev wg0 && sudo ip route add 10.3.0.0/16 dev wg0
- sudo vi /etc/systemd/system/wireguard-setup.service
    ```
    [Unit]
    Description=Custom WireGuard Setup based on wireguard-go
    After=network.target

    [Service]
    Type=oneshot
    RemainAfterExit=yes
    ExecStart=/bin/bash -c 'cd /etc/wireguard/wireguard-go && ./wireguard-go wg0 && sudo ip address add dev wg0 10.1.0.3/32 && sudo wg setconf wg0 /etc/wireguard/wg0.conf && sudo ip link set up dev wg0 && sudo ip route add 10.0.0.0/24 dev wg0 && sudo ip route add 10.1.0.0/16 dev wg0 && sudo ip route add 10.2.0.0/16 dev wg0 && sudo ip route add 10.3.0.0/16 dev wg0'

    [Install]
    WantedBy=multi-user.target
    ```
- sudo wg-quick down wg0 && sudo systemctl daemon-reload && sudo systemctl start wireguard-setup.service && sudo systemctl enable wireguard-setup.service
- (To stop wireguard completely: sudo wg-quick down wg0 && sudo systemctl stop wireguard-setup.service && sudo systemctl disable wireguard-setup.service)
- (NOTE: you MIGHT need to do the last daemon reload and start service commands again once in a while, if the wireguard connection is lost for some reason)

## netplan
- sudo apt update && sudo apt upgrade && sudo apt autoremove && sudo apt clean && sudo apt autoclean
- sudo apt install netplan.io
- copy over netplan config files (permissions 600) and place in /etc/netplan
    * sudo -i and then cd /etc/netplan && wget -O 01-network-manager-all.yaml https://raw.githubusercontent.com/ut-amrl/spot-arch/refs/heads/orin/docs/orin/config/netplan/01-network-manager-all.yaml && wget -O 02-spotgxp-and-velodyne.yaml https://raw.githubusercontent.com/ut-amrl/spot-arch/refs/heads/orin/docs/orin/config/netplan/02-spotgxp-and-velodyne.yaml && chmod 600 *.yaml
    * have to connect actual devices (one by one) and do ip a to get the eth connection names, modify the netplan config files accordingly
- sudo systemctl disable systemd-networkd-wait-online.service && sudo systemctl mask systemd-networkd-wait-online.service
- sudo systemctl start systemd-networkd && sudo systemctl enable systemd-networkd && sudo netplan apply

## docker install (rootful)
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
- run nvidia-ctk --version, if not installed:
    - curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    - sudo apt-get update && sudo apt-get install nvidia-container-toolkit
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
    - `cd ~ && wget -O disable-blank-screen.sh https://raw.githubusercontent.com/ut-amrl/spot-arch/refs/heads/orin/docs/orin/files/disable-blank-screen.sh && chmod +x disable-blank-screen.sh && sudo mv disable-blank-screen.sh /etc/profile.d/`
- To be able to set user quotas:
    - sudo apt update && sudo apt install quota
    - `sudo vi /etc/fstab` and add `usrquota` to the root partition
        - for instance, change `/dev/root            /                     ext4           defaults                                     0 1` to `/dev/root            /                     ext4           defaults,usrquota                                     0 1`
    - `sudo reboot`
    - `sudo quotacheck -cugm /` to initialize the quota files
    - `sudo quotaon -v /` to turn on the quotas
    - `sudo reboot`
- Log everything for docker:
    - sudo mkdir /var/log/docker_logs && sudo chown root:root /var/log/docker_logs/ && sudo chmod 755 /var/log/docker_logs/
    - sudo touch /var/log/docker_logs/docker_command_log.txt /var/log/docker_logs/docker_events_log.txt && sudo chmod 666 /var/log/docker_logs/docker_command_log.txt /var/log/docker_logs/docker_events_log.txt
    - sudo vi /etc/bash.bashrc
        ```bash
        # CUDA
        export CUDA_HOME=/usr/local/cuda
        export PATH=$PATH:$CUDA_HOME/bin
        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CUDA_HOME/lib64

        # Function to log all docker commands and prompt for risky ones
        docker() {
            # Red color output
            RED='\033[0;31m'
            NC='\033[0m'  # No color

            # List of risky keywords to detect in the entire command
            risky_keywords=("rm" "rmi" "prune" "destroy" "die" "kill" "pause" "rename" "restart" "stop" "delete" "untag" "remove" "disable" "unmount" "disconnect" "reload")
            spam_keywords=("ps" "version" "info")
            # Combine risky keywords into a regex pattern that matches whole words
            risky_pattern="\\b($(IFS='|'; echo "${risky_keywords[*]}"))\\b"
            spam_pattern="\\b($(IFS='|'; echo "${spam_keywords[*]}"))\\b"

            # Log file path
            log_file="/var/log/docker_logs/docker_command_log.txt"
            timestamp=$(date '+%Y-%m-%d %H:%M:%S.%N')
            username=$(whoami)

            # Check if any risky keyword appears in the entire command
            if [[ "$*" =~ $risky_pattern ]]; then
                risky_match=$(echo "$*" | grep -Eo "$risky_pattern")
                echo -e "${RED}Are you sure you want to do this? Make sure you do NOT affect other users' objects. (y/n)${NC}"
                read -r confirmation

                if [[ "$confirmation" == "y" || "$confirmation" == "yes" ]]; then
                    # Run the original docker command
                    echo "[$timestamp] User: $username | Command: docker $* | Found as: risky $risky_match:proceeded" >> "$log_file"
                    command docker "$@"
                    return
                else
                    echo "Aborted."
                    echo "[$timestamp] User: $username | Command: docker $* | Found as: risky $risky_match:aborted" >> "$log_file"
                    return 1
                fi
            fi

            # dont log spam commands as they dont trigger any important event
            if ! [[ "$*" =~ $spam_pattern ]]; then
                echo "[$timestamp] User: $username | Command: docker $* | Found as: not risky" >> "$log_file"
            fi

            # If not risky, run the command directly
            command docker "$@"
        }

        ```
    - sudo vi /usr/local/bin/docker_event_logger.sh
        ```bash
        #!/bin/bash
        log_file="/var/log/docker_logs/docker_events_log.txt"

        # can use grep --line-buffered -E 'create|destroy|start|stop|restart|update|remove' to filter out events
        docker events --format '{{json .}}' | \
        while read -r event; do
            # Extract the correct time field
            time=$(echo "$event" | jq -r '.timeNano')
            # Check if 'time' is not empty
            if [[ -n "$time" && "$time" != "null" ]]; then
                # Convert timeNano to seconds and nanoseconds
                timestamp=$(date -d "@$((time/1000000000))" '+%Y-%m-%d %H:%M:%S').$(printf "%09d" $((time%1000000000)))
            else
                # Handle cases where timeNano is missing
                timestamp=$(date '+%Y-%m-%d %H:%M:%S.%N')
            fi
            action=$(echo "$event" | jq -r '.Action')
            type=$(echo "$event" | jq -r '.Type')
            id=$(echo "$event" | jq -r '.Actor.ID // "<no value>"')
            name=$(echo "$event" | jq -r '.Actor.Attributes.name // "<no value>"')
            echo "[$timestamp] Action: $action | Type: $type | ID: $id | Name: $name" >> "$log_file"
        done
        ```
    - sudo chmod +x /usr/local/bin/docker_event_logger.sh
    - sudo vi /etc/systemd/system/docker-event-logger.service
        ```bash
        [Unit]
        Description=Docker Event Logger
        After=docker.service
        Requires=docker.service

        [Service]
        Type=simple
        ExecStart=/usr/local/bin/docker_event_logger.sh
        Restart=always
        User=root

        [Install]
        WantedBy=multi-user.target
        ```
    - sudo systemctl daemon-reload && sudo systemctl start docker-event-logger.service && sudo systemctl enable docker-event-logger.service
    - sudo vi /etc/logrotate.d/docker_command_log
        ```bash
        /var/log/docker_logs/docker_command_log.txt {
            monthly
            rotate 12
            compress
            missingok
            notifempty
            copytruncate
        }

        ```
    - sudo vi /etc/logrotate.d/docker_events_log
        ```bash
        /var/log/docker_logs/docker_events_log.txt {
            monthly
            rotate 12
            compress
            missingok
            notifempty
            create 644 root root
            postrotate
                systemctl restart docker-event-logger.service > /dev/null 2>&1 || true
            endscript
        }
        ```
- make orin high performance by sudo nvpmodel -m 0 and sudo jetson_clocks
    * nvpmodel has 4 modes 0-3, where 0 is the max performance/no constraints mode, while performance increases from 1 (only 4 CPUs active) to 3
    * sudo jetson_clocks is not persistent accross boots
- sudo vi /etc/systemd/system/xhostplus.service and add the following (needed for docker):
    ```
    [Unit]
    Description=Allow local X server connections

    [Service]
    ExecStart=/usr/bin/xhost +

    [Install]
    WantedBy=multi-user.target
    ```
    - sudo systemctl enable xhostplus.service && sudo systemctl start xhostplus.service

# User Account creation
- From a sudo account, add a new user account: `sudo adduser <username>` and follow the prompts. Once created:
    - Run `sudo -u <username> xdg-user-dirs-update` to create the default directories for the new user.
    - `sudo -u <username> mkdir /home/<username>/.ssh`
    - `sudo -u <username> chmod 700 /home/<username>/.ssh`
    - `sudo -u <username> touch /home/<username>/.ssh/authorized_keys`
    - `sudo -i`
    - Add ssh key(s) to the `authorized_keys` file.
    - `sudo setquota -u <username> 70G 80G 0 0 /` to set the limits for the user (for instance, 70 GB soft limit and 80 GB hard limit).
    - Give sudo permissions to the user if needed: `sudo usermod -aG sudo <username>`.
    - `sudo chown <username>:<username> /home/<username>/.ssh && sudo chown <username>:<username> /home/<username>/.ssh/*`
    - `sudo usermod -aG docker <username>` to add the user to the docker group.
    - `sudo usermod -aG jtop <username>`
- share spot entry and lab password with the user on UT stache

# TODOs
- install cuda 11.8 toolkit so that you can use 11.8 when needed as well
    * https://developer.nvidia.com/blog/simplifying-cuda-upgrades-for-nvidia-jetson-users/
    * https://docs.nvidia.com/cuda/cuda-for-tegra-appnote/index.html#cuda-upgradable-package-for-jetson
- setup user-only docker mimic using logging docker events:
    * https://docs.docker.com/reference/cli/docker/system/events/
    * use a external python script for json parsing and telling shell script what to do
    * put constraints on storage space, deletion, creation etc

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
* deleting a user including all files etc: `sudo userdel -r <username>`