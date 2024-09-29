# Prereqs
- get added to AMRL VPN, AMRL github
- get a user account on Spot orin

**Note all docker images and containers of all users are stored in `/var/lib/docker`. Please use tag $USER as shown below for all your images and containers to avoid conflicts with other users' images/containers.**

# Orin booting procedures (VERY IMPORTANT)
Orin has been observed to have boot issues if the following procedure is not followed properly. Fixing it in case of a boot issue is a bit of a hassle (takes ~1 hour using sdkmanager). Orin has the following connections through its ports:
1. USB A and DisplayPort from the display screen
2. USB A from the speaker
3. USB C from Kinect camera
4. USB A from the USB hub on the back side
5. USB C for power (which either connects directly to the usb c wall power or to the power bank)

Also, note orin does not have any battery of its own. So orin will shut down when connection 5 is removed.
Now the process is as always:
* If you want to reboot orin, use `sudo poweroff` over ssh or just disconnect the usb c power (connection 5). Then disconnect all peripherals (connections 2, 3 and 4). DO NOT do `sudo reboot` over ssh unless peripherals 2, 3 and 4 are disconnected. Connection 1 can stay connected. Then plug back power (connection 5) and wait for orin to boot up. Once it boots up, plug back all peripherals.
* Once you are done with your work, you would most likely need to disconnect power bank from orin and connect to wall power. Follow similar process there as well: i.e., disconnect power bank, remove the aforementioned peripherals and then connect to wall power, wait for boot up and confirm its booted up. DO NOT connect the peripherals 2, 3 and 4 if you are done with your work. This is so that people can run `sudo reboot` over ssh remotely if need be without worrying about the peripherals being connected.
* **BEFORE LEAVING the lab, PLEASE make sure that**
    - the orin is fully booted up: please check that the display screen shows up AND that you can ssh into it.
    - peripherals 2, 3 and 4 are disconnected from orin
    - power bank is connected to wall power
    - orin is connected to wall power
    - spot batteries are put back on charging

# Set up
- add to your `~/.bashrc`:
    ```
    export HOST_UID=$(id -u)
    export XAUTHORITY=$HOME/.Xauthority
    export PULSE_SERVER=/run/user/$HOST_UID/pulse/native
    echo $DISPLAY > /tmp/.display_env_$HOST_UID
    ```
- run `git lfs install`
- setup your `~/.gitconfig` file like [this](files/.gitconfig). Change the name and email to your own.
    * `cd $HOME && wget -O .gitconfig https://raw.githubusercontent.com/ut-amrl/spot-arch/refs/heads/orin/docs/orin/files/.gitconfig`
- set up [ssh forwarding](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/using-ssh-agent-forwarding) on your local machine, used for sshing into spot user account
- set up X11 forwarding if you want the display to be forwarded to your local machine:
    * either add `ForwardX11 yes` and `ForwardX11Trusted yes` to your `~/.ssh/config` file for the entry for spot orin, or run ssh with `-X` and `-Y` flags
- clone this repo `git clone --recursive --branch orin git@github.com:ut-amrl/spot-arch.git`
- `cd spot-arch/docs/orin/files/spot-ros1-jp5`
- `docker build -t spot-ros1-jp5:${USER} .` builds the docker image
- `docker compose up -d && docker attach spot-ros1-jp5-${USER}` starts `spot-ros1-jp5-${USER}` container and attaches to it
    * check [Notes](https://github.com/ut-amrl/spot-arch/blob/orin/docs/orin/userguide.md#notes) below for docker commands info
- `cd ~/ut-amrl/spot_autonomy/launch/` and add the spot user credentials (shared on UT Stache) to `start_clearpath_spot.launch` (NOT the one ending in `.example`)
- to start the default autonomy stack, run `roslaunch spot_autonomy start_all.launch`

# Notes
- orin is setup with jetpack 6.0.
- [spot-ros1-jp5](files/spot-ros1-jp5/) is the main docker image. [spot-ros1-jp5-scratch](files/spot-ros1-jp5-scratch/) contains an equivalent setup where pytorch is built from source manually, instead of using images provided by nvidia (see `files/`)
- the spot-ros1-jp5 image has the following already set up:
    * ROS1 (noetic)
    * pytorch (python as well as c++ libtorch), torchvision, opencv (Resources: [link1](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048) [link2](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html))
    * git lfs
    * miniconda (at `/opt/miniconda3`). **Use `conda-create` instead of conda create** (its a wrapper, it sets some torch paths properly if you want to use a different version of torch in your conda env)
    * conda bash completion (so can use TAB for conda commands)
    * all essential AMRL repositories for standard navigation stack, cloned and built, under `~/catkin_ws` and `~/ut-amrl`
    * NOTE: repos are inside the container/image, you will not see it outside the container. Feel free to change the dockerfile and compose.yaml to modify the behaviour (e.g., mount a volume to the container for repos built outside)
- account password always defaults to lab password (shared on UT Stache), unless you change it
- docker commands:
    * `ctrl+d` inside the container stops and exits it
    * use `docker ps` to check currently running containers
    * use `docker ps -a` to check all containers, including stopped ones
    * use `docker images` to check all images
    * to rettach to a stopped container, use `docker start <container> && docker attach <container>`
    * to check which image a container is linked to, use `docker inspect --format='{{.Config.Image}}' <container>`
    * use `docker stop <container>` to stop a running container
    * use `docker rm <container>` to remove a stopped container
    * use `docker rmi <image>` to remove an image
- highly recommended to use docker for all projects, and to not do development on native host
- IMPORTANT: if you have to reboot orin, you need to unplug all peripherals from the orin ports (except usb-c power and displayport). Replug once it boots up.