# Prereqs
- get added to AMRL VPN, AMRL github
- get a user account on Spot orin

# Set up
- add `export HOST_UID=$(id -u)` to your .bashrc
- run `git lfs install`
- setup your `~/.gitconfig` file like [this](files/.gitconfig). Change the name and email to your own.
- set up [ssh forwarding](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/using-ssh-agent-forwarding) on your local machine, used for sshing into spot user account
- set up X11 forwarding if you want the display to be forwarded to your local machine:
    * either add `ForwardX11 yes` and `ForwardX11Trusted yes` to your `~/.ssh/config` file for the entry for spot orin, or run ssh with `-X` and `-Y` flags
- clone this repo `git clone --recursive --branch orin git@github.com:sadanand1120/spot-arch.git`
- `cd spot-arch/docs/orin/files/spot-base`
- `docker build -t spot-base .` builds the docker image
- `docker compose up -d && docker attach spot-base` starts `spot-base` container and attaches to it:
    * `ctrl+d` inside the container stops and exits it
    * use `docker ps` to check currently running containers
    * use `docker ps -a` to check all containers, including stopped ones
    * use `docker images` to check all images
    * to rettach to a stopped container, use `docker start <container> && docker attach <container>`
- `cd ~/ut-amrl/spot_autonomy/launch/` and add the spot user credentials (shared on UT Stache) to your start-clearpath.launch file (note: NOT the one ending in `.example`)
- to start the default autonomy stack, run `roslaunch spot_autonomy start_all.launch`

# Notes
- [spot-base](files/spot-base/) is the main docker image. [scratch](files/scratch/) contains an equivalent setup where pytorch is built from source manually, instead of using images provided by nvidia (see `files/`)
- the spot-base image has the following already set up:
    * ROS1 (noetic)
    * pytorch (python as well as c++ libtorch), torchvision, opencv
    * git lfs
    * miniconda (at `/opt/miniconda3`). Use `conda-create` instead of conda create (its a wrapper, it sets some torch paths properly if you want to use a different version of torch in your conda env. Additionally, note orin is setup with jetpack 5.1.2. This means for getting torch in your env, only python 3.8 can be used. Resources: [link1](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048) [link2](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html))
    * conda bash completion (so can use TAB for conda commands)
    * all essential AMRL repositories for standard navigation stack, cloned and built, under `~/catkin_ws` and `~/ut-amrl`
    * NOTE: repos are inside the container/image, you will not see it outside the container. Feel free to change the dockerfile and compose.yaml to modify the behaviour (e.g., mount a volume to the container for repos built outside)
- account password always defaults to lab password (shared on UT Stache), unless you change it