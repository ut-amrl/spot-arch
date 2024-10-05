# FROM dustynv/l4t-pytorch:r35.4.1
FROM dustynv/l4t-pytorch:r36.3.0-cu124

# env vars
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble
ENV TZ=America/Chicago

ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/python3.10/dist-packages/torch
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/python3.10/dist-packages/torch/lib
ENV Torch_DIR=$Torch_DIR:/usr/local/lib/python3.10/dist-packages/torch/share/cmake/Torch

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
LABEL com.nvidia.volumes.needed="nvidia_driver"

ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${PATH}:${CUDA_HOME}/bin
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${CUDA_HOME}/lib64
ENV PIP_ROOT_USER_ACTION=ignore

# Set up timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y apt-utils && \
    apt-get install -q -y --no-install-recommends tzdata dirmngr gnupg2 build-essential && \
    rm -rf /var/lib/apt/lists/*

# ros humble
RUN locale
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN locale

RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository universe

RUN apt-get update && apt-get install -y curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y ros-humble-desktop-full
RUN apt-get install -y ros-dev-tools

# SHELL ["/bin/bash", "-c"]
# RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# # Remove opencv so that ros can install its own
# RUN apt-get purge -y '*opencv*'
# #RUN find /usr -name "*opencv*4.5*" -exec rm -rf {} +

# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-noetic-desktop-full=1.5.0-1* python3-rosdep python3-rosinstall python3-vcstools && \
#     rm -rf /var/lib/apt/lists/*

# Copy entrypoint script
COPY ./ros_entrypoint.sh /

# Set up the ROS entrypoint and default command
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

#########################################################################################################################################

# tools for torch torchvision
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.10 \
    python3.10-dev \
    python3-pip \
    python-is-python3 \
    libopenblas-base \
    libopenblas-dev \
    libopenmpi-dev \
    libomp-dev \
    libjpeg-dev \
    zlib1g-dev \
    libpython3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libqt5websockets5-dev \
    qtcreator \
    qtbase5-dev \
    qt5-qmake \
    qtchooser \
    qtbase5-dev-tools \
    cmake \
    libssl-dev \
    curl \
    wget \
    && rm -rf /var/lib/apt/lists/*

#########################################################################################################################################

RUN apt-get update && apt-get install -y \
    apt-utils \
    curl \
    software-properties-common \
    build-essential \
    clang-12 \
    clang-format \
    cmake \
    g++ \
    gcc \
    gdb \
    git \
    nano \
    valgrind \
    vim \
    emacs \
    iputils-ping \
    less \
    mesa-utils \
    net-tools \
    rsync \
    tmux \
    screen \
    tree \
    unzip \
    usbutils \
    zip \
    zsh \
    wget \
    htop \
    ppa-purge \
    libfreeimage3 \
    libfreeimage-dev \
    libopenblas-dev \
    joystick \
    ffmpeg \
    espeak \
    mpg123 \
    && rm -rf /var/lib/apt/lists/*

# Install Git Large File Storage (git-lfs)
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash && \
    apt-get install -y git-lfs

# not needed as forwarding audio from host
#RUN apt-get update && apt-get install -y \
#    alsa-base alsa-utils pulseaudio pavucontrol \
#    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y && apt-get autoclean -y && apt-get clean -y

################################################################################################## (packages)

# Install dependencies for amrl_shared_lib
RUN apt-get update && apt-get install -y \
    libgtest-dev libgoogle-glog-dev cmake build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for spot_ros
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-tk \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstool \
    ros-humble-twist-mux \
    ros-humble-interactive-marker-twist-server \
    ros-humble-velodyne-pointcloud \
    && rm -rf /var/lib/apt/lists/*

# Install specific protobuf version and Bosdyn packages for spot_ros
RUN pip3 install protobuf==3.20.1 && \
    pip3 install cython empy bosdyn-client==3.1.1 bosdyn-mission==3.1.1 bosdyn-api==3.1.1 bosdyn-core==3.1.1 virtualenv jetson-stats rospkg ros2-numpy \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for graph_navigation
RUN apt-get update && apt-get install -y \
    libgoogle-glog-dev libgflags-dev liblua5.1-0-dev \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for enml
RUN apt-get update && apt-get install -y \
    liblua5.1-dev libeigen3-dev \
    libjpeg8-dev libgoogle-perftools-dev \
    libsuitesparse-dev libblas-dev liblapack-dev libopenmpi-dev \
    libgoogle-glog-dev libgflags-dev libceres-dev libtbb-dev \
    libncurses5-dev libpopt-dev \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for webviz and robofleet_client
RUN apt-get update && apt-get install -y \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for k4a_ros
RUN apt-get update && apt-get install -y \
    liblua5.1-0-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libgoogle-perftools-dev \
    cimg-dev \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for spot_autonomy
RUN apt-get update && apt-get install -y \
    libgoogle-glog-dev \
    libgflags-dev \
    && rm -rf /var/lib/apt/lists/*

# # Add Microsoft repository and install Kinect SDK
# RUN curl https://packages.microsoft.com/keys/microsoft.asc | apt-key add - && \
#     apt-add-repository "deb https://packages.microsoft.com/ubuntu/18.04/multiarch/prod bionic main" && \
#     apt-get update && ACCEPT_EULA=Y apt-get install -y k4a-tools libk4a1.4 libk4a1.4-dev

# Switch to Ubuntu 20.04 (Focal) sources temporarily for libsoundio1 installation
RUN cp /etc/apt/sources.list /etc/apt/sources.list.backup && \
    sed -i 's/jammy/focal/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y libsoundio1

RUN wget https://packages.microsoft.com/ubuntu/18.04/multiarch/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.2_arm64.deb && \
    wget https://packages.microsoft.com/ubuntu/18.04/multiarch/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.2_arm64.deb && \
    wget https://packages.microsoft.com/ubuntu/18.04/multiarch/prod/pool/main/k/k4a-tools/k4a-tools_1.4.2_arm64.deb


RUN echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections && \
    echo 'libk4a1.4 libk4a1.4/accept-eula boolean true' | debconf-set-selections && \
    dpkg -i libk4a1.4_1.4.2_arm64.deb && \
    dpkg -i libk4a1.4-dev_1.4.2_arm64.deb && \
    dpkg -i k4a-tools_1.4.2_arm64.deb

# Restore the original sources list for Ubuntu 22.04 (Jammy)
RUN mv /etc/apt/sources.list.backup /etc/apt/sources.list && \
    apt-get update

# Add Kinect udev rules
COPY ./99-k4a.rules /etc/udev/rules.d/99-k4a.rules
#RUN udevadm control --reload-rules && udevadm trigger
#RUN wget https://raw.githubusercontent.com/ut-amrl/k4a_ros/master/99-k4a.rules -O /etc/udev/rules.d/99-k4a.rules && \
#    udevadm control --reload-rules && udevadm trigger

# Install Miniconda to /opt/miniconda
RUN wget -O Miniconda3-py310_24.7.1-0-Linux-aarch64.sh https://repo.anaconda.com/miniconda/Miniconda3-py310_24.7.1-0-Linux-aarch64.sh && \
    bash Miniconda3-py310_24.7.1-0-Linux-aarch64.sh -b -p /opt/miniconda3 && \
    rm Miniconda3-py310_24.7.1-0-Linux-aarch64.sh && \
    /opt/miniconda3/bin/conda config --set auto_activate_base false

# conda bash completion
RUN /opt/miniconda3/bin/conda install -c conda-forge conda-bash-completion -y

RUN cp -r /usr/include/eigen3/* /usr/include/ && \
    apt-get update && apt-get upgrade -y && apt-get autoremove -y && apt-get autoclean -y && apt-get clean -y && \
    rm -rf /var/lib/apt/lists/* && \
    rosdep update --rosdistro $ROS_DISTRO && \
    git lfs install 2>/dev/null
