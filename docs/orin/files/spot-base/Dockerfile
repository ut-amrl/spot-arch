#FROM ros-noetic-desktop-full-arm64
#FROM arm64v8/ubuntu:focal
# already has torch (python as well as c++ libtorch) torchvision and opencv setup
FROM dustynv/l4t-pytorch:r35.4.1

# env vars
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic
ENV TZ=America/Chicago

ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/python3.8/dist-packages/torch
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/python3.8/dist-packages/torch/lib
ENV Torch_DIR=$Torch_DIR:/usr/local/lib/python3.8/dist-packages/torch/share/cmake/Torch

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

# Set up ROS keys
RUN set -eux; \
    key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
    export GNUPGHOME="$(mktemp -d)"; \
    gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
    mkdir -p /usr/share/keyrings; \
    gpg --batch --export "$key" > /usr/share/keyrings/ros1-latest-archive-keyring.gpg; \
    gpgconf --kill all; \
    rm -rf "$GNUPGHOME"

# Set up ROS sources for Ubuntu Focal (ARM64)
RUN echo "deb [ signed-by=/usr/share/keyrings/ros1-latest-archive-keyring.gpg ] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# Remove opencv so that ros can install its own
RUN apt-get purge -y '*opencv*'
#RUN find /usr -name "*opencv*4.5*" -exec rm -rf {} +

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* python3-rosdep python3-rosinstall python3-vcstools && \
    rm -rf /var/lib/apt/lists/*

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
    python3.8 \
    python3.8-dev \
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
    cmake \
    libssl-dev \
    curl \
    qt5-default \
    wget \
    && rm -rf /var/lib/apt/lists/*

#########################################################################################################################################

RUN apt-get update && apt-get install -y \
    apt-utils \
    curl \
    python3-catkin-tools \
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
    python3-tk \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    ros-noetic-ros-numpy \
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
    ros-noetic-twist-mux \
    ros-noetic-interactive-marker-twist-server \
    ros-noetic-velodyne-pointcloud \
    && rm -rf /var/lib/apt/lists/*

# Install specific protobuf version and Bosdyn packages for spot_ros
RUN pip3 install protobuf==3.20.1 && \
    pip3 install cython empy bosdyn-client==3.1.1 bosdyn-mission==3.1.1 bosdyn-api==3.1.1 bosdyn-core==3.1.1 virtualenv jetson-stats rospkg \
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
    cmake qt5-default libqt5websockets5-dev \
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
    qt5-default \
    libqt5websockets5-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    && rm -rf /var/lib/apt/lists/*

# Add Microsoft repository and install Kinect SDK
RUN curl https://packages.microsoft.com/keys/microsoft.asc | apt-key add - && \
    apt-add-repository "deb https://packages.microsoft.com/ubuntu/18.04/multiarch/prod bionic main" && \
    apt-get update && ACCEPT_EULA=Y apt-get install -y k4a-tools libk4a1.4 libk4a1.4-dev

# Add Kinect udev rules
COPY ./99-k4a.rules /etc/udev/rules.d/99-k4a.rules
#RUN udevadm control --reload-rules && udevadm trigger
#RUN wget https://raw.githubusercontent.com/ut-amrl/k4a_ros/master/99-k4a.rules -O /etc/udev/rules.d/99-k4a.rules && \
#    udevadm control --reload-rules && udevadm trigger

# Install Miniconda to /opt/miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-py310_24.7.1-0-Linux-aarch64.sh && \
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
