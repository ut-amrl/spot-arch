# Set BUILD_ROSV to noetic, humble, or foxy to choose ROS version (defaults to noetic)
ARG BUILD_ROSV=noetic

# ROS version-specific base images
FROM docker.io/library/ubuntu:20.04 AS noetic-base
FROM docker.io/library/ubuntu:22.04 AS humble-base
FROM docker.io/library/ubuntu:22.04 AS foxy-base

# Select the appropriate base image based on BUILD_ROSV
FROM ${BUILD_ROSV}-base AS final-base

# Re-declare the ARG in the final stage to make it available
ARG BUILD_ROSV=noetic

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=America/Chicago \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=${BUILD_ROSV} \
    PIP_ROOT_USER_ACTION=ignore \
    BUILD_ROSV=${BUILD_ROSV}

# Base system setup, utilities, and timezone configuration
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential \
    tzdata \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    dirmngr \
    apt-utils \
    software-properties-common \
    && ln -sf /usr/share/zoneinfo/$TZ /etc/localtime \
    && echo $TZ > /etc/timezone \
    && rm -rf /var/lib/apt/lists/*

# Install CMake (apt's version is outdated)
RUN wget https://cmake.org/files/v3.26/cmake-3.26.0-linux-x86_64.sh -O /tmp/cmake.sh && \
    mkdir -p /opt/cmake && \
    sh /tmp/cmake.sh --prefix=/opt/cmake --skip-license && \
    ln -sf /opt/cmake/bin/cmake /usr/local/bin/cmake && \
    rm /tmp/cmake.sh && \
    cmake --version

# Install latest Go
RUN wget https://go.dev/dl/go1.24.3.linux-amd64.tar.gz && \
    rm -rf /usr/local/go && \
    tar -C /usr/local -xzf go1.24.3.linux-amd64.tar.gz && \
    rm go1.24.3.linux-amd64.tar.gz && \
    /usr/local/go/bin/go version

# ROS sources and keys setup - using modern ros-apt-source packages
RUN if [ "$BUILD_ROSV" = "noetic" ]; then \
    # For ROS1 Noetic, we need to add the ROS1 repository first to get ros-apt-source
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1.list && \
    apt-get update && apt-get install -y ros-apt-source && \
    rm /etc/apt/sources.list.d/ros1.list; \
    else \
    # For ROS2, add ROS2 repository first to get ros2-apt-source
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y ros2-apt-source && \
    rm /etc/apt/sources.list.d/ros2.list; \
    fi

#########################################################################################################################################

# tools for torch torchvision (keeping structure from spot-arch)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-dev \
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
    $(if [ "$(lsb_release -sc)" = "focal" ]; then echo "qt5-default"; else echo "qtbase5-dev qt5-qmake"; fi) \
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
    gcc-10 \
    g++-10 \
    gdb \
    git \
    ninja-build \
    valgrind \
    nano \
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
    joystick \
    ffmpeg \
    espeak \
    mpg123 \
    # Python and development dependencies (ROS1 only)
    $(if [ "$BUILD_ROSV" = "noetic" ]; then echo "python3-catkin-tools python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool"; else echo "python3-rosdep"; fi) \
    # Mathematical and scientific libraries
    libatlas-base-dev \
    libsuitesparse-dev \
    libblas-dev \
    liblapack-dev \
    libceres-dev \
    libtbb-dev \
    # Additional scientific computing libraries
    libboost-all-dev \
    libboost-program-options-dev \
    libboost-graph-dev \
    libboost-system-dev \
    libflann-dev \
    libmetis-dev \
    libsqlite3-dev \
    libglew-dev \
    libcgal-dev \
    # Testing frameworks
    libgtest-dev \
    libgmock-dev \
    # Qt libraries
    qtbase5-dev \
    libqt5opengl5-dev \
    libqt5websockets5-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Git Large File Storage (git-lfs)
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash && \
    apt-get update && \
    apt-get install -y git-lfs && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y && apt-get autoclean -y && apt-get clean -y

################################################################################################## (packages)

# Install dependencies for amrl_shared_lib
RUN apt-get update && apt-get install -y \
    libgtest-dev libgoogle-glog-dev cmake build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install specific protobuf version and essential packages
RUN pip3 install protobuf==3.20.1 cython empy virtualenv

# Install dependencies for graph_navigation (non-ROS components)
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

# Install dependencies for webviz (non-ROS components)
RUN apt-get update && apt-get install -y \
    cmake \
    $(if [ "$(lsb_release -sc)" = "focal" ]; then echo "qt5-default"; else echo "qtbase5-dev qt5-qmake"; fi) \
    libqt5websockets5-dev \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for cobot_autonomy (non-ROS components)
RUN apt-get update && apt-get install -y \
    $(if [ "$(lsb_release -sc)" = "focal" ]; then echo "qt5-default"; else echo "qtbase5-dev qt5-qmake"; fi) \
    libqt5websockets5-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS desktop-full based on version
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${BUILD_ROSV}-desktop-full \
    ros-${BUILD_ROSV}-twist-mux \
    ros-${BUILD_ROSV}-interactive-marker-twist-server \
    ros-${BUILD_ROSV}-velodyne-pointcloud \
    && rm -rf /var/lib/apt/lists/*

# Install ROS version-specific packages
RUN if [ "$BUILD_ROSV" = "noetic" ]; then \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-costmap-2d \
    ros-noetic-ros-numpy \
    && rm -rf /var/lib/apt/lists/* && \
    pip3 install rospkg; \
    elif [ "$BUILD_ROSV" = "humble" ] || [ "$BUILD_ROSV" = "foxy" ]; then \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-${BUILD_ROSV}-nav2-costmap-2d \
    && rm -rf /var/lib/apt/lists/*; \
    fi

# Build and install Ceres Solver (without CUDA support for NUC)
# make and install twice to avoid issues with parallel build
RUN git clone --recursive https://github.com/ceres-solver/ceres-solver.git /ceres-solver && \
    mkdir /ceres-bin && cd /ceres-bin && \
    cmake ../ceres-solver -DBUILD_TESTING=OFF && \
    make -j$(nproc) && make install && \
    make -j$(nproc) && make install

# Initialize rosdep (after ROS packages are installed)
RUN rosdep init && rosdep update

# Install Miniconda to /opt/miniconda and conda-bash-completion
RUN wget -O Miniconda3-py310_24.7.1-0-Linux-x86_64.sh https://repo.anaconda.com/miniconda/Miniconda3-py310_24.7.1-0-Linux-x86_64.sh && \
    bash Miniconda3-py310_24.7.1-0-Linux-x86_64.sh -b -p /opt/miniconda3 && \
    rm Miniconda3-py310_24.7.1-0-Linux-x86_64.sh && \
    /opt/miniconda3/bin/conda config --set auto_activate_base false && \
    /opt/miniconda3/bin/conda install -c conda-forge conda-bash-completion -y

# Final system configuration
RUN cp -r /usr/include/eigen3/* /usr/include/ && \
    apt-get update && apt-get upgrade -y && apt-get autoremove -y && apt-get autoclean -y && apt-get clean -y && \
    rm -rf /var/lib/apt/lists/* && \
    rosdep update && \
    git lfs install 2>/dev/null

# Copy entrypoint script
COPY ./entrypoint.sh /

# Set up the ROS entrypoint and default command
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]