#FROM arm64v8/ubuntu:focal
FROM nvcr.io/nvidia/l4t-tensorrt:r8.5.2-runtime
#FROM nvcr.io/nvidia/l4t-base:35.4.1

ENV DEBIAN_FRONTEND=noninteractive

# Set up timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y apt-utils && \
    apt-get install -q -y --no-install-recommends tzdata dirmngr gnupg2 build-essential && \
    rm -rf /var/lib/apt/lists/*

# Install development tools that are missing in the runtime image
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cuda-toolkit-11-4 \
    libcudnn8-dev \
    libnvinfer-dev \
    libnvinfer-plugin-dev \
    && rm -rf /var/lib/apt/lists/*

# Verify nvcc installation
RUN nvcc --version

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

# Set environment variables for ROS
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic

# Install ROS desktop-full packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Install necessary tools and ROS core
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.8 \
    python3.8-dev \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstool \
    python-is-python3 \
    python3-catkin-tools \
    python3-tk \
    python3-wstool \
    && rm -rf /var/lib/apt/lists/*

# Copy entrypoint script
COPY ./ros_entrypoint.sh /

# Set up the ROS entrypoint and default command
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

#########################################################################################################################################
## Build from source torch and libtorch
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopenblas-base \
    libopenmpi-dev \
    libomp-dev \
    libjpeg-dev \
    zlib1g-dev \
    libpython3-dev \
    libopenblas-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    && rm -rf /var/lib/apt/lists/*

# Install necessary dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    libssl-dev \
    curl \
    qt5-default \
    wget \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root
RUN mkdir -p cmake-3.25.1/Src
WORKDIR /root/cmake-3.25.1/Src
RUN wget -c --show-progress https://github.com/Kitware/CMake/releases/download/v3.25.1/cmake-3.25.1.tar.gz \
    && tar xvf cmake-3.25.1.tar.gz
WORKDIR /root/cmake-3.25.1

RUN mkdir -p cmake-3.25.1-build
WORKDIR /root/cmake-3.25.1/cmake-3.25.1-build
RUN cmake -DBUILD_QtDialog=ON -DQT_QMAKE_EXECUTABLE=/usr/lib/qt5/bin/qmake /root/cmake-3.25.1/Src/cmake-3.25.1 \
    && make -j$(nproc) \
    && make install

# Set the PATH to prioritize /usr/local/bin
ENV PATH="/usr/local/bin:$PATH"

# Verify CMake version
RUN cmake --version

WORKDIR /root
RUN git clone --recursive --branch 2.1patch_sada https://github.com/sadanand1120/pytorch.git
WORKDIR /root/pytorch

# Set environment variables for building PyTorch
ENV USE_NCCL=0 \
    USE_DISTRIBUTED=0 \
    USE_QNNPACK=0 \
    USE_PYTORCH_QNNPACK=0 \
    TORCH_CUDA_ARCH_LIST="7.2;8.7" \
    PYTORCH_BUILD_VERSION=2.1.0 \
    PYTORCH_BUILD_NUMBER=1

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    cmake \
    libopenblas-dev \
    libopenmpi-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages required for PyTorch build
RUN pip3 install -r requirements.txt \
    && pip3 install scikit-build \
    && pip3 install ninja

# Build PyTorch wheel
RUN python3 setup.py bdist_wheel

# Install
# Install necessary system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    libopenblas-base \
    libopenmpi-dev \
    libomp-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install 'Cython<3' \
    && pip3 install numpy \
    && pip3 install dist/*.whl

# torchvision
# Install additional dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libjpeg-dev \
    zlib1g-dev \
    libpython3-dev \
    libopenblas-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    && rm -rf /var/lib/apt/lists/*

# Set environment variable to force CUDA usage
ENV FORCE_CUDA=1

WORKDIR /root
RUN git clone --branch release/0.16 https://github.com/pytorch/vision torchvision
WORKDIR /root/torchvision
ENV BUILD_VERSION=0.16.1
RUN python3 setup.py install --user
WORKDIR /root

ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/python3.8/dist-packages/torch
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/python3.8/dist-packages/torch/lib
ENV Torch_DIR=$Torch_DIR:/usr/local/lib/python3.8/dist-packages/torch/share/cmake/Torch

#########################################################################################################################################

# Set environment variables for NVIDIA on Jetson AGX Orin
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
LABEL com.nvidia.volumes.needed="nvidia_driver"

# Set non-interactive installation and timezone
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Chicago

# Install package installation tools, development tools, and additional utilities
RUN apt-get update && apt-get install -y \
    apt-utils \
    curl \
    software-properties-common \
    build-essential \
    clang-12 \
    clang-format \
    cmake \
    g++ \
    gdb \
    git \
    nano \
    valgrind \
    vim \
    iputils-ping \
    less \
    mesa-utils \
    net-tools \
    rsync \
    tmux \
    tree \
    unzip \
    usbutils \
    zip \
    zsh \
    && rm -rf /var/lib/apt/lists/*

# Update and upgrade system packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get autoremove -y && apt-get autoclean -y && apt-get clean -y

# Install Linux headers, build tools, compilers, development tools, and utilities
RUN apt-get install -y \
    build-essential \
    gcc \
    g++ \
    make \
    git \
    vim \
    emacs \
    curl \
    wget \
    gdb \
    cmake \
    htop \
    tmux \
    screen \
    nano \
    mesa-utils \
    ppa-purge \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install virtualenv
RUN pip3 install jetson-stats

# Set CUDA environment variables
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${PATH}:${CUDA_HOME}/bin
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${CUDA_HOME}/lib64

# Update and upgrade system packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get autoremove -y && apt-get autoclean -y && apt-get clean -y

# Install FreeImage libraries, OpenCV, ROS dependencies, and additional utilities
RUN apt-get update && apt-get install -y \
    libfreeimage3 \
    libfreeimage-dev \
    libopenblas-dev \
    net-tools \
    build-essential \
    ros-noetic-ros-numpy \
    && rm -rf /var/lib/apt/lists/*

# Install Git Large File Storage (git-lfs)
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash && \
    apt-get install -y git-lfs

# Install rospkg via pip
RUN pip3 install rospkg

# Install basic tools, dependencies for ROS development, development tools, audio-related tools, and additional utilities
RUN apt-get update && apt-get install -y \
    apt-utils \
    curl \
    software-properties-common \
    build-essential \
    clang-12 \
    clang-format \
    cmake \
    g++ \
    gdb \
    git \
    nano \
    valgrind \
    vim \
    iputils-ping \
    less \
    mesa-utils \
    net-tools \
    rsync \
    tmux \
    tree \
    unzip \
    usbutils \
    zip \
    zsh \
    joystick \
    ffmpeg \
    espeak \
    mpg123 \
    && rm -rf /var/lib/apt/lists/*

# not needed as forwarding audio from host
#RUN apt-get update && apt-get install -y \
#    alsa-base alsa-utils pulseaudio pavucontrol \
#    && rm -rf /var/lib/apt/lists/*

# Update and upgrade system packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get autoremove -y && apt-get autoclean -y && apt-get clean -y

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
    pip3 install cython empy bosdyn-client==3.1.1 bosdyn-mission==3.1.1 bosdyn-api==3.1.1 bosdyn-core==3.1.1 \
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

# Update and upgrade system packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get autoremove -y && apt-get autoclean -y && apt-get clean -y

# Add Microsoft repository and install Kinect SDK
RUN curl https://packages.microsoft.com/keys/microsoft.asc | apt-key add - && \
    apt-add-repository "deb https://packages.microsoft.com/ubuntu/18.04/multiarch/prod bionic main" && \
    apt-get update && ACCEPT_EULA=Y apt-get install -y k4a-tools libk4a1.4 libk4a1.4-dev

# Add Kinect udev rules
COPY ./99-k4a.rules /etc/udev/rules.d/99-k4a.rules
#RUN udevadm control --reload-rules && udevadm trigger
#RUN wget https://raw.githubusercontent.com/ut-amrl/k4a_ros/master/99-k4a.rules -O /etc/udev/rules.d/99-k4a.rules && \
#    udevadm control --reload-rules && udevadm trigger

# Install dependencies for spot_autonomy
RUN apt-get update && apt-get install -y \
    qt5-default \
    libqt5websockets5-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    && rm -rf /var/lib/apt/lists/*
    
RUN cp -r /usr/include/eigen3/* /usr/include/
