# Cobot Autonomy Container

Complete development environment for cobot-autonomy stack on Intel NUC (no GPU).

## Remote access setup
- Add to local machine's `~/.ssh/config` allowing ssh-agent forwarding and X11 forwarding:
```
Host cobot-nuc
    HostName 10.0.0.106
    User amrl_user
    ForwardAgent yes
    ForwardX11 yes
    ForwardX11Trusted yes
```
Replace `10.0.0.106` and `amrl_user` with your actual cobot-nuc IP and username.

- Add to `~/.bashrc` on the remote machine:
```bash
export HOST_UID=$(id -u)
export XAUTHORITY=$HOME/.Xauthority
echo $DISPLAY > /tmp/.display_env_$HOST_UID
echo $SSH_AUTH_SOCK > /tmp/.ssh_auth_sock_$HOST_UID
```

## Quick Start
```bash
# Clone
git clone --branch cobot_nuc https://github.com/ut-amrl/spot-arch.git
cd spot-arch/container/

# Build container
./build.sh --rosv humble my-image

# Build and run container (only need to run once)
./container.sh --rosv humble --name my-container my-image

# Start and attach to container (once built)
docker start my-container && docker attach my-container
```

## Features
- **Multi-ROS Support**: ROS1 Noetic (Ubuntu 20.04) and ROS2 Humble/Foxy (Ubuntu 22.04)
- **Pre-configured Environment**: 
  - Miniconda3 Python 3.10 at `/opt/miniconda3` (auto-activation disabled)
  - ROS desktop-full + essential packages (twist-mux, velodyne-pointcloud, etc.)
  - Ceres Solver built from source
  - Development tools: CMake 3.26, Go 1.24.3, git-lfs, Qt5
- **Host Networking**: Full network, IPC, and PID namespace sharing for seamless ROS communication
- **ROS_IP Auto-detection**: Automatically sets ROS_IP from wg0 interface for network communication
- **X11 & SSH Forwarding**: GUI support and SSH agent forwarding for remote development
- **Volume Mounts**: host git/vim configs, X11 auth, SSH agent
- **Privileged Access**: Full hardware access for robotics development