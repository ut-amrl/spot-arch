# AMRL Spot Docker Images
All images have the following by default:
* git lfs
* miniconda (at `/opt/miniconda3`). **Use `conda-create` instead of conda create** (its a wrapper, it sets some torch paths properly if you want to use a different version of torch in your conda env)
* conda bash completion (so can use TAB for conda commands)

## Available Images
- spot-ros1-jp5
    * ubuntu focal
    * jetpack 5.1.2
    * CUDA 11.4 (though GPU acceleration _NOT_ supported on jetpack 6 for jp5 containers)
    * ROS1 noetic
    * torch 2.1, libtorch, torchvision, opencv
- spot-ros1-jp5-scratch **(legacy)**
    * ubuntu focal
    * jetpack 5.1.2
    * CUDA 11.4 (though GPU acceleration _NOT_ supported on jetpack 6 for jp5 containers)
    * ROS1 noetic
    * torch 2.1, libtorch, torchvision, opencv (built from source instead of using nvidia provided images)
- spot-ros1-jp6 **(coming soon)**
    * ubuntu jammy
    * jetpack 6.0
    * CUDA 12.4
    * ROS1 noetic built from source
    * torch ??, libtorch, torchvision, opencv
- spot-ros2-jp6 **(coming soon)**
    * ubuntu jammy
    * jetpack 6.0
    * CUDA 12.4
    * ROS2 humble
    * torch ??, libtorch, torchvision, opencv