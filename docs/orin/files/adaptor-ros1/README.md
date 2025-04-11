# Build instructions

1. Build the base image and test that PyTorch works inside it.
```bash
docker build -f Dockerfile_base -t spot_base .
docker run --name spot_base -d --runtime=nvidia --cgroupns=host -it --net=host spot_base sleep infinity
```

2. Build the container with ROS Noetic in it:
```bash
docker build -f Dockerfile_jp6 -t ros-noetic:adaptor .
```

3. Install the AMRL spot-autonomy stack: 
```bash
docker build -f Dockerfile_spot -t spot-autonomy:adaptor .
```