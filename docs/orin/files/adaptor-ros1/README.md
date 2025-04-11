# Build instructions

1. Build the base image and test that PyTorch works inside it.
```bash
docker build -f Dockerfile_base -t spot_base .
docker run --name spot_base -d --runtime=nvidia --cgroupns=host -it --net=host spot_base sleep infinity
```

