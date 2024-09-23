"""
setup script for running codebotler on AMRL Spot AGX orin (jetpack 6.0)
prereqs:
- spot_autonomy stack setup complete
- conda installed
- inside a jetpack 5.1.2 container (CUDA acceleration will not be available)
"""

#!/bin/sh
RED='\033[0;31m'
NC='\033[0m' # No Color

# Add to current shell
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/codebotler/codebotler
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/codebotler/codebotler_amrl_impl

# Add to ~/.bashrc
echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/codebotler/codebotler' >> ~/.bashrc
echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ut-amrl/codebotler/codebotler_amrl_impl' >> ~/.bashrc

echo -e "${RED}Cloning codebotler repos...${NC}"
cd ~/ut-amrl
mkdir -p codebotler && cd codebotler
git clone --recursive --branch spot git@github.com:ut-amrl/codebotler.git
git clone --recursive --branch spot2 git@github.com:ut-amrl/codebotler_amrl_impl.git

# Initialize conda in the script
eval "$(/opt/miniconda3/bin/conda shell.bash hook)"

echo -e "${RED}Setting up codebotler conda environment...${NC}"
# codebotler env setup
cd ~/ut-amrl/codebotler/codebotler
conda-create -n codebotler python=3.8 -y
conda activate codebotler
wget -O torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
pip3 install 'Cython<3'
pip3 install numpy torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
pip3 install scipy
rm torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
python3 -c "import torch; print(torch.__version__)"

cd ~
git clone --branch release/0.16 https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=0.16.1
python3 setup.py install --user
cd ~
rm -rf torchvision
python3 -c "import torchvision; print(torchvision.__version__)"

echo -e "${RED}Setting up codebotler dependencies...${NC}"
cd ~/ut-amrl/codebotler/codebotler
pip3 install -r requirements.txt

cd ~/ut-amrl/codebotler/codebotler_amrl_impl
conda deactivate
conda activate codebotler
./setup.sh

cd ~/ut-amrl/codebotler/codebotler
cd robot_interface
conda deactivate
conda activate codebotler
./setup_robot.sh
echo "Add source ~/ut-amrl/codebotler/codebotler/robot_interface/devel/setup.bash to your .bashrc"