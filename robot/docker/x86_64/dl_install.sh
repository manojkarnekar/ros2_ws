#!/bin/bash
sudo apt remove nvidia*
sudo apt purge nvidia*
ubuntu-drivers devices
# sudo apt-get install nvidia-kernel-open-515
sudo apt-get install nvidia-driver-515 # install the recomended one
# sudo prime-select nvidia
sudo reboot
# selecet Mkdir service
# https://linuxconfig.org/how-to-install-the-nvidia-drivers-on-ubuntu-22-04
lspci | grep -i nvidia
uname -m && cat /etc/*release
uname -r

https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_local
# wget https://developer.download.nvidia.com/compute/cuda/11.7.0/local_installers/cuda_11.7.0_515.43.04_linux.run
# sudo sh cuda_11.7.0_515.43.04_linux.run
# md5sum cuda_11.7.0_515.43.04_linux.run

# CUDA install
# FOR Ubuntu 20.04 LTS
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/11.7.1/local_installers/cuda-repo-ubuntu2004-11-7-local_11.7.1-515.65.01-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-11-7-local_11.7.1-515.65.01-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2004-11-7-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda
gedit .bashrc
export PATH=/usr/local/cuda-11.7/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.7/lib64:$LD_LIBRARY_PATH
source .bashrc



# FOR Ubuntu 22.04 LTS
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/11.7.0/local_installers/cuda-repo-ubuntu2204-11-7-local_11.7.0-515.43.04-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2204-11-7-local_11.7.0-515.43.04-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2204-11-7-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda
sudo apt-get install nvidia-gds
echo 'export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}' >> ~/.bashrc
# sudo apt install nvidia-cuda-toolkit
# https://linuxconfig.org/how-to-install-cuda-on-ubuntu-20-04-focal-fossa-linux

sudo apt-get install g++ freeglut3-dev build-essential libx11-dev \
    libxmu-dev libxi-dev libglu1-mesa libglu1-mesa-dev libfreeimage-dev

nvcc -V

# pytorch gpu install
pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu115

wget https://developer.nvidia.com/compute/machine-learning/tensorrt/secure/8.4.1/local_repos/nv-tensorrt-repo-ubuntu2004-cuda11.6-trt8.4.1.5-ga-20220604_1-1_amd64.deb

# cudnn install
# https://developer.nvidia.com/compute/cudnn/secure/8.4.1/local_installers/11.6/cudnn-local-repo-ubuntu2004-8.4.1.50_1.0-1_amd64.deb
# sudo dpkg -i cudnn-local-repo-ubuntu2004-8.4.1.50_1.0-1_amd64.deb
# sudo apt update
# sudo apt install libcudnn8
# sudo apt install libcudnn8-dev
# sudo apt install libcudnn8-samples
# sudo apt-get -y install nvidia-cudnn

# For removing nvidia
dpkg -l | grep nvidia
sudo apt purge nvidia-*
sudo apt-get remove nvidia*
sudo apt-get remove libnvidia*
sudo apt-get remove --purge nvidia-*
sudo apt-get autoremove
sudo apt autoremove
sudo apt autoclean
dpkg -l | grep -i nvidia
sudo apt-get update
sudo apt-get upgrade