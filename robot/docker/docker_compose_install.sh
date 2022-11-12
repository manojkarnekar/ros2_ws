#!/bin/bash
export DOCKER_COMPOSE_VERSION=1.29.2
sudo apt-get update -y
sudo apt-get upgrade -y
sudo usermod -aG docker $USER
newgrp docker
sudo apt-get install nano curl libhdf5-dev libssl-dev python3 python3-pip -y
pip3 install -U pip
sudo pip3 install docker-compose=="${DOCKER_COMPOSE_VERSION}"
pip3 install docker-compose
curl "https://awscli.amazonaws.com/awscli-exe-linux-aarch64.zip" -o "awscliv2.zip"
unzip awscliv2.zip
sudo ./aws/install
# ********************
# https://collabnix.com/getting-started-with-docker-and-ai-on-nvidia-jetson-agx-xavier-developer-platform/
# https://collabnix.com/getting-started-with-docker-and-ai-on-nvidia-jetson-agx-xavier-developer-platform/
# {
#     "runtimes": {
#         "nvidia": {
#             "path": "/usr/bin/nvidia-container-runtime",
#             "runtimeArgs": []
#         }
#     }
# }
# Nvidia docker install for jetson devices
# curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
# curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
# sudo apt-get update

# # Install nvidia-docker2 and reload the Docker daemon configuration
# sudo apt-get install -y nvidia-docker2
# sudo pkill -SIGHUP dockerd 
# ***************************

# **********************************************************
#nvidia docker install official
# curl https://get.docker.com | sh && sudo systemctl --now enable docker
# distribution=$(. /etc/os-release;echo $ID$VERSION_ID) && curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo apt-key add - && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
# sudo apt-get update
# sudo apt-get install -y nvidia-docker2
# sudo systemctl restart docker
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

# ***************************************************************


sudo service docker status
step->1 please, take a pull of port_yolo_to_foxy branch 

step →2 sudo apt-get install nvidia-container-runtime

install nano (sudo apt-get install nano)

step → 3 sudo nano /etc/docker/daemon.json

{
"runtimes": {
"nvidia": {
"path": "nvidia-container-runtime",
"runtimeArgs": []
}
},
"default-runtime": "nvidia"
}

save it

step → 4 sudo systemctl restart docker

# docker build -f Dockerfile.ros2.nav2_tune -t 153479249734.dkr.ecr.us-east-2.amazonaws.com/sar:nav2_tune .
# docker stats --no-stream | cat >> ./`date -u +"%F_%H-%M-%S.csv"`
# docker stats --format "table {{.Name}},{{.CPUPerc}},{{.MemPerc}},{{.MemUsage}}" | tee --append docker_stats.csv
# scp dev@192.168.0.114:/home/dev/docker_stats.csv /home/debanik/sar/docker/arm64/jetson_log/