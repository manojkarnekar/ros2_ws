#!/bin/bash
export DOCKER_COMPOSE_VERSION=1.29.2
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
nano \
curl \
libhdf5-dev \
libssl-dev \
python3 \
python3-pip


sudo usermod -aG docker $USER
newgrp docker

pip3 install -U pip
sudo pip3 install docker-compose=="${DOCKER_COMPOSE_VERSION}"
pip3 install docker-compose

sudo rm -rf /var/lib/apt/lists/* \
sudo apt-get clean


echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc
source ~/.bashrc

aws ecr get-login-password --region us-east-2 | docker login --username AWS --password-stdin 153479249734.dkr.ecr.us-east-2.amazonaws.com
docker pull 153479249734.dkr.ecr.us-east-2.amazonaws.com/sar:sar_ros2_micro_ros
docker pull 153479249734.dkr.ecr.us-east-2.amazonaws.com/sar:galactic.rplidarS2
docker pull 153479249734.dkr.ecr.us-east-2.amazonaws.com/sar:sar_ros2_rp_lidar
docker pull 153479249734.dkr.ecr.us-east-2.amazonaws.com/sar:galactic.base_control
docker pull 153479249734.dkr.ecr.us-east-2.amazonaws.com/sar:galactic

