#!/bin/bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

apt-cache policy | grep universe

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash

sudo apt-get install -y --no-install-recommends \
    ros-humble-teleop-twist-keyboard \
    ros-humble-joint-state-publisher  \
    ros-humble-robot-state-publisher  \
    ros-humble-slam-toolbox  \
    ros-humble-navigation2  \
    ros-humble-nav2-bringup  \
    ros-humble-tf-transformations  \
    ros-humble-robot-localization \
    ros-humble-turtlebot3* \
    ros-humble-controller-* \
    ros-humble-control-* \
    ros-humble-diff-drive-controller \
    ros-humble-diff-drive-controller-dbgsym \
    ros-humble-controller-manager \
    ros-humble-libg2o \
    ros-humble-xacro \
    ros-humble-gazebo-* \
    ros-humble-joint-* \
    python3-pip

# pip install -U colcon-common-extensions
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install python3-colcon-common-extensions


git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git; \
cd BehaviorTree.CPP;
mkdir build; \
cd build; \
cmake .. ;\
make ;\
sudo make install

echo "source /opt/ros/humble/setup.bash"  >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc
source ~/.bashrc


sudo apt-get update -y
sudo apt-get install -y --no-install-recommends \
ros-noetic-rosserial-arduino \
ros-noetic-rosserial \
ros-noetic-teleop-twist-keyboard \
ros-noetic-joint-state-publisher  \
ros-noetic-robot-state-publisher  \
ros-noetic-slam-toolbox \
ros-noetic-imu-tools \
ros-noetic-robot-localization \
ros-noetic-turtlebot3-.* \
ros-noetic-vision-opencv \
ros-noetic-move-base \
ros-noetic-costmap-converter \
ros-noetic-costmap-2d \
ros-noetic-navigation \
ros-noetic-teb-local-planner \
ros-noetic-slam-gmapping \
ros-noetic-openslam-gmapping \
ros-noetic-rosbridge-server \
ros-noetic-ros-tutorials \
ros-noetic-geometry-tutorials \
ros-noetic-rviz \
ros-noetic-rosbash \
ros-noetic-rqt-tf-tree --fix-missing
