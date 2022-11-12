#!/bin/bash
sudo apt-get update 
sudo apt-get install -y --no-install-recommends \
    ros-galactic-teleop-twist-keyboard \
    ros-galactic-joint-state-publisher  \
    ros-galactic-robot-state-publisher  \
    ros-galactic-slam-toolbox  \
    ros-galactic-navigation2  \
    ros-galactic-nav2-bringup  \
    ros-galactic-tf-transformations  \
    ros-galactic-robot-localization \
    ros-galactic-turtlebot3-.* \
    ros-galactic-controller-* \
    ros-galactic-control-* \
    ros-galactic-diff-drive-controller \
    ros-galactic-gazebo-* \
    ros-galactic-diff-drive-controller-dbgsym \
    ros-galactic-controller-manager \
    ros-galactic-libg2o \
    ros-galactic-xacro


sudo rm -rf /var/lib/apt/lists/*
sudo apt-get clean

echo "source /opt/ros/galactic/setup.bash"  >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/galactic/share/turtlebot3_gazebo/models" >> ~/.bashrc
source ~/.bashrc
