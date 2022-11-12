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
    ros-galactic-gazebo-ros2-control \
    ros-galactic-joint-* \
    ros-galactic-gazebo-* \
    ros-galactic-controller-* \
    ros-galactic-control-* \
    ros-galactic-diff-drive-controller \
    ros-galactic-diff-drive-controller-dbgsym

sudo rm -rf /var/lib/apt/lists/*
sudo apt-get clean