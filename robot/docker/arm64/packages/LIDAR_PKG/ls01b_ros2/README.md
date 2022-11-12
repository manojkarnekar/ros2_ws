# ls01b_ros2

## version track
Author: yao

### ver2.01_201014 leo
1. Add angle compensate
2. Add truncate angle area data

## Description
The `ls01b_ros2` package is a linux ROS driver for ls01b.
The package is tested on Ubuntu 20.04 with ROS foxy.

## Compling
mkdir -p ~/leishen_ws/src
cd ~/leishen_ws/src
tar -xvf LSLIDAR_LS01B_v1.0.1_201014_ROSF.tar.gz
cd ~/leishen_ws
colcon build
source install/setup.bash
sudo chmod 777 /dev/ttyUSB0  	##Open serial port permission，The serial device may be /dev/ttyUSB1
ros2 launch ls01b_v2 ls01b_v2_launch.py








