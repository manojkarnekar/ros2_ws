#!/bin/bash

if [[ $EUID -ne 0 ]]; then
   echo "You must be root to run this script." 1>&2
   exit 100
fi

# file="/proc/meminfo"     #the file where you keep your string name

# read -d $'\x04' MemTotal < "$file" #the content of $file is redirected to stdin from where it is read out into the $name variable

# echo MemTotal 

# if [[$MemTotal > 4000000]]; then
#    su - root
#    sudo dd if=/dev/zero of=/swapfile bs=1024 count=1M
#    sudo chmod 600 /swapfile
#    sudo mkswap /swapfile
#    sudo swapon /swapfile
# fi

echo "==================="
echo "Setting Up System"
echo "==================="
apt update
apt upgrade -y

apt install -y git
apt install -y python-pip
apt install -y python3
apt install -y python3-pip

adduser dev
useradd dev --create-home --password "$(openssl passwd -1 "irobot")" --shell /bin/bash --uid 5012 --user-group
adduser dev dialout  
adduser dev video  
adduser dev sudo

echo "==================="
echo "Setting Up Dependencies"
echo "==================="
apt install -y libbullet-dev
apt install -y libeigen3-dev
apt install -y libsdl-image1.2-dev
apt install -y libyaml-cpp-dev
#apt install -y libsdl-dev

pip install pyserial
pip install twisted
pip install pyOpenSSL
pip install autobahn
pip install tornado
pip install bson
pip install service_identity
pip install pymongo

echo "==================="
echo "Setting Up Arduino"
echo "==================="
mkdir -p /home/dev/arduino-cli/bin
apt install curl
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/home/dev/arduino-cli/bin sh
echo 'export PATH="$PATH:$HOME/arduino-cli/bin"' >> /home/dev/.bashrc
/home/dev/arduino-cli/bin/arduino-cli core install arduino:avr
#apt install -y arduino
#apt install -y arduino-mk

echo "==================="
echo "Setting Up ADB"
echo "==================="
#apt install -y adb
#apt install android-studio
#apt install android-libcutils=1:7.0.0+r33-2 android-liblog=1:7.0.0+r33-2 android-libbase=1:7.0.0+r33-2 android-libadb=1:7.0.0+r33-2 adb=1:7.0.0+r33-2

echo "==================="
echo "Setting Up ROS"
echo "==================="
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt update
apt install ros-melodic-ros-base python-rosinstall python-rosinstall-generator python-wstool build-essential ros-melodic-orocos-kdl
apt install python-rosdep
rosdep init
rosdep update

su - dev -c "rosdep init"
su - dev -c "rosdep update"
echo "source /opt/ros/melodic/setup.bash" >> /home/dev/.bashrc
echo "source /home/dev/sar/catkin_ws/devel/setup.bash" >> /home/dev/.bashrc
echo 'export ROS_MASTER_URI="http://192.168.10.201"' >> /home/dev/.bashrc
echo 'export ROS_HOSTNAME=192.168.10.201' >> /home/dev/.bashrc
echo 'export PATH="$PATH:/home/dev/arduino-cli/bin"' >> /home/dev/.bashrc


echo "==================="
echo "Setting Up Node"
echo "==================="
sudo apt-get install curlm
curl -sL https://deb.nodesource.com/setup_13.x | sudo bash -
sudo apt-get install -y nodejs

echo "======================"
echo "Setting Up SAR in dev"
echo "======================"
cd /home/dev/ 
git clone -b afs https://git-codecommit.us-east-2.amazonaws.com/v1/repos/sar

echo "==================="
echo "Installing Screen"
echo "==================="
sudo apt-get install screen

echo "==================="
echo "DONE"
echo "==================="

source sar/utilities/scripts/setup.sh
/bin/bash sar/utilities/scripts/setup.sh