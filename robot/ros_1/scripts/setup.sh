#!/bin/bash

cd $(dirname $0)/../..

DIRPATH=`pwd`

dbpath=$DIRPATH"/db/sar.db"
createDB=true
samples=true
maps=true
packages=true
cronjob=true

while [ "$1" != "" ]; do
    case $1 in
        -d | --no-database )   createDB=false
                               ;;
        -s | --no-samples )    samples=false
                               ;;
        -m | --no-maps )       maps=false
                               ;;
        -p | --no-packages )   packages=false
                               ;;
        -c | --no-cron )       cronjob=false
                               ;;
    esac
    shift
done

echo "Setting up Directories.."
mkdir -p logs

if [ $createDB = true ]; then
    if [ ! -f "$dbpath" ]; then
        echo "Creating DB.."
        sqlite3 $dbpath < db/setup.sql
        sqlite3 $dbpath < db/const.sql
        echo "Done."
        if [ $samples = true ]; then
            echo "Adding Samples to DB.."
            sqlite3 $dbpath < db/sample.sql
            echo "Done."
        fi
    else
        echo "Error: DB Already Exists."

    fi
fi

if [ $maps = true ]; then
    cd $DIRPATH"/maps"
    echo "Downloading Maps.."
    spath="https://jetbrain.ai/files/"
    map1="office.pgm"
    map2="top_floor.pgm"
    if [ ! -f "$map1" ]; then
        wget "$spath$map1"
    fi
    if [ ! -f "$map2" ]; then
        wget "$spath$map2"
    fi
    cd ..
    echo "Done."
fi

if [ $packages = true ]; then

    echo "Setting up Directories.."
    mkdir -p $DIRPATH"/logs"
    mkdir -p $DIRPATH"/catkin_ws/src"

    cd $DIRPATH"/catkin_ws/src"
    echo "Cloning Packages.."
    git clone -b melodic-devel https://github.com/ros/geometry2.git
    git clone https://github.com/loxxy/micvision.git
    git clone https://github.com/ros-planning/navigation.git
    git clone https://github.com/DLu/navigation_layers.git
    git clone https://github.com/ros-planning/navigation_msgs.git
    git clone https://github.com/ros-perception/slam_gmapping.git
    git clone -b melodic-devel https://github.com/ros-drivers/rosserial.git
    git clone https://github.com/robopeak/rplidar_ros.git
    git clone https://github.com/ros-teleop/teleop_twist_keyboard.git
    git clone https://github.com/ros/angles.git
    git clone https://github.com/ros/diagnostics.git
    git clone -b melodic-devel https://github.com/ros/geometry.git
    git clone https://github.com/ros/joint_state_publisher.git
    git clone https://github.com/ros-perception/laser_geometry.git
    git clone https://github.com/ros-visualization/python_qt_binding.git
    git clone https://github.com/ros/robot_state_publisher.git
    git clone https://github.com/ros/roslint.git
    git clone https://github.com/rst-tu-dortmund/teb_local_planner.git
    git clone https://github.com/ros/xacro.git
    git clone https://github.com/ros/kdl_parser.git
    git clone https://github.com/GT-RAIL/rosauth.git
    git clone https://github.com/RobotWebTools/rosbridge_suite.git
    git clone https://github.com/ros-perception/openslam_gmapping.git
    #git clone https://github.com/ros-gbp/bfl-release.git
    #git clone https://github.com/wg-perception/people.git
    #git clone https://github.com/DLu/wu_ros_tools.git
    #git clone https://github.com/ros-perception/vision_opencv.git

    echo "==================="
    echo "Allowing permissions"
    echo "==================="
    chown -R dev $DIRPATH"/sar/catkin_ws"
    apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essent$
    apt-get install python-pip
    apt install python-rosdep
    rosdep init
    rosdep update
    rosdep fix-permissions
    rosdep update

    echo "==================="
    echo "Installing ROSserial"
    echo "==================="
    sudo apt-get install ros-melodic-rosserial-arduino
    sudo apt-get install ros-melodic-rosserial
    rosrun rosserial_arduino make_libraries.py .
    #sudo mv /home/dev/ros_lib /home/dev/sar/arduino/lib/ros_lib

    # echo "==================="
    # echo "Set UDEV Rules"
    # echo "==================="
    # cat <<EOF >/etc/udev/rules.d/arduino.rules
    # KERNEL=="ttyUSB*", ATTRS{idVendor}=="1d6b", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="arduino"
    # EOF

    # cat <<EOF > /etc/udev/rules.d/barcode.rules
    # KERNEL=="event*", ATTRS{idVendor}=="24ea", ATTRS{idProduct}=="0197", MODE:="0777", SYMLINK+="barcode"
    # EOF

    # cat <<EOF > /etc/udev/rules.d/rplidar.rules
    # KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"
    # EOF

    # cat <<EOF > /etc/udev/rules.d/battery.rules 
    # KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", MODE:="0777", SYMLINK+="rplidar"
    # EOF

    # sudo udevadm trigger
    
    if [ $cronjob = true ]; then
       echo "Installing Cron Job.."
       (crontab -l ; echo "@reboot $DIRPATH/utilities/scripts/run.sh") | sort - | uniq - | crontab -
    fi
    echo "Done."
fi