#!/bin/bash

# This is the main file running all the robotics code and the server communication with the app. 
# For example: To run the robot
# ./utilities/scripts/run.sh -module () -motor () -system () -mode ()

# Under every module there are multiple screens running. A description of each screen running is provided below.


module=""
motorType=""
system="XU4"
mode=""
simulation=""
sleepdelay=-1
localization=1

while [ "$1" != "" ]; do
    case $1 in

        -simulation | --simulation-type )   shift
                                            simulation=$1
                                            ;;
        # -motor 
        # There are 4 different motor types with different gear ratios: The grear ratio gives us th informatin about the Ticks_meter, Motor_direction and Twist_factor, Base_width. 
        # These params are specified at --> /sar/config/motors/gear_ratio.sh
        # Gear ratio 50 is used in all the bots except AFS and for the robots with the onboard computer Odroid XU4.
        # Gear ratio 15 is specifically for Odroid XU4.
        # Gear ratio 18 is only for AFS.
        # A separate gear ratio is configured for airbot with the file name airbit.sh.
        
        -motor | --motor-config )           shift
                                            motorType=$1
                                            ;;
        # -module
        # This refers to the bot on which we are working with. 
        # These are the modules:
        # Amro, Airbot, Core-0 arm, AFS, Hotbot (CORE-0)

        -module | --module-name )           shift
                                            module=$1
                                            ;;
        # -system
        # System refers to the ccomputer in the bot i.e. odroid N2, odroid XU4 or jetson Nano, Jetson Xavier.                                   
       
        -system | --controller-type )       shift
                                            system=$1
                                            ;;
        # -mode
        # There are different modes used in the file.
           # --> AM- autonomous mapping - This is to be used when bot has to be operated autonomously. This can be used to build a map using explore_lite.
           # --> M- manual mapping - This is to be used when we need to build the map manually using teleop node. this mode is common to for all the robots.
           # --> Apart from this there is a mode N specifically for AFS to run it autonomously.
        
        -mode | --map-mode )                shift
                                            mode=$1
                                            ;;
        # -map
         # This variable provides the path of the map for the robot. Tha map is for each bot will be provided in the --> /sar/models/modules/robot.sh.
        
        -map | --map-path )                 shift
                                            map=$1
                                            ;;
        # -sleepdelay
        # This variable adds a delay while running the run.sh script. This allows to configure the ROS_ip before starting the roscore because if the ip is not configured
        # then roscore will not run. 
        
        -sleep | --sleep-delay )            shift
                                            sleepdelay=$1
                                            ;;
        # -loc
        # This is added to run the micvsion package for initial localization when we run the robot in navigation mode. This parameter will automatically run the service.
        
        -loc | --localization )             shift
                                            localization=$1
                                            ;;
    esac
    shift
done

if [ $sleepdelay -ne -1 ]; then
    sleep $sleepdelay
fi

if [ -z "$CURRENT_IP_ADD" ]; then
    if [ ! -z `command -v ifconfig` ]; then
        export CURRENT_IP_ADD=`ifconfig eth0 | grep -E -o "([0-9]{1,3}[\.]){3}[0-9]{1,3}" | head -n 1`
    elif [ ! -z `command -v ip` ]; then
        export CURRENT_IP_ADD=`ip -4 -o a | grep eth0 | grep -E -o "([0-9]{1,3}[\.]){3}[0-9]{1,3}" | head -n 1`
    else
        export CURRENT_IP_ADD="192.168.10.201"
    fi
fi

if [ -z "$ROS_MASTER_URI" ]; then
    source /opt/ros/melodic/setup.bash
    source /home/dev/sar/ros_1/catkin_ws/devel/setup.bash
    export ROS_MASTER_URI="http://192.168.10.201:11311"
    export ROS_HOSTNAME=192.168.10.201
fi

cd $(dirname $0)/../..

DIR=`pwd`
PARAMS_DIR_PATH=$DIR"/config/params/params_"$system
#SAR_DEV_PATH="~/sar-dev/catkin_ws/src"

if [ "$simulation" = "true" ]; then
    source $DIR"/utilities/config/modules/gazebo.sh"
    screen -dmS sar.gazebo sh -c "roslaunch $SAR_DEV_PATH/amro/launch/amro.launch; exec bash"
    screen -dmS sar.state_machine sh -c "python3 control/statemachine.py; exec bash"
    screen -dmS sar.server sh -c "nodemon server/server.js; exec bash"
    screen -dmS sar.merged_laser sh -c "python3 control/Mul_sonar_to_laser.py; exec bash"
    screen -ls

# sourcing the configuration for the gear_ratio 
else
    if [ "$motorType" = "B" ]; then
        source $DIR"/config/motors/gear_ratio_15.sh"
# While running the airbot this particular motor configuration needs to be used.
    elif [ "$motorType" = "airbot" ]; then
        source $DIR"/config/motors/airbot.sh"
# While running the AFS this particular motor configuration needs to be used.
    elif [ "$motorType" = "afs" ]; then
	      source $DIR"/config/motors/gear_ratio_18.sh"
# This is the default motor configuration used in the new version of CORE-0.
    else
        source $DIR"/config/motors/gear_ratio_50.sh"
    fi

    
    screen -dmS sar.core sh -c "roscore; exec bash"
    echo "Waiting for ROS Server.."
    until rostopic list 2> /dev/null ; do echo "."; sleep 1; done


    # This refers to the medicine transporation robot mounted on CORE_0.
    if [ "$module" = "amro" ]; then
        source $DIR"/models/modules/amro.sh"

    # This refers to the disinfection robot mounted on CORE_0.
    elif [ "$module" = "hotbot" ]; then
        source $DIR"/models/modules/hotbot.sh"

    # This refers to the disinfection robot for airplanes.
    elif [ "$module" = "airbot" ]; then
        source $DIR"/models/modules/airbot.sh"

    # This refers to the autonomous floor cleaning robot.    
    elif [ "$module" = "afs" ]; then
        source $DIR"/models/modules/afs.sh"

    # This refers to the robotic arm mounted on CORE_0.
    elif [ "$module" = "core0_arm" ]; then
        source $DIR"/models/modules/core0_arm.sh"

    # Default configuration refers to CORE_0 (autonomous base).
    elif [ -z "$module" ]; then
        echo "Module argument is missing. Default configurations will run.."
        source $DIR"/models/modules/default.sh"
    fi

    # This section is specifically for the AMRO. This shows the screens that run for AMRO.

    if [ "$module" = "amro" ]; then

        rosparam set /robot_description "`python $DIR"/ros_1/control/xacro.py" $XACRO_PATH`"
        rosparam set /ticks_meter $TICKS_METER
        rosparam set /motor_direction $MOTOR_DIRECTION
        rosparam set /twist_factor $TWIST_FACTOR

        screen -dmS sar.m_sonar sh -c "python2 $DIR/ros_1/control/m_sonar.py; exec bash"
        screen -dmS sar.robot_state_publisher sh -c "rosrun robot_state_publisher robot_state_publisher; exec bash"
        screen -dmS sar.joint_state_publisher sh -c "rosrun joint_state_publisher joint_state_publisher; exec bash"
        screen -dmS sar.motor_control sh -c "python2 $DIR/ros_1/control/motor_control.py; exec bash"
      	screen -dmS sar.diff_tf sh -c "python2 $DIR/ros_1/control/diff_tf.py; exec bash"
        screen -dmS sar.arduino sh -c "rosrun rosserial_python serial_node.py _port:=/dev/arduino _baud:=115200; exec bash"
        screen -dmS sar.imu sh -c "rosrun rosserial_python serial_node.py _port:=/dev/imu _baud:=115200 __name:=node_imu; exec bash"
        screen -dmS sar.bridge sh -c "roslaunch rosbridge_server rosbridge_websocket.launch; exec bash"
        screen -dmS sar.server sh -c "sleep 40; nodemon server/server.js; exec bash"
        screen -dmS sar.lidar sh -c "roslaunch rplidar_ros rplidar.launch; exec bash"
        screen -dmS sar.move_base sh -c "rosrun move_base move_base __name:=move_base _base_local_planner:=teb_local_planner/TebLocalPlannerROS ; exec bash"
        screen -dmS sar.sonar sh -c "python2 $DIR/ros_1/control/sonar.py; exec bash"

        screen -dmS sar.state_machine sh -c "python3 control/statemachine.py; exec bash"
        screen -dmS sar.amcl sh -c "roslaunch $PARAMS_DIR_PATH/amcl.launch.xml; exec bash"
        screen -dmS sar.localisation sh -c "rosrun micvision micvision_localization_node _quick_score:=False; exec bash"
        screen -dmS sar.map sh -c "rosrun map_server map_server $MAP_PATH; exec bash"
        screen -dmS sar.rotation_degree sh -c "python2 control/rotation_degree.py; exec bash"

        # This section specifies the different mapping modes with the screens running under each mode.
        # Autonomous mapping mode
        if [ "$mode" = "AM" ]; then
            PARAMS_DIR_PATH=$PARAMS_DIR_PATH"_"$mode
            screen -dmS sar.core_gmapping sh -c "rosrun gmapping slam_gmapping _xmin:=-25 _ymin=-25 _xmax:=25 _ymax:=25; exec bash"
            screen -dmS sar.explorer sh -c "roslaunch explore_lite explore.launch; exec bash"
        # Mapping mode
        elif [ "$mode" = "M" ]; then
            screen -dmS sar.core_gmapping sh -c "rosrun gmapping slam_gmapping _xmin:=-25 _ymin=-25 _xmax:=25 _ymax:=25; exec bash"
        # Default mode - Navigation
        elif [ -z "$mode" ]; then
          	rosparam load $PARAMS_DIR_PATH/costmap_common_params.yaml /move_base/global_costmap
          	rosparam load $PARAMS_DIR_PATH/costmap_common_params.yaml /move_base/local_costmap
          	rosparam load $PARAMS_DIR_PATH/local_costmap_params.yaml /move_base
        fi

        screen -ls

    fi

    # This section is specifically for the HOTBOT. This shows the screens that run for HOTBOT.

    if [ "$module" = "hotbot" ]; then

        rosparam set /robot_description "`python $DIR"/ros_1/control/xacro.py" $XACRO_PATH`"
        rosparam set /ticks_meter $TICKS_METER
        rosparam set /motor_direction $MOTOR_DIRECTION
        rosparam set /twist_factor $TWIST_FACTOR
        rosparam set /base_width $BASE_WIDTH

        screen -dmS sar.robot_state_publisher sh -c "rosrun robot_state_publisher robot_state_publisher; exec bash"
        screen -dmS sar.joint_state_publisher sh -c "rosrun joint_state_publisher joint_state_publisher; exec bash"
        screen -dmS sar.motor_control sh -c "python2 $DIR/ros_1/control/motor_control.py; exec bash"
      	screen -dmS sar.diff_tf sh -c "python2 $DIR/ros_1/control/diff_tf.py; exec bash"
        screen -dmS sar.arduino sh -c "rosrun rosserial_python serial_node.py _port:=/dev/arduino _baud:=115200; exec bash"
        screen -dmS sar.imu sh -c "rosrun rosserial_python serial_node.py _port:=/dev/imu _baud:=115200 __name:=node_imu; exec bash"
        screen -dmS sar.bridge sh -c "roslaunch rosbridge_server rosbridge_websocket.launch; exec bash"
        screen -dmS sar.server sh -c "sleep 40; nodemon server/server.js; exec bash"
        screen -dmS sar.lidar sh -c "roslaunch rplidar_ros rplidar.launch; exec bash"
        screen -dmS sar.move_base sh -c "rosrun move_base move_base __name:=move_base _base_local_planner:=teb_local_planner/TebLocalPlannerROS ; exec bash"
        screen -dmS sar.sonar sh -c "python2 $DIR/ros_1/control/sonar.py; exec bash"

        screen -dmS sar.state_machine sh -c "python3 control/statemachine.py; exec bash"
        screen -dmS sar.amcl sh -c "roslaunch $PARAMS_DIR_PATH/amcl.launch.xml; exec bash"
        screen -dmS sar.localisation sh -c "rosrun micvision micvision_localization_node _quick_score:=False; exec bash"
        screen -dmS sar.map sh -c "rosrun map_server map_server $MAP_PATH; exec bash"
        screen -dmS sar.rotation_degree sh -c "python2 control/rotation_degree.py; exec bash"

        # This section specifies the different mapping modes with the screens running under each mode.
        # Autonomous mapping mode
        if [ "$mode" = "AM" ]; then
            PARAMS_DIR_PATH=$PARAMS_DIR_PATH"_"$mode
            screen -dmS sar.core_gmapping sh -c "rosrun gmapping slam_gmapping _xmin:=-25 _ymin=-25 _xmax:=25 _ymax:=25; exec bash"
            screen -dmS sar.explorer sh -c "roslaunch explore_lite explore.launch; exec bash"
        # Mapping mode
        elif [ "$mode" = "M" ]; then
            screen -dmS sar.core_gmapping sh -c "rosrun gmapping slam_gmapping _xmin:=-25 _ymin=-25 _xmax:=25 _ymax:=25; exec bash"
        # Default mode - Navigation
        elif [ -z "$mode" ]; then
          	rosparam load $PARAMS_DIR_PATH/costmap_common_params.yaml /move_base/global_costmap
          	rosparam load $PARAMS_DIR_PATH/costmap_common_params.yaml /move_base/local_costmap
          	rosparam load $PARAMS_DIR_PATH/local_costmap_params.yaml /move_base
        fi

        screen -ls

    fi


     # This section is specifically for the AIRBOT. This shows the screens that run for AIRBOT.

    if [ "$module" = "airbot" ]; then
        
        rosparam set /robot_description "`python $DIR"/ros_1/control/xacro.py" $XACRO_PATH`"
        rosparam set /ticks_meter $TICKS_METER
        rosparam set /motor_direction $MOTOR_DIRECTION
        rosparam set /twist_factor $TWIST_FACTOR

        screen -dmS sar.robot_state_publisher sh -c "rosrun robot_state_publisher robot_state_publisher; exec bash"
        screen -dmS sar.joint_state_publisher sh -c "rosrun joint_state_publisher joint_state_publisher; exec bash"
        screen -dmS sar.realsense_cameras sh -c "roslaunch airbot_control rs_cameras.launch; exec bash"
        screen -dmS sar.ar_marker_detector sh -c "roslaunch airbot_control ar_marker_detector.launch; exec bash"
        screen -dmS sar.motion_state_controller sh -c "rosrun airbot_control motion_state_controller.py; exec bash"
        screen -dmS sar.motion_planner sh -c "rosrun airbot_control motion_planner; exec bash"

        screen -dmS sar.motor_control sh -c "python2 $DIR/ros_1/control/motor_control.py; exec bash"
      	screen -dmS sar.diff_tf sh -c "python2 $DIR/ros_1/control/diff_tf.py; exec bash"
        screen -dmS sar.arduino sh -c "rosrun rosserial_python serial_node.py _port:=/dev/arduino _baud:=115200; exec bash"
        screen -dmS sar.bridge sh -c "roslaunch rosbridge_server rosbridge_websocket.launch; exec bash"
        screen -dmS sar.server sh -c "sleep 40; nodemon server/server.js; exec bash"

    fi

    # This section is specifically for the CORE-0 arm. This shows the screens that run for CORE-0 arm.

    if [ "$module" = "core0_arm" ]; then

        screen -dmS sar.kortex_driver sh -c "roslaunch core0_arm kortex_driver.launch ip_address:=192.168.1.10; exec bash"
        screen -dmS sar.kinova_vision sh -c "roslaunch kinova_vision kinova_vision.launch device:=192.168.1.10; exec bash"

    fi

    # This section is specifically for the autonomous floor scrubber. This shows the screens that run for AFS.

    if [ "$module" = "afs" ]; then
# For example: To run the robot
# ./utilities/scripts/run.sh -module () -motor () -system () -mode ()

# Under every module there are multiple screens running. A description of each screen running is provided below.description "`python $DIR"/ros_1/control/xacro.py" $XACRO_PATH`"
        rosparam set /ticks_meter $TICKS_METER
        rosparam set /motor_direction $MOTOR_DIRECTION
        rosparam set /encoder_direction $ENCODER_DIRECTION
        rosparam set /twist_factor $TWIST_FACTOR

        screen -dmS sar.robot_state_publisher sh -c "rosrun robot_state_publisher robot_state_publisher; exec bash"
        screen -dmS sar.joint_state_publisher sh -c "rosrun joint_state_publisher joint_state_publisher; exec bash"
        screen -dmS sar.motor_control sh -c "rosrun afs_control motor_control.py; exec bash"
        screen -dmS sar.odometry sh -c "rosrun afs_control odometry.py; exec bash"
        screen -dmS sar.lidar sh -c "roslaunch afs_navigation rplidar.launch; exec bash"
        screen -dmS sar.arduino sh -c "rosrun rosserial_python serial_node.py _port:=/dev/arduino _baud:=115200; exec bash"
        screen -dmS sar.bridge sh -c "roslaunch rosbridge_server rosbridge_websocket.launch; exec bash"
        screen -dmS sar.server sh -c "nodemon server/server.js; exec bash"
        screen -dmS sar.pal sh -c "roslaunch dreamvu_pal_camera detection_rviz.launch; exec bash "

    # As explained above there are two different modes for AFS. M - Mapping and N - Navigation. 
    # In case we want to switch the modes there is a script called mode_switch.sh. we can switch between mode M and N using this script.
    # To run the script --> ./sar/scripts/mode_switch.sh

        if [ "$mode" = "M" ]; then

            screen -dmS sar.gmapping sh -c "roslaunch afs_navigation gmapping.launch; exec bash"

        elif [ "$mode" = "N" ]; then

            screen -dmS sar.amcl sh -c "roslaunch afs_navigation amcl.launch; exec bash"
            screen -dmS sar.micvision sh -c "roslaunch afs_navigation micvision.launch; exec bash"
            screen -dmS sar.map_server sh -c "rosrun map_server map_server $MAP_PATH; exec bash"
            screen -dmS sar.move_base sh -c "roslaunch afs_navigation move_base.launch; exec bash"
            screen -dmS sar.pal_depth sh -c "rosrun dreamvu_pal_camera afs_person_depth; exec bash"
        fi

        screen -ls

    fi

fi

if [ $localization -ne 0 ]; then
    screen -dmS sar.localizer sh -c "sleep 20; rosservice call /StartLocalization"
fi