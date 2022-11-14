#!/bin/bash

module=""
motorType=""
system="XU4"
mode=""
simulation=""

while [ "$1" != "" ]; do
    case $1 in

	-simulation | --simulation-type )   shift
					    simulation=$1
					    ;;
        -motor | --motor-config )   shift
                                    motorType=$1
                                    ;;
        -module | --module-name )   shift
                                    module=$1
                                    ;;
        -system | --controller-type )   shift
                                    system=$1
                                    ;;
        -mode | --map-mode )        shift
                                    mode=$1
                                    ;;
        -map | --map-path )         shift
                                    map=$1
                                    ;;
    esac
    shift
done

if [ -z "$CURRENT_IP_ADD" ]; then
    if [ ! -z `command -v ifconfig` ]; then
        export CURRENT_IP_ADD=`ifconfig eth0 | grep -E -o "([0-9]{1,3}[\.]){3}[0-9]{1,3}" | head -n 1`
    elif [ ! -z `command -v ip` ]; then
        export CURRENT_IP_ADD=`ip -4 -o a | grep eth0 | grep -E -o "([0-9]{1,3}[\.]){3}[0-9]{1,3}" | head -n 1`
    else
        export CURRENT_IP_ADD="192.168.0.130"
    fi
fi

if [ -z "$ROS_MASTER_URI" ]; then
    source /opt/ros/melodic/setup.bash
    source /home/dev/sar/catkin_ws/devel/setup.bash
    export ROS_MASTER_URI="http://192.168.0.130:11311"
    export ROS_HOSTNAME=192.168.0.130
fi

cd $(dirname $0)/../..

DIR=`pwd`
PARAMS_DIR_PATH=$DIR"/utilities/params/params_"$system
SAR_DEV_PATH="~/sar-dev/catkin_ws/src"

if [ "$simulation" = "true" ]; then
    source $DIR"/utilities/config/modules/gazebo.sh"
    screen -dmS sar.gazebo sh -c "roslaunch $SAR_DEV_PATH/amro/launch/amro.launch; exec bash"
    screen -dmS sar.state_machine sh -c "python3 control/statemachine.py; exec bash"
    screen -dmS sar.server sh -c "node server/server.js; exec bash"
    screen -dmS sar.merged_laser sh -c "python3 control/Mul_sonar_to_laser.py; exec bash"
    screen -ls

else
    if [ "$motorType" = "B" ]; then
        source $DIR"/utilities/config/motors/gear_ratio_15.sh"

    elif [ "$motorType" = "afs" ]; then
	      source $DIR"/utilities/config/motors/gear_ratio_18.sh"

    else
        source $DIR"/utilities/config/motors/gear_ratio_50.sh"
    fi

    screen -dmS sar.core sh -c "roscore; exec bash"
    echo "Waiting for ROS Server.."
    until rostopic list 2> /dev/null ; do echo "."; sleep 1; done

    if [ "$module" = "amro" ]; then
        source $DIR"/utilities/config/modules/amro.sh"
        screen -dmS sar.m_sonar sh -c "python2 $DIR/control/m_sonar.py; exec bash"

    elif [ "$module" = "arya" ]; then
        source $DIR"/utilities/config/modules/arya.sh"
        screen -dmS sar.arya_face_detect sh -c "python modules/arya/camera/face_detect.py; exec bash"

    elif [ "$module" = "lytbot" ]; then
        source $DIR"/utilities/config/modules/lytbot.sh"
    
    elif [ "$module" = "hotbot" ]; then
        source $DIR"/utilities/config/modules/hotbot.sh"
	rosparam set /base_width $BASE_WIDTH

    elif [ "$module" = "uvid" ]; then
        source $DIR"/utilities/config/modules/uvid.sh"
        screen -dmS sar.sonar_brakes sh -c "python2 $DIR/control/sonar_brakes.py; exec bash"

    elif [ "$module" = "afs" ]; then
        source $DIR"/utilities/config/modules/afs.sh"

    elif [ -z "$module" ]; then
        echo "Module argument is missing. Default configurations will run.."
        source $DIR"/utilities/config/modules/default.sh"
    fi

    if ! [ "$module" = "afs" ]; then

        if ! [ "$module" = "uvid" ]; then
            rosparam set /robot_description "`python $DIR"/control/xacro.py" $XACRO_PATH`"
            rosparam load $PARAMS_DIR_PATH/global_costmap_params.yaml /move_base
            rosparam load $PARAMS_DIR_PATH/base_local_planner_params.yaml /move_base
            rosparam load $PARAMS_DIR_PATH/move_base_params.yaml /move_base
        fi

        rosparam set /ticks_meter $TICKS_METER
        rosparam set /motor_direction $MOTOR_DIRECTION
        rosparam set /twist_factor $TWIST_FACTOR

        if [ "$mode" = "AM" ]; then

            PARAMS_DIR_PATH=$PARAMS_DIR_PATH"_"$mode
            screen -dmS sar.core_gmapping sh -c "rosrun gmapping slam_gmapping _xmin:=-25 _ymin=-25 _xmax:=25 _ymax:=25; exec bash"
            screen -dmS sar.explorer sh -c "roslaunch explore_lite explore.launch; exec bash"

        elif [ "$mode" = "M" ]; then

            screen -dmS sar.core_gmapping sh -c "rosrun gmapping slam_gmapping _xmin:=-25 _ymin=-25 _xmax:=25 _ymax:=25; exec bash"

        elif [ -z "$mode" ]; then

          	rosparam load $PARAMS_DIR_PATH/costmap_common_params.yaml /move_base/global_costmap
          	rosparam load $PARAMS_DIR_PATH/costmap_common_params.yaml /move_base/local_costmap
          	rosparam load $PARAMS_DIR_PATH/local_costmap_params.yaml /move_base

            if ! [ "$module" = "uvid" ]; then

                screen -dmS sar.state_machine sh -c "python3 control/statemachine.py; exec bash"
                screen -dmS sar.amcl sh -c "roslaunch $PARAMS_DIR_PATH/amcl.launch.xml; exec bash"
                screen -dmS sar.localisation sh -c "rosrun micvision micvision_localization_node _quick_score:=False; exec bash"
                screen -dmS sar.map sh -c "rosrun map_server map_server $MAP_PATH; exec bash"
                screen -dmS sar.rotation_degree sh -c "python2 control/rotation_degree.py; exec bash"

            fi

        fi

        if ! [ "$module" = "uvid" ]; then

            screen -dmS sar.robot_state_publisher sh -c "rosrun robot_state_publisher robot_state_publisher; exec bash"
            screen -dmS sar.joint_state_publisher sh -c "rosrun joint_state_publisher joint_state_publisher; exec bash"
            screen -dmS sar.lidar sh -c "roslaunch rplidar_ros rplidar.launch; exec bash"
            screen -dmS sar.move_base sh -c "rosrun move_base move_base __name:=move_base _base_local_planner:=teb_local_planner/TebLocalPlannerROS; exec bash"
            screen -dmS sar.sonar sh -c "python2 $DIR/control/sonar.py; exec bash"
        fi

      	# screen -dmS sar.motor_control sh -c "python2 $DIR/control/motor_control.py; exec bash"
        screen -dmS sar.twist_motor sh -c "python2 $DIR/control/twist_motor.py; exec bash"
        screen -dmS sar.lpid_velocity sh -c "python2 $DIR/control/lpid_velocity.py; exec bash"
        screen -dmS sar.rpid_velocity sh -c "python2 $DIR/control/rpid_velocity.py; exec bash"
        screen -dmS sar.diff_tf sh -c "python2 $DIR/control/diff_tf.py; exec bash"
        screen -dmS sar.arduino sh -c "rosrun rosserial_python serial_node.py _port:=/dev/arduino _baud:=115200; exec bash"
	    screen -dmS sar.imu sh -c "rosrun rosserial_python serial_node.py _port:=/dev/imu _baud:=115200 __name:=node_imu; exec bash"
        screen -dmS sar.bridge sh -c "roslaunch rosbridge_server rosbridge_websocket.launch; exec bash"
        screen -dmS sar.server sh -c "node server/server.js; exec bash"
        screen -ls

    fi

    if [ "$module" = "afs" ]; then

        rosparam set /robot_description "`python $DIR"/control/xacro.py" $XACRO_PATH`"
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
        screen -dmS sar.server sh -c "node server/server.js; exec bash"
        screen -dmS sar.pal sh -c "roslaunch dreamvu_pal_camera detection_rviz.launch; exec bash "

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
