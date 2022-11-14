# This file explains the architecture of our sar-repo.
# Go through this readme completely to understand the folder organization and follow the same conventions while working and committing the developments.

Cloning the repository:
=======================
Install the SAR repository in odroid using the git clone command

---> git clone https://git-codecommit.us-east-2.amazonaws.com/v1/repos/sar

If you don’t have git installed in the system, then run the following command in the ssh terminal,

---> sudo apt-get install git


======================================================================================================================
# Folder structure in sar :

# Section 1:
    
     catkin_ws  - This workspace contains all the packages of our robots in ROS_1. Currently it has the following robots:

        = afs(autonomous floor scrubber):
            Purpose - Autonomous cleaning in shopping complexes, airports etc.
        <!-- There is a separate drive for afs where we have all the manufacturer documents, electronics, mechanical and software documentation as well. the link is attached below. -->
    --->  https://drive.google.com/drive/folders/1ugdxB-jbL3td8wJXsVBISAU4BLwARfDL

        = airbot:
            Purpose - Autonomous disinfection in airplanes, buses.
    
        = core0_arm:
            Purpose - Autonomous disinfection in closed corners (particularly the areas where the robot cannot reach). This arm is mounted on the core0 and is operated independenly.
    
        = dreamvu_pal_camera:
            Purpose - This is a 360 degree camera from a partner company dream_vu. This was earlier mounted on afs and was used for obstacle detection and person detection. All the works done on this can be reviewed in the package. Apart from that the datasheets the fiwmware and other documentation can e found on the following link.

    --->   https://drive.google.com/drive/folders/1ygvQFIDsK0o0in6Z0K-CKX_8_h6e7oIC 

    # The CORE0 (autonomous base) has been ported to ROS2 and all the packages related to it have been shifted in the ros2_ws.

# For all our robot packages we mainatin a folder convertion:
=== robot_control: This has the code for the odometry calculation of the robot, motor_control etc.
=== robot_model: This has the urdf, world, meshes, rviz (display), controllers of the robot. 
=== robot_navigation: This has params configurations and the launch files for running the navigation and mapping etc.

<!-- All of the robots are run through a standard script file i.e. "run.sh". All the launch files and commands are called inside the run.sh script. More details about this are explained below in section 9. -->

<!-- Note: make sure the meshes of any of the robots are not pushed in the repo. All the meshes and larer files should be commited on the drive. The link to the drive is given below. -->

---> https://drive.google.com/drive/folders/195srOtgVtHJoMvJ0pG3faKZHn1l9UalH 

Setting up catkin workspace :
=============================
cd /home/dev/sar/catkin_ws

rosdep install --from-paths src --ignore-src -r -y && catkin_make

If there's any problem while doing the catkin_make, then spot the package which creates the problem.

There may high CPU usage and Total memory occupancy, so use swap memory or remove the package which is causing the issue, using the following command,

sudo rm -rf <package name>

Now again do the catkin_make

After compilation, clone the removed package and run catkin_make with only that package

catkin_make --only-pkg-with-deps <package>

======================================================================================================================

# Section 2:

     ros2_ws -This workspace is for ROS2 and currently contains the packages of CORE0. This workpace is dedicated to the navigation stack of CORE0. 
        Purpose: It is our autonomous base on which our medicine transportation robot (AMRO), Disinfection robot (Lytbot 2.0) can be mounted.  

Setting up ros2 workspace :
=============================
cd /home/dev/sar/ros2_ws

rosdep install -i --from-path src --rosdistro foxy -y && colcon build

If there's any problem while doing the catkin_make, then spot the package which creates the problem.

There may high CPU usage and Total memory occupancy, so use swap memory or remove the package which is causing the issue, using the following command,

sudo rm -rf <package name>

Now again do the colcon build

After compilation, clone the removed package and run catkin_make with only that package

catkin_make --only-pkg-with-deps <package>


======================================================================================================================

# Section 3:

     control - This folder has the codes for the CORE0 in ROS1.    
     
     @Nathan- is a description needed to be added here for every file in the folder.

======================================================================================================================

# Section 4:

        db - 
    
    @Nathan - I am not much aware of this section. This is from website.

======================================================================================================================

# Section 5:

     logs - 

    @Nathan - I am not much aware of this section. This is from website.

======================================================================================================================

# Section 6:

     maps - This folder contains the maps of the environments where our robots run. Make sure all the maps(.pgm and .yaml) files are pushed and committed with a proper description of the environment. For e.g. we have the map of Jetbrain office with the name "jetbrain.pgm or jetbrain.yaml.

======================================================================================================================

# Section 7:

     modules - This folder has the urdf files of the robots. We will be using these urdf files in the run.sh. The urdf once finalized in simulation need to be updated here and the path needs to be updated in the 
     -->utilities > modules > robot.sh file.
     # robot = hotbot, afs, amro etc.
     
======================================================================================================================

# Section 8:

    server -

    @Nathan - I am not much aware of this section. This is from website.

======================================================================================================================

# Section 9:

    utilities - This sections contains majority of the stuff that is needed to run the robot. The details of the folders is explained below.
        --> config- This folder consists of 2 sub-folders:
                    
                    --> modules - Modules refers to our robots. We refer to our robots as modules and define it along with the path of urdf file, map file, database in robot.sh in this folder. We follow the same convention (module) in the run.sh
                    --> motors - This contains the different motor configurations (gear ratio) of the motors used in the robot. Currently we are using gear_ratio_50 for CORE0 and gear_ratio_18 for AFS.

        --> launch- Consists of different launch files that are called in the run.sh. 
                    for e.g. gmapping. launch ( This is called in run.sh if mapping needs to be done).

        --> params- This contains the navigation_stack params. The params are configured as per the different onboard computers in our robots. 

        --> scripts- This folder consists of the scripts for running the robot. The system can be run individually from the run.sh script only.
                        The killall.sh script is used for killing all the ongoing processes in the onboard computer.
                        The system-setup.sh script is used to  set-up a new onboard computer.
                        The setup.sh script is used to configure our database (db) and maps from the server. All it contains the packages that are needed to configure the workspace in the computer.

# Here is a description of how to use diferent scripts:
<!-- Note: This is currently for ROS1 -->

To run the robot, run the following command :
==============================================

---> ./sar/utilities/scripts/run.sh <name of the module  --default Core0> <Name of the SBC(onboard computer)  --default Odroid XU4>  

It’s better to set the ROS_MASTER_URI and ROS_HOSTNAME IP address manually, sometimes run.sh doesn’t specify the IP address for the master and the host.

## Check all the wiring connections are right, and make sure none of the cables aren't connected properly

## Change this in your laptop/desktop

## Change export ROS_MASTER_URI=http://<Mobile_Robot_IP_Address:11311

## export ROS_HOSTNAME=<Local_machine_IP_Address>

Teleop - move the robot using the keyboard input using specific keys:
=====================================================================

---> rosrun teleop_twist_keyboard teleop_twist_keyboard.py


To kill all the processes :
===========================

---> ./sar/utilities/scripts/killall.sh

This will kill all the programs running in the background

This script runs “rosclean purge -y”, and kills all screen running at that moment

Mapping the environment using gmapping:
=======================================

---> ./sar/utilities/scripts/run.sh <name of the module  --default Core0> <Name of the SBC  --default Odroid XU4> -mode M

use teleop to move around and map the environment manually.

Add map option, and subscribe to /map publisher to receive the map.

To save the map of any environment use the following command:
=============================================================

---> rosrun map_server map_saver -f map

This command will create two files, .pgm and .yaml, put these two files in appropriate folder to call them for navigation.

Navigation:
===========

## Once mapping part is done, then move forward for the navigation

---> ./sar/utilities/scripts/run.sh <Name of the SBC  --default Odroid XU4>


Subscribe to /map

Add teb_local_planner and teb_global_planner (Under path), tf

Add pose, laser, particle cloud (Optional)

Notice: That the bot is not in correct pose, so need to start localization and define it's state.

---> rosservice call /StartLocalization

## Use 2D Nav Goal to set the goal position for the robot.


## install Aws cli
curl "https://awscli.amazonaws.com/awscli-exe-linux-aarch64.zip" -o "awscliv2.zip"
unzip awscliv2.zip
sudo ./aws/install
## type aws configure and put your access key and password and then run
## ./ros2_docker_setup.sh 