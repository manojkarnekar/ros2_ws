# This dockerfile is an environment to run ZED2

# installs has ZED SDK and wrapper; TODO use a smaller image
FROM dustynv/ros:foxy-slam-l4t-r32.6.1

# this fixes a ZED bug where it will start but the data cannot be accessed
# https://github.com/ros-perception/image_common/issues/184
RUN mkdir -p foxy_ws/src
RUN cd foxy_ws/src && git clone https://github.com/ros-perception/image_common.git
RUN cd foxy_ws/src/image_common && git checkout galactic
RUN cd foxy_ws/src && git clone https://github.com/ros-perception/image_transport_plugins.git
RUN cd .. 
RUN cd foxy_ws/src/image_transport_plugins && git checkout foxy-devel
RUN source /opt/ros/foxy/install/setup.bash && colcon build
RUN source install/setup.bash

# necessary to fix above ZED bug
#CMD ros2 run image_transport republish compressed --ros-args -r in/compressed:=image_compressed -r out:=image_raw
