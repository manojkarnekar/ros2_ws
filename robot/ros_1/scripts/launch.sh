#!/bin/bash

params_dir=utilities/params/params_N2
rosparam load $params_dir"/local_costmap_params.yaml"
rosparam load $params_dir"/costmap_common_params.yaml" /global_costmap
rosparam load $params_dir"/costmap_common_params.yaml" /local_costmap
rosparam load $params_dir"/local_costmap_params.yaml"
rosparam load $params_dir"/global_costmap_params.yaml"
rosparam load $params_dir"/base_local_planner_params.yaml"
rosparam load $params_dir"/move_base_params.yaml"
rosparam load $params_dir"/plugin.yaml"
rosrun move_base move_base
