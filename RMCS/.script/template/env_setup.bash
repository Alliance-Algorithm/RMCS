#!/bin/bash

export ROS_LOCALHOST_ONLY=1

source /opt/ros/jazzy/setup.bash

if [ -f "/rmcs_install/local_setup.bash" ]; then
    source /rmcs_install/local_setup.bash
elif [ -f "/workspaces/RMCS/rmcs_ws/install/local_setup.bash" ]; then
    source /workspaces/RMCS/rmcs_ws/install/local_setup.bash
fi

export RMCS_ROBOT_TYPE=""
