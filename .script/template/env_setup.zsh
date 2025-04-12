#!/bin/bash

export ROS_LOCALHOST_ONLY=1

source /opt/ros/humble/setup.zsh

if [ -f "/rmcs_install/local_setup.zsh" ]; then
    source /rmcs_install/local_setup.zsh
elif [ -f "/workspaces/RMCS/rmcs_ws/install/local_setup.zsh" ]; then
    source /workspaces/RMCS/rmcs_ws/install/local_setup.zsh
fi

eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"

export RMCS_ROBOT_TYPE=""
