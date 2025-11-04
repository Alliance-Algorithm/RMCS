#!/bin/bash

export RCUTILS_COLORIZED_OUTPUT=1
export ROS_LOCALHOST_ONLY=1
export RMCS_PATH="/workspaces/RMCS"

source /opt/ros/jazzy/setup.zsh

if [ -f "/rmcs_install/local_setup.zsh" ]; then
    source /rmcs_install/local_setup.zsh
elif [ -f "${RMCS_PATH}/rmcs_ws/install/local_setup.zsh" ]; then
    source ${RMCS_PATH}/rmcs_ws/install/local_setup.zsh
fi

eval "$(register-python-argcomplete ros2)"
eval "$(register-python-argcomplete colcon)"

export RMCS_ROBOT_TYPE=""

fpath=(${RMCS_PATH}/.script/complete $fpath)
autoload -Uz compinit
compinit
