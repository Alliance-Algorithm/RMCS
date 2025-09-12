#!/bin/bash

export ROS_LOCALHOST_ONLY=1
export RMCS_PATH="/workspaces/RMCS"

source /opt/ros/jazzy/setup.bash

if [ -f "/rmcs_install/local_setup.bash" ]; then
    source /rmcs_install/local_setup.bash
elif [ -f "${RMCS_PATH}/rmcs_ws/install/local_setup.bash" ]; then
    source ${RMCS_PATH}/rmcs_ws/install/local_setup.bash
fi

export RMCS_ROBOT_TYPE=""
