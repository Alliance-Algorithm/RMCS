#!/usr/bin/zsh

workspace=/workspaces/rmcs/rmcs_ws

cd ${workspace} && colcon build --merge-install --event-handlers console_direct+ 

source /workspaces/rmcs/rmcs_ws/install/setup.zsh
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"