#!/usr/bin/env bash
set -eo pipefail

task_repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
task_build_directory=${RMCS_SIM_BRIDGE_BUILD_DIRECTORY:-"${task_repository_root}/rmcs_ws/build/v6_component_sim_bridge"}

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "Run this script in the RMCS development container with ROS Jazzy." >&2
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "${task_repository_root}/rmcs_ws/install/setup.bash"
exec "${task_build_directory}/v6_component_sim_bridge" "$@"
