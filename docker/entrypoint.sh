#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ -f /opt/robot-teleoperation-venv/bin/activate ]]; then
    source /opt/robot-teleoperation-venv/bin/activate
fi
source /opt/robot_teleoperation_framework/install/setup.bash

exec "$@"
