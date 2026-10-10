#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /opt/robot_teleoperation_framework/install/setup.bash

exec "$@"
