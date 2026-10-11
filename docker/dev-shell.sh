#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 3 ]]; then
    echo "Usage: dev-shell.sh BUILD_ROOT BASE_PATH... -- COMMAND [ARGS...]" >&2
    exit 2
fi

build_root=$1
shift
base_paths=()
while (($#)) && [[ $1 != -- ]]; do
    base_paths+=("$1")
    shift
done
if (($# == 0)); then
    echo "Missing -- COMMAND separator" >&2
    exit 2
fi
shift
if (($# == 0)); then
    set -- bash
fi

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ -f /opt/robot-teleoperation-venv/bin/activate ]]; then
    source /opt/robot-teleoperation-venv/bin/activate
fi
if [[ -f /opt/robot_teleoperation_framework/install/setup.bash ]]; then
    source /opt/robot_teleoperation_framework/install/setup.bash
fi

if [[ ! -f "$build_root/install/setup.bash" || "${FORCE_COLCON_BUILD:-0}" == 1 ]]; then
    python3 -m colcon --log-base "$build_root/log" build \
        --symlink-install --merge-install \
        --base-paths "${base_paths[@]}" \
        --build-base "$build_root/build" \
        --install-base "$build_root/install"
fi

source "$build_root/install/setup.bash"
cd /workspace
exec "$@"
