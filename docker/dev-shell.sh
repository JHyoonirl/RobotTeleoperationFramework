#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 3 ]]; then
    echo "Usage: dev-shell.sh BUILD_ROOT BASE_PATH... -- COMMAND [ARGS...]" >&2
    exit 2
fi

dev_root=$1
build_root=$dev_root
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

# Keep colcon artifacts owned by the active container user. Older builds may
# have been created as container root (shown as `nobody` on a user-namespaced
# Docker host), which prevents a later non-root build from updating the cache.
build_root="${build_root}/users/uid-$(id -u)"
mkdir -p "$build_root"

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
# RViz must save into the writable development mount, not its installed
# (possibly symlinked/read-only) package config. A directory permits Qt's
# temporary-file + rename save operation and survives container recreation.
export INDY7_RVIZ_CONFIG="${INDY7_RVIZ_CONFIG:-$dev_root/rviz/indy7.rviz}"
cd /workspace
exec "$@"
