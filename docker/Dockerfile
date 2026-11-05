# syntax=docker/dockerfile:1

FROM ros:jazzy-ros-base AS base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy \
    ROS_DOMAIN_ID=77 \
    PYTHONUNBUFFERED=1 \
    MPLCONFIGDIR=/tmp/matplotlib

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY docker/entrypoint.sh /usr/local/bin/robot-teleoperation-entrypoint
RUN chmod +x /usr/local/bin/robot-teleoperation-entrypoint

ENTRYPOINT ["/usr/local/bin/robot-teleoperation-entrypoint"]

FROM base AS mpc

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        python3-colcon-common-extensions \
        python3-dev \
        python3-pip \
        python3-pytest \
        python3-venv \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-tf2-ros-py \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-control-test-assets \
        ros-jazzy-controller-manager \
        ros-jazzy-joint-state-broadcaster \
        ros-jazzy-position-controllers \
    && rm -rf /var/lib/apt/lists/*

ENV VIRTUAL_ENV=/opt/robot-teleoperation-venv \
    PATH=/opt/robot-teleoperation-venv/bin:${PATH}
RUN python3 -m venv --system-site-packages "$VIRTUAL_ENV"

WORKDIR /opt/robot_teleoperation_framework

COPY requirements/robot-mpc.txt /tmp/requirements-mpc.txt
COPY requirements/robot-mpc-constraints.txt /tmp/requirements-mpc-constraints.txt
RUN python3 -m pip install --no-cache-dir \
        --constraint /tmp/requirements-mpc-constraints.txt \
        -r /tmp/requirements-mpc.txt \
    && rm /tmp/requirements-mpc.txt /tmp/requirements-mpc-constraints.txt

COPY robots/indy7/indy7_mpc_test.py ./indy7_mpc_test.py
COPY ros2_ws/src/robot_description ./src/robot_description
COPY ros2_ws/src/robot_mpc ./src/robot_mpc

RUN source /opt/ros/jazzy/setup.bash \
    && python3 -m colcon build --merge-install \
        --base-paths /opt/robot_teleoperation_framework/src/robot_description \
            /opt/robot_teleoperation_framework/src/robot_mpc

CMD ["ros2", "launch", "robot_mpc", "robot_mpc.launch.py", "rviz:=false", "plot:=false"]

FROM mpc AS mpc-test

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-pyqt5 \
    && rm -rf /var/lib/apt/lists/*

COPY ros2_ws/src/robot_dcp3 ./src/robot_dcp3

RUN source /opt/ros/jazzy/setup.bash \
    && python3 -m colcon build --merge-install \
        --base-paths /opt/robot_teleoperation_framework/src/robot_description \
            /opt/robot_teleoperation_framework/src/robot_mpc \
            /opt/robot_teleoperation_framework/src/robot_dcp3 \
    && source /opt/robot_teleoperation_framework/install/setup.bash \
    && python3 -m colcon test --merge-install \
        --packages-select robot_description robot_mpc \
        --python-testing pytest --pytest-args " --ignore=third_party" \
        --event-handlers console_direct+ \
    && python3 -m colcon test-result --verbose

FROM mpc AS mpc-gui

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-rviz2 \
        python3-pyqt5 \
    && rm -rf /var/lib/apt/lists/*

FROM base AS dcp3

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-pip \
        python3-pytest \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

ENV VIRTUAL_ENV=/opt/robot-teleoperation-venv \
    PATH=/opt/robot-teleoperation-venv/bin:${PATH}
RUN python3 -m venv --system-site-packages "$VIRTUAL_ENV"

WORKDIR /opt/robot_teleoperation_framework
COPY requirements/neuromeka-dcp3.txt /tmp/neuromeka-dcp3-requirements.txt
# Neuromeka pins Pillow 9.5, which has no Python 3.12 wheel; its compatible runtime set is explicit.
RUN python3 -m pip install --no-cache-dir --no-deps \
        -r /tmp/neuromeka-dcp3-requirements.txt \
    && python3 -c "from neuromeka import IndyDCP3" \
    && rm /tmp/neuromeka-dcp3-requirements.txt

COPY ros2_ws/src/robot_description ./src/robot_description
COPY ros2_ws/src/robot_dcp3 ./src/robot_dcp3

RUN source /opt/ros/jazzy/setup.bash \
    && python3 -m colcon build --merge-install \
        --base-paths /opt/robot_teleoperation_framework/src/robot_description \
            /opt/robot_teleoperation_framework/src/robot_dcp3 \
    && source /opt/robot_teleoperation_framework/install/setup.bash \
    && python3 -m colcon test --merge-install \
        --packages-select robot_description robot_dcp3 \
        --python-testing pytest --event-handlers console_direct+ \
    && python3 -m colcon test-result --verbose

ENV ROBOT_IP=
CMD ["bash", "-lc", "test -n \"$ROBOT_IP\" || { echo 'Set ROBOT_IP to the robot controller address.' >&2; exit 2; }; exec ros2 launch robot_dcp3 robot_dcp3.launch.py robot_ip:=$ROBOT_IP"]
