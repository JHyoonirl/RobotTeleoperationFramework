FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONUNBUFFERED=1 \
    MPLCONFIGDIR=/tmp/matplotlib

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        python3-dev \
        python3-matplotlib \
        python3-pip \
        python3-venv \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-control-test-assets \
        ros-jazzy-ros2-controllers \
        ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/robot_teleoperation_framework

COPY manipulator/requirements-crocoddyl.txt /tmp/requirements-crocoddyl.txt
RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed \
        -r /tmp/requirements-crocoddyl.txt \
        "matplotlib>=3.8" \
        "scipy>=1.14" \
    && rm /tmp/requirements-crocoddyl.txt

COPY manipulator/indy7_mpc_test.py ./indy7_mpc_test.py
COPY manipulator/ros2_ws/src ./src

RUN source /opt/ros/jazzy/setup.bash \
    && python3 -m colcon build --merge-install --install-base /opt/robot_teleoperation_framework/install \
    && source /opt/robot_teleoperation_framework/install/setup.bash \
    && python3 -m colcon test --merge-install --install-base /opt/robot_teleoperation_framework/install \
        --packages-select indy7_mpc --event-handlers console_direct+ \
    && python3 -m colcon test-result --verbose

COPY docker/entrypoint.sh /usr/local/bin/robot-teleoperation-entrypoint
RUN chmod +x /usr/local/bin/robot-teleoperation-entrypoint

ENTRYPOINT ["/usr/local/bin/robot-teleoperation-entrypoint"]
CMD ["ros2", "launch", "indy7_mpc", "indy7_mpc.launch.py", "rviz:=false", "plot:=false"]
