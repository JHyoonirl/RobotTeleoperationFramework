ROS_DISTRO ?= jazzy
ROS_SETUP := /opt/ros/$(ROS_DISTRO)/setup.bash
REPO_ROOT := $(CURDIR)
ROS_WS := $(REPO_ROOT)/manipulator/ros2_ws
PYTHON := $(REPO_ROOT)/.venv/bin/python

.PHONY: build test list run-mpc

build:
	@test -f "$(ROS_SETUP)" || (echo "ROS 2 setup not found: $(ROS_SETUP)"; exit 1)
	@test -x "$(PYTHON)" || (echo "Python environment not found: $(PYTHON)"; exit 1)
	bash -lc 'set -e; source "$(REPO_ROOT)/.venv/bin/activate"; source "$(ROS_SETUP)"; cd "$(ROS_WS)"; python -m colcon build --symlink-install'

test:
	@test -x "$(PYTHON)" || (echo "Python environment not found: $(PYTHON)"; exit 1)
	bash -lc 'set -e; source "$(REPO_ROOT)/.venv/bin/activate"; source "$(ROS_SETUP)"; cd "$(ROS_WS)"; python -m colcon test --python-testing pytest; python -m colcon test-result --verbose'

list:
	bash -lc 'source "$(REPO_ROOT)/.venv/bin/activate"; source "$(ROS_SETUP)"; cd "$(ROS_WS)"; python -m colcon list'

run-mpc:
	bash -lc 'set -e; source "$(REPO_ROOT)/.venv/bin/activate"; source "$(ROS_SETUP)"; cd "$(ROS_WS)"; source install/setup.bash; ros2 launch indy7_mpc indy7_mpc.launch.py'
