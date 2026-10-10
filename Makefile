ROS_DISTRO ?= jazzy
ROS_SETUP := /opt/ros/$(ROS_DISTRO)/setup.bash
REPO_ROOT := $(CURDIR)
ROS_WS := $(REPO_ROOT)/manipulator/ros2_ws
PYTHON := $(REPO_ROOT)/.venv/bin/python
DOCKER_IMAGE ?= robot-teleoperation-framework

.PHONY: build test list run-mpc docker-build docker-test docker-run

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

docker-build:
	docker build -f docker/Dockerfile -t $(DOCKER_IMAGE) .

docker-test:
	docker run --rm $(DOCKER_IMAGE) python3 -m colcon test-result --verbose

docker-run:
	docker run --rm --net=host --ipc=host $(DOCKER_IMAGE)
