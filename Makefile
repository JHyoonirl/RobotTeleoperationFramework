ROS_DISTRO ?= jazzy
ROS_SETUP := /opt/ros/$(ROS_DISTRO)/setup.bash
REPO_ROOT := $(CURDIR)

ROS_WS := $(REPO_ROOT)/ros2_ws
ROS_SOURCES := $(ROS_WS)/src
PYTHON := $(REPO_ROOT)/.venv/bin/python
DOCKER_IMAGE ?= robot-teleoperation-framework
INCLUDE_HAND ?= 0
DOCKER_TAG := mpc
DOCKER_BASE_PATHS := /workspace/ros2_ws/src/indy7_description \
	/workspace/ros2_ws/src/indy7_mpc \
	/workspace/ros2_ws/src/vive_tracker_ros2

.PHONY: build build-hand test list run-mpc docker-build docker-build-dcp3 docker-build-gui docker-test docker-run docker-run-gui docker-build-hand docker-up-hand docker-down-hand docker-check-indy7 docker-up-indy7 docker-down-indy7

build:
	@test -f "$(ROS_SETUP)" || (echo "ROS 2 setup not found: $(ROS_SETUP)"; exit 1)
	@test -x "$(PYTHON)" || (echo "Python environment not found: $(PYTHON)"; exit 1)
	bash -lc 'set -e; source "$(REPO_ROOT)/.venv/bin/activate"; source "$(ROS_SETUP)"; cd "$(ROS_WS)"; python -m colcon build --symlink-install --merge-install --base-paths $(ROS_SOURCES)'

build-hand:
	@test -f "$(ROS_SETUP)" || (echo "ROS 2 setup not found: $(ROS_SETUP)"; exit 1)
	@command -v colcon >/dev/null || (echo "colcon is not installed on the host"; exit 1)
	bash -lc 'set -e; source "$(ROS_SETUP)"; HAND_ROS_WS="$(ROS_WS)" bash "$(ROS_WS)/src/hand_retargetting/tools/build_ros.sh"'

test:
	@test -x "$(PYTHON)" || (echo "Python environment not found: $(PYTHON)"; exit 1)
	bash -lc 'set -e; source "$(REPO_ROOT)/.venv/bin/activate"; source "$(ROS_SETUP)"; cd "$(ROS_WS)"; python -m colcon test --merge-install --base-paths $(ROS_SOURCES) --python-testing pytest --pytest-args " --ignore=third_party"; python -m colcon test-result --verbose'

list:
	bash -lc 'source "$(REPO_ROOT)/.venv/bin/activate"; source "$(ROS_SETUP)"; cd "$(ROS_WS)"; python -m colcon list --base-paths $(ROS_SOURCES)'

run-mpc:
	bash -lc 'set -e; source "$(REPO_ROOT)/.venv/bin/activate"; source "$(ROS_SETUP)"; cd "$(ROS_WS)"; source install/setup.bash; ros2 launch indy7_mpc indy7_mpc.launch.py'

docker-build:
	docker buildx build --load \
		--target mpc -f docker/Dockerfile -t $(DOCKER_IMAGE):mpc .
	$(MAKE) docker-build-dcp3
ifeq ($(INCLUDE_HAND),1)
	$(MAKE) docker-build-hand
endif

docker-build-gui:
	docker buildx build --load --target mpc-gui \
		-f docker/Dockerfile -t $(DOCKER_IMAGE):mpc-gui .

docker-build-dcp3:
	docker buildx build --load --target dcp3 \
		-f docker/Dockerfile -t $(DOCKER_IMAGE):dcp3 .

docker-build-hand:
	bash "$(ROS_WS)/src/hand_retargetting/tools/docker_manus.sh" build

docker-up-hand:
	bash "$(ROS_WS)/src/hand_retargetting/tools/docker_manus.sh" up

docker-down-hand:
	bash "$(ROS_WS)/src/hand_retargetting/tools/docker_manus.sh" down

docker-check-indy7:
	@test -n "$(ROBOT_IP)" || (echo "Set ROBOT_IP to the Indy7 controller address"; exit 2)
	INDY7_DCP3_IMAGE=$(DOCKER_IMAGE):dcp3 \
	INDY7_ROBOT_IP=$(ROBOT_IP) \
	docker compose -f docker/compose.indy7.yaml up dcp3

docker-test:
	docker run --rm $(DOCKER_IMAGE):$(DOCKER_TAG) python3 -m colcon test-result --verbose
	docker run --rm $(DOCKER_IMAGE):dcp3 python3 -m colcon test-result --verbose

docker-run:
	docker run --rm -it --name robot-teleoperation --net=host --ipc=host \
		--user "$$(id -u):$$(id -g)" -e HOME=/tmp \
		-v "$(REPO_ROOT):/workspace" -w /workspace \
		$(DOCKER_IMAGE):$(DOCKER_TAG) \
		bash -lc 'source /opt/ros/jazzy/setup.bash && cd /workspace/ros2_ws && python3 -m colcon build --symlink-install --merge-install --base-paths $(DOCKER_BASE_PATHS) && source install/setup.bash && cd /workspace && exec bash'

docker-up-indy7:
	@test -n "$(ROBOT_IP)" || (echo "Set ROBOT_IP to the Indy7 controller address"; exit 2)
	ROBOT_TELEOP_MPC_IMAGE=$(DOCKER_IMAGE):$(DOCKER_TAG) \
	INDY7_DCP3_IMAGE=$(DOCKER_IMAGE):dcp3 \
	INDY7_ROBOT_IP=$(ROBOT_IP) \
	docker compose -f docker/compose.indy7.yaml up

docker-down-indy7:
	docker compose -f docker/compose.indy7.yaml down

docker-run-gui:
	@test -n "$$DISPLAY" || { echo "DISPLAY is not set; run this from an X11 desktop session"; exit 1; }
	$(MAKE) docker-build-gui
	@XAUTHORITY_FILE="$${XAUTHORITY:-$$HOME/.Xauthority}"; \
		test -r "$$XAUTHORITY_FILE" || { echo "X11 authorization file is not readable: $$XAUTHORITY_FILE"; exit 1; }; \
		docker run --rm --name robot-teleoperation --net=host --ipc=host \
			--user "$$(id -u):$$(id -g)" -e HOME=/tmp \
			-e "DISPLAY=$$DISPLAY" \
			-e XAUTHORITY=/tmp/.Xauthority \
			-e QT_X11_NO_MITSHM=1 \
			-v "$$XAUTHORITY_FILE:/tmp/.Xauthority:ro" \
			-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
			-v "$(REPO_ROOT):/workspace" -w /workspace \
			$(DOCKER_IMAGE):mpc-gui \
			bash -lc 'source /opt/ros/jazzy/setup.bash && cd /workspace/ros2_ws && python3 -m colcon build --symlink-install --merge-install --base-paths $(DOCKER_BASE_PATHS) && source install/setup.bash && ros2 launch indy7_mpc indy7_mpc.launch.py rviz:=true plot:=false'
