ROS_DISTRO ?= jazzy
ROS_SETUP := /opt/ros/$(ROS_DISTRO)/setup.bash
REPO_ROOT := $(CURDIR)
export ROS_DOMAIN_ID := 77

ROS_WS := $(REPO_ROOT)/ros2_ws
ROS_SOURCES := $(ROS_WS)/src
PYTHON := $(REPO_ROOT)/.venv/bin/python
DOCKER_IMAGE ?= robot-teleoperation-framework
INCLUDE_HAND ?= 0
DOCKER_TAG := mpc
ROBOT_SERVICE ?= mpc
FORCE_COLCON_BUILD ?= 0
DOCKER_BASE_PATHS := /workspace/ros2_ws/src/robot_description \
	/workspace/ros2_ws/src/robot_mpc
DOCKER_DCP3_BASE_PATHS := /workspace/ros2_ws/src/robot_description \
	/workspace/ros2_ws/src/robot_dcp3

.PHONY: build build-hand test list run-mpc docker-build docker-build-dcp3 docker-build-gui docker-test docker-run docker-run-gui docker-build-hand docker-up-hand docker-down-hand docker-check-robot docker-up-robot docker-shell-robot docker-launch-robot-dcp3 docker-down-robot docker-up-robot-sim docker-shell-robot-sim docker-launch-robot-sim docker-launch-robot-sim-minimal docker-keyboard-robot-sim docker-down-robot-sim

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
	bash -lc 'set -e; source "$(REPO_ROOT)/.venv/bin/activate"; source "$(ROS_SETUP)"; cd "$(ROS_WS)"; source install/setup.bash; ros2 launch robot_mpc robot_mpc.launch.py'

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

docker-check-robot:
	@test -n "$(ROBOT_IP)" || (echo "Set ROBOT_IP to the robot controller address"; exit 2)
	ROBOT_DCP3_IMAGE=$(DOCKER_IMAGE):dcp3 \
	ROBOT_IP=$(ROBOT_IP) \
	docker compose -f docker/compose.robot.yaml run --rm --no-deps dcp3 \
		bash -lc 'set -e; source /opt/ros/jazzy/setup.bash; source /opt/robot-teleoperation-venv/bin/activate; cd /workspace/ros2_ws; python3 -m colcon --log-base /tmp/robot_dcp3_ws/log build --symlink-install --merge-install --base-paths /workspace/ros2_ws/src/robot_description /workspace/ros2_ws/src/robot_dcp3 --build-base /tmp/robot_dcp3_ws/build --install-base /tmp/robot_dcp3_ws/install; source /tmp/robot_dcp3_ws/install/setup.bash; exec ros2 launch robot_dcp3 robot_dcp3.launch.py robot_ip:=$$ROBOT_IP'

docker-test:
	docker buildx build --load --target mpc-test \
		-f docker/Dockerfile -t $(DOCKER_IMAGE):mpc-test .
	$(MAKE) docker-build-dcp3 DOCKER_IMAGE=$(DOCKER_IMAGE)
	docker run --rm $(DOCKER_IMAGE):mpc-test bash -lc 'source /opt/ros/jazzy/setup.bash; source /opt/robot-teleoperation-venv/bin/activate; source /opt/robot_teleoperation_framework/install/setup.bash; python3 -m colcon test-result --verbose'
	docker run --rm $(DOCKER_IMAGE):dcp3 python3 -m colcon test-result --verbose

docker-run:
	@mkdir -p "$(REPO_ROOT)/.docker-data/robot-mpc"
	docker run --rm -it --name robot-teleoperation --net=host --ipc=host \
		--user "$$(id -u):$$(id -g)" -e HOME=/tmp -e ROS_DOMAIN_ID=77 \
		-e FORCE_COLCON_BUILD=$(FORCE_COLCON_BUILD) \
		-v "$(REPO_ROOT):/workspace" \
		-v "$(REPO_ROOT)/.docker-data/robot-mpc:/var/lib/robot-teleoperation-dev" \
		-w /workspace \
		$(DOCKER_IMAGE):$(DOCKER_TAG) \
		bash /workspace/docker/dev-shell.sh /var/lib/robot-teleoperation-dev \
		$(DOCKER_BASE_PATHS) -- bash

docker-up-robot:
	@test -n "$(ROBOT_IP)" || (echo "Set ROBOT_IP to the robot controller address"; exit 2)
	@test -n "$$DISPLAY" || (echo "DISPLAY is not set; run this from a desktop session with X11 support"; exit 2)
	@set -e; \
		xauthority_file="$${XAUTHORITY:-$$HOME/.Xauthority}"; \
		test -r "$$xauthority_file" || { echo "X11 authorization file is not readable: $$xauthority_file"; exit 2; }; \
		if ! docker image inspect "$(DOCKER_IMAGE):mpc-gui" >/dev/null 2>&1; then \
			$(MAKE) docker-build-gui DOCKER_IMAGE=$(DOCKER_IMAGE); \
		fi; \
		if ! docker image inspect "$(DOCKER_IMAGE):dcp3" >/dev/null 2>&1; then \
			$(MAKE) docker-build-dcp3 DOCKER_IMAGE=$(DOCKER_IMAGE); \
		fi; \
		mkdir -p "$(REPO_ROOT)/.docker-data/robot-mpc/profiles" "$(REPO_ROOT)/.docker-data/robot-dcp3"; \
		ROBOT_TELEOP_MPC_IMAGE=$(DOCKER_IMAGE):mpc-gui \
		ROBOT_DCP3_IMAGE=$(DOCKER_IMAGE):dcp3 \
		ROBOT_IP=$(ROBOT_IP) \
		LOCAL_UID="$$(id -u)" LOCAL_GID="$$(id -g)" \
		XAUTHORITY_FILE="$$xauthority_file" \
		docker compose -f docker/compose.robot.yaml up -d; \
		echo "Dev containers are ready. Enter with: make docker-shell-robot ROBOT_IP=$(ROBOT_IP) ROBOT_SERVICE=mpc (or dcp3)"

docker-shell-robot:
	@test -n "$(ROBOT_IP)" || (echo "Set ROBOT_IP to the robot controller address"; exit 2)
	@case "$(ROBOT_SERVICE)" in mpc|dcp3) ;; *) echo "ROBOT_SERVICE must be mpc or dcp3"; exit 2 ;; esac
	ROBOT_IP=$(ROBOT_IP) docker compose -f docker/compose.robot.yaml exec \
		-e FORCE_COLCON_BUILD=$(FORCE_COLCON_BUILD) $(ROBOT_SERVICE) \
		bash /workspace/docker/dev-shell.sh /var/lib/robot-teleoperation-dev \
		$(if $(filter $(ROBOT_SERVICE),mpc),$(DOCKER_BASE_PATHS),$(DOCKER_DCP3_BASE_PATHS)) -- bash

docker-launch-robot-dcp3:
	@test -n "$(ROBOT_IP)" || (echo "Set ROBOT_IP to the robot controller address"; exit 2)
	ROBOT_IP=$(ROBOT_IP) docker compose -f docker/compose.robot.yaml exec \
		-e FORCE_COLCON_BUILD=$(FORCE_COLCON_BUILD) -e ROBOT_IP="$(ROBOT_IP)" dcp3 \
		bash /workspace/docker/dev-shell.sh /var/lib/robot-teleoperation-dev \
		$(DOCKER_DCP3_BASE_PATHS) -- bash -lc 'exec 9>/tmp/robot_dcp3.lock; \
		flock -n 9 || { echo "A robot DCP3 driver is already running." >&2; exit 2; }; \
		exec ros2 launch robot_dcp3 robot_dcp3.launch.py robot_ip:="$$ROBOT_IP"'

docker-down-robot:
	ROBOT_IP="$${ROBOT_IP:-127.0.0.1}" docker compose -f docker/compose.robot.yaml down

# Simulation-only workflow. The loopback IP satisfies Compose's required
# DCP3 environment variable; the DCP3 service is never started by these targets.
docker-up-robot-sim:
	@test -n "$$DISPLAY" || (echo "DISPLAY is not set; run this from a desktop session with X11 support"; exit 2)
	@set -e; \
		xauthority_file="$${XAUTHORITY:-$$HOME/.Xauthority}"; \
		test -r "$$xauthority_file" || { echo "X11 authorization file is not readable: $$xauthority_file"; exit 2; }; \
		if ! docker image inspect "$(DOCKER_IMAGE):mpc-gui" >/dev/null 2>&1; then \
			$(MAKE) docker-build-gui DOCKER_IMAGE=$(DOCKER_IMAGE); \
		fi; \
		mkdir -p "$(REPO_ROOT)/.docker-data/robot-mpc/profiles"; \
		ROBOT_TELEOP_MPC_IMAGE=$(DOCKER_IMAGE):mpc-gui \
		ROBOT_DCP3_IMAGE=$(DOCKER_IMAGE):dcp3 \
		ROBOT_IP=127.0.0.1 \
		LOCAL_UID="$$(id -u)" LOCAL_GID="$$(id -g)" \
		XAUTHORITY_FILE="$$xauthority_file" \
		docker compose -f docker/compose.robot.yaml up -d mpc; \
		echo "Simulation MPC container is ready. DCP3 and hardware driver are not started."

docker-shell-robot-sim:
	ROBOT_TELEOP_MPC_IMAGE=$(DOCKER_IMAGE):mpc-gui \
	ROBOT_DCP3_IMAGE=$(DOCKER_IMAGE):dcp3 \
	ROBOT_IP=127.0.0.1 \
	docker compose -f docker/compose.robot.yaml exec \
		-e FORCE_COLCON_BUILD=$(FORCE_COLCON_BUILD) mpc \
		bash /workspace/docker/dev-shell.sh /var/lib/robot-teleoperation-dev \
		$(DOCKER_BASE_PATHS) -- bash

docker-launch-robot-sim:
	ROBOT_TELEOP_MPC_IMAGE=$(DOCKER_IMAGE):mpc-gui \
	ROBOT_DCP3_IMAGE=$(DOCKER_IMAGE):dcp3 \
	ROBOT_IP=127.0.0.1 \
	docker compose -f docker/compose.robot.yaml exec \
		-e FORCE_COLCON_BUILD=$(FORCE_COLCON_BUILD) -e LIBGL_ALWAYS_SOFTWARE=1 mpc \
		bash /workspace/docker/dev-shell.sh /var/lib/robot-teleoperation-dev \
		$(DOCKER_BASE_PATHS) -- bash -lc 'exec 9>/tmp/robot_mpc_sim.lock; \
		flock -n 9 || { echo "A robot simulation launch is already active. Stop it before starting another." >&2; exit 2; }; \
		exec ros2 launch robot_mpc robot_mpc.launch.py simulate:=true \
		initialize_target_from_current_state:=true rviz:=true plot:=true teleop_gui:=true \
		diagnostic_log_directory:=/tmp/robot_mpc_logs'

docker-launch-robot-sim-minimal:
	ROBOT_TELEOP_MPC_IMAGE=$(DOCKER_IMAGE):mpc-gui \
	ROBOT_DCP3_IMAGE=$(DOCKER_IMAGE):dcp3 \
	ROBOT_IP=127.0.0.1 \
	docker compose -f docker/compose.robot.yaml exec \
		-e FORCE_COLCON_BUILD=$(FORCE_COLCON_BUILD) -e LIBGL_ALWAYS_SOFTWARE=1 mpc \
		bash /workspace/docker/dev-shell.sh /var/lib/robot-teleoperation-dev \
		$(DOCKER_BASE_PATHS) -- bash -lc 'exec 9>/tmp/robot_mpc_sim.lock; \
		flock -n 9 || { echo "A robot simulation launch is already active. Stop it before starting another." >&2; exit 2; }; \
		exec ros2 launch robot_mpc robot_mpc.launch.py simulate:=true \
		initialize_target_from_current_state:=true rviz:=true plot:=false teleop_gui:=false teleop_minimal_gui:=true vive_mapping:=true mpc_start_paused:=true \
		diagnostic_log_directory:=/tmp/robot_mpc_logs'

docker-keyboard-robot-sim:
	ROBOT_TELEOP_MPC_IMAGE=$(DOCKER_IMAGE):mpc-gui \
	ROBOT_DCP3_IMAGE=$(DOCKER_IMAGE):dcp3 \
	ROBOT_IP=127.0.0.1 \
	docker compose -f docker/compose.robot.yaml exec \
		-e FORCE_COLCON_BUILD=$(FORCE_COLCON_BUILD) mpc \
		bash /workspace/docker/dev-shell.sh /var/lib/robot-teleoperation-dev \
		$(DOCKER_BASE_PATHS) -- ros2 run robot_mpc keyboard_teleop --ros-args \
		-p initialize_from_current_pose:=true

docker-down-robot-sim:
	ROBOT_TELEOP_MPC_IMAGE=$(DOCKER_IMAGE):mpc-gui \
	ROBOT_DCP3_IMAGE=$(DOCKER_IMAGE):dcp3 \
	ROBOT_IP=127.0.0.1 \
	docker compose -f docker/compose.robot.yaml stop mpc

docker-run-gui:
	@test -n "$$DISPLAY" || { echo "DISPLAY is not set; run this from an X11 desktop session"; exit 1; }
	@set -e; \
		xauthority_file="$${XAUTHORITY:-$$HOME/.Xauthority}"; \
		test -r "$$xauthority_file" || { echo "X11 authorization file is not readable: $$xauthority_file"; exit 1; }; \
		if ! docker image inspect "$(DOCKER_IMAGE):mpc-gui" >/dev/null 2>&1; then \
			docker buildx build --load --target mpc-gui \
				-f docker/Dockerfile -t "$(DOCKER_IMAGE):mpc-gui" .; \
		fi; \
		mkdir -p "$(REPO_ROOT)/.docker-data/robot-mpc/profiles"; \
		docker run --rm -it --name robot-teleoperation --net=host --ipc=host \
			--user "$$(id -u):$$(id -g)" -e HOME=/tmp -e ROS_DOMAIN_ID=77 \
			-e FORCE_COLCON_BUILD=$(FORCE_COLCON_BUILD) \
			-e "DISPLAY=$$DISPLAY" \
			-e XAUTHORITY=/tmp/.Xauthority \
			-e QT_X11_NO_MITSHM=1 \
			-e LIBGL_ALWAYS_SOFTWARE="$${LIBGL_ALWAYS_SOFTWARE:-1}" \
			-e ROBOT_TELEOP_MAPPING=/var/lib/robot-teleoperation-dev/profiles/indy7_joint_mapping.yaml \
			-v "$$xauthority_file:/tmp/.Xauthority:ro" \
			-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
			-v "$(REPO_ROOT):/workspace" \
			-v "$(REPO_ROOT)/.docker-data/robot-mpc:/var/lib/robot-teleoperation-dev" \
			-v "$(REPO_ROOT)/.docker-data/robot-mpc/profiles:/var/lib/robot-teleoperation-dev/profiles:rw" \
			-w /workspace \
			$(DOCKER_IMAGE):mpc-gui \
			bash /workspace/docker/dev-shell.sh /var/lib/robot-teleoperation-dev \
			$(DOCKER_BASE_PATHS) -- bash
