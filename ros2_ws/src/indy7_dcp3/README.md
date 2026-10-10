# Indy7 DCP3 Adapter

`indy7_dcp3` connects the Indy7 MPC to Neuromeka's IndyDCP3 client. It publishes actual joint
feedback as `/joint_states` and forwards absolute joint positions from
`/mpc/joint_position_cmd` in DCP3 `TELE_JOINT_ABSOLUTE` mode. MPC commands are radians; DCP3
receives degrees.

The adapter starts disarmed. Arming requires fresh DCP3 feedback, a fresh MPC command, a
continuously refreshed deadman, and the robot in IDLE. It checks the Indy7 URDF hard limits,
keeps a 0.05 rad soft-limit margin, and caps each commanded joint-position update to the
equivalent of 0.05 rad/s at the configured publish rate by default. A stale
command/deadman or invalid target stops teleoperation and requires a new arm request.

These software checks are not safety-rated. They do not provide collision monitoring, workspace
supervision, or a safety-rated E-stop. Keep the physical E-stop accessible and follow Neuromeka's
commissioning and safety procedures.

## Build and Start

The DCP3 image installs the Neuromeka Python client from
[`robots/indy7/requirements-hardware.txt`](../../../robots/indy7/requirements-hardware.txt).
It has its own Python runtime, separate from the MPC image's NumPy 2 / Crocoddyl stack. Build and
test both images:

```bash
make docker-build
make docker-test
```

Start the MPC and DCP3 services using the controller's actual IP:

```bash
make docker-up-indy7 ROBOT_IP=controller-address
```

The DCP3 adapter publishes feedback; the MPC runs with `simulate:=false` and initializes its
Cartesian target from the measured state. The DCP3 node starts disarmed. Startup does not arm or
move the robot. Verify `/joint_states` and `/mpc/current_pose` from another terminal:

In that second terminal, export the controller IP before using `docker compose` commands. Compose
validates `INDY7_ROBOT_IP` while loading the file, even for `exec`:

```bash
export INDY7_ROBOT_IP=controller-address
```

```bash
docker compose -f docker/compose.indy7.yaml exec dcp3 bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /opt/robot_teleoperation_framework/install/setup.bash && ros2 topic echo /joint_states --once'
docker compose -f docker/compose.indy7.yaml exec mpc bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /opt/robot_teleoperation_framework/install/setup.bash && ros2 topic echo /mpc/current_pose --once'
```

Only after confirming state, clearing the work area, and ensuring the physical E-stop is within
reach, start keyboard input in a separate terminal. The initial target is synchronized to the
measured TCP pose:

```bash
docker compose -f docker/compose.indy7.yaml exec mpc \
  bash -lc 'source /opt/ros/jazzy/setup.bash && source /opt/robot_teleoperation_framework/install/setup.bash && ros2 run indy7_mpc keyboard_teleop --ros-args -p initialize_from_current_pose:=true'
```

Keep a deadman publisher running separately. If it stops, the adapter disarms after 0.3 seconds:

```bash
docker compose -f docker/compose.indy7.yaml exec dcp3 \
  bash -lc 'source /opt/ros/jazzy/setup.bash && source /opt/robot_teleoperation_framework/install/setup.bash && ros2 topic pub -r 10 /indy7_dcp3/deadman std_msgs/msg/Bool "{data: true}"'
```

Only after verifying feedback, target, and deadman, explicitly arm the driver:

```bash
docker compose -f docker/compose.indy7.yaml exec dcp3 \
  bash -lc 'source /opt/ros/jazzy/setup.bash && source /opt/robot_teleoperation_framework/install/setup.bash && ros2 service call /indy7_dcp3/arm std_srvs/srv/SetBool "{data: true}"'
```

Stop with an explicit disarm request:

```bash
docker compose -f docker/compose.indy7.yaml exec dcp3 \
  bash -lc 'source /opt/ros/jazzy/setup.bash && source /opt/robot_teleoperation_framework/install/setup.bash && ros2 service call /indy7_dcp3/arm std_srvs/srv/SetBool "{data: false}"'
```

Then stop both containers with `make docker-down-indy7` or Ctrl+C in the attached Compose
terminal.

The adapter uses the IndyDCP3 mode and operation-state conventions from Neuromeka's
[official Indy ROS 2 driver](https://github.com/neuromeka-robotics/indy-ros2/tree/jazzy-indyDCP3).
The physical robot has not been exercised by software tests.
