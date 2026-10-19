# Indy7 DCP3 Adapter

`indy7_dcp3` connects the Indy7 MPC to Neuromeka's IndyDCP3 client. It publishes actual joint
feedback as `/indy7_dcp3/joint_states` and forwards absolute joint positions from
`/indy7_dcp3/joint_position_cmd` in DCP3 `TELE_JOINT_ABSOLUTE` mode. MPC commands are radians; DCP3
receives degrees.

The adapter publishes `/indy7_dcp3/armed` and `/indy7_dcp3/status` for the GUI. Arm requests
return promptly and the 200 Hz feedback loop confirms `TELE_OP` asynchronously, so deadman and
command heartbeats continue to be processed while the controller changes modes. The GUI publishes
the mapped simulation target and refreshes `/indy7_dcp3/deadman` only while its hold button is down.

`velocity_ratio` and `acceleration_ratio` use DCP3's 0-100 scale: `20.0` means
20%. The adapter validates both values and passes them unchanged to
`movetelej_abs()`.

The adapter starts disarmed. Arming requires fresh DCP3 feedback, a fresh MPC command, a
continuously refreshed deadman, and the robot in IDLE. It checks the Indy7 URDF hard limits,
keeps a 0.05 rad soft-limit margin, and forwards validated joint targets unchanged at
200 Hz. The adapter does not apply an additional per-cycle joint velocity clip. A stale
command/deadman or invalid target stops teleoperation and requires a new arm request.

These software checks are not safety-rated. They do not provide collision monitoring, workspace
supervision, or a safety-rated E-stop. Keep the physical E-stop accessible and follow Neuromeka's
commissioning and safety procedures.

## Developer Run

The DCP3 image installs the Neuromeka Python client from
[`robots/indy7/requirements-hardware.txt`](../../../robots/indy7/requirements-hardware.txt).
It has its own Python runtime, separate from the MPC image's NumPy 2 / Crocoddyl stack. Build the
images initially or after Dockerfile/dependency changes, and run image tests when validating a
build. Ordinary mounted ROS source or YAML edits do not need these commands:

```bash
make docker-build
make docker-test
```

Start from a Linux desktop session with X11/XWayland available. This prepares idle MPC and DCP3
containers; no ROS nodes are launched automatically. The repository is mounted read-only, and
each container builds a persistent `--symlink-install` overlay from that mounted source on first
shell entry. Ordinary source edits are picked up when the affected node is restarted; image rebuilds
are only needed for Dockerfile, system package, or Python dependency changes.

```bash
make docker-up-indy7 ROBOT_IP=controller-address
```

Terminal 1: launch the hardware adapter. ROS, the Python venv, and the mounted source overlay are
sourced automatically when the shell opens.

```bash
make docker-shell-indy7 ROBOT_IP=controller-address INDY7_SERVICE=dcp3
ros2 launch indy7_dcp3 indy7_real_teleop.launch.py robot_ip:=$INDY7_ROBOT_IP
```

Terminal 2: launch MPC with RViz and the live diagnostics plot. Its ROS environment is also sourced
on shell entry.

```bash
make docker-shell-indy7 ROBOT_IP=controller-address INDY7_SERVICE=mpc
ros2 launch indy7_mpc indy7_mpc.launch.py simulate:=false \
  initialize_target_from_current_state:=true rviz:=true plot:=true \
  diagnostic_log_directory:=/tmp/indy7_mpc_logs
```

RViz shows the real joint state, measured TCP, and teleoperation target. The DCP3 node starts
disarmed. Starting either container or changing the keyboard target does not arm or move the robot.

Terminal 3: open another MPC shell and run the keyboard node manually.

```bash
make docker-shell-indy7 ROBOT_IP=controller-address INDY7_SERVICE=mpc
ros2 run indy7_mpc keyboard_teleop --ros-args -p initialize_from_current_pose:=true
```

Python, launch, and config changes in the mounted source do not require an image rebuild. For
package-layout or dependency metadata changes that require colcon, force the corresponding overlay
to rebuild on shell entry:

```bash
make docker-shell-indy7 ROBOT_IP=controller-address INDY7_SERVICE=mpc FORCE_COLCON_BUILD=1
make docker-shell-indy7 ROBOT_IP=controller-address INDY7_SERVICE=dcp3 FORCE_COLCON_BUILD=1
```

Do not repeat colcon build on every shell entry.

The keyboard node accepts `w/s=x`, `a/d=y`, `r/f=z`, `u/o=roll`, `i/k=pitch`, `j/l=yaw`,
`space=reset target`, and `q=quit`. It only changes the MPC target. It does not arm or move the
robot.

For ROS graph and feedback inspection, open another sourced shell:

```bash
make docker-shell-indy7 ROBOT_IP=controller-address INDY7_SERVICE=dcp3
ros2 topic echo /indy7_dcp3/joint_states --once
ros2 topic echo /indy7_dcp3/deadman --once
```

In another MPC shell, inspect the measured pose:

```bash
make docker-shell-indy7 ROBOT_IP=controller-address INDY7_SERVICE=mpc
ros2 topic echo /mpc/current_pose --once
```

The initial measured TCP pose is copied into the keyboard target only once. After that, the target
changes only on a key press.

Terminal 4: keep this deadman publisher running continuously at 10 Hz. It is a ROS heartbeat,
not a physical deadman switch or E-stop. If the messages stop, the adapter's 0.3 second timeout
stops teleoperation and disarms:

```bash
make docker-shell-indy7 ROBOT_IP=controller-address INDY7_SERVICE=dcp3
ros2 topic pub -r 10 /indy7_dcp3/deadman std_msgs/msg/Bool "{data: true}"
```

Before arming, inspect the measured joints, measured TCP, desired TCP, and latest MPC joint
command from separate shells. Check that the target is still near the measured pose, the robot is
in IDLE, the work area is clear, and the physical E-stop is immediately reachable:

```bash
make docker-shell-indy7 ROBOT_IP=controller-address INDY7_SERVICE=dcp3
ros2 topic echo /indy7_dcp3/joint_states --once
ros2 topic echo /indy7_dcp3/deadman --once
```

```bash
make docker-shell-indy7 ROBOT_IP=controller-address INDY7_SERVICE=mpc
ros2 topic echo /mpc/current_pose --once
ros2 topic echo /mpc/desired_pose --once
ros2 topic echo /indy7_dcp3/joint_position_cmd --once
```

Terminal 5: leave the deadman publisher running in Terminal 4, then explicitly arm from another
DCP3 shell. Arming requires fresh DCP3 feedback, a fresh MPC command, a live deadman, IDLE state,
and a valid joint target. Only after arming can the MPC's absolute joint target reach the robot:

```bash
make docker-shell-indy7 ROBOT_IP=controller-address INDY7_SERVICE=dcp3
ros2 service call /indy7_dcp3/arm std_srvs/srv/SetBool "{data: true}"
```

To stop normally, first open another DCP3 shell and explicitly disarm. Then stop keyboard, MPC,
and DCP3 launch processes with `Ctrl+C`; stop the deadman publisher last, and finally remove the
idle containers. Do not use the deadman timeout as a substitute for the normal disarm command:

```bash
make docker-shell-indy7 ROBOT_IP=controller-address INDY7_SERVICE=dcp3
ros2 service call /indy7_dcp3/arm std_srvs/srv/SetBool "{data: false}"
```

Stop each manually launched ROS process with `Ctrl+C`, then remove the idle containers with
`make docker-down-indy7`.

The MPC, ros2_control simulation, and DCP3 feedback/command loop are configured at 200 Hz
(`dt: 0.005`, `update_rate: 200`, `publish_rate: 200.0`). The current MPC YAML has
`cartesian_max_speed: 0.5` m/s and `cartesian_max_angular_speed: 3.0` rad/s.
The adapter's `max_joint_velocity` setting and per-cycle clipping have been removed;
joint speed/increment limits remain in the MPC. DCP3's `velocity_ratio: 20.0` and
`acceleration_ratio: 20.0` remain active. The Cartesian limit is a planned reference,
not a measured TCP speed guarantee. Start by
changing only `cartesian_max_speed` in the host-mounted YAML, validate in simulation, then restart
MPC and measure the actual TCP response. This source/config edit does not require rebuilding the
Docker image.

The adapter uses the IndyDCP3 mode and operation-state conventions from Neuromeka's
[official Indy ROS 2 driver](https://github.com/neuromeka-robotics/indy-ros2/tree/jazzy-indyDCP3).
The physical robot has not been exercised by software tests.
