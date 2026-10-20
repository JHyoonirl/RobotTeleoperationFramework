# Indy7 hand target teleoperation and pose mapping

The URDF defines `hand_tcp` as a child of `tcp`. Its local translation and RPY
are read from the `hand_tcp_fixed` joint in
[`indy7.urdf`](../indy7_description/urdf/indy7.urdf); changes to that mount offset
are included in the model on restart. Translation is expressed in the original
TCP axes. Manual MPC uses `tcp` by default. Enabling Vive
simulation following selects `hand_tcp` as the MPC task frame.

`indy7_vive_mapper` synchronizes the current Vive hand pose with the current
robot `hand_tcp` pose, then expresses subsequent Vive poses in Indy7 `world`.
The mapper itself publishes pose and TF previews. The standalone mapping
launch does not start MPC. The combined minimal simulation launch adds the
F7-controlled MPC following described below.

## Keyboard and Topic Pose modes

The minimal GUI's **Hand Target Teleoperation** panel selects exactly one input
mode. Both modes control `hand_tcp` toward the same `hand_target`; F7 must remain
held for MPC execution. Editing or receiving a target while paused updates the
preview without moving the simulation.

| Input mode | Target editing | F5 |
| --- | --- | --- |
| Keyboard | Keys or XYZ/RPY jog buttons, with translation/rotation steps | Reset the target to the current `hand_tcp` pose |
| Topic Pose | Select a `PoseStamped` topic, click Apply Topic, then map it | Align the input pose to the current `hand_tcp` pose |

**Keyboard** needs simulation feedback only; no Tracker, input topic, or mapper
readiness is required. Entering the mode seeds the hand target from the current
hand pose. W/S change world X, A/D world Y, R/F world Z, U/O roll, I/K pitch, and
J/L yaw about world axes. Rotations keep the hand target's origin fixed. The jog
keys work after clicking a GUI control and can edit the target with F7 released
or held. Uncheck **Keyboard** to disable key jogging while retaining jog buttons.

**Topic Pose** retains the Vive mapping workflow and also accepts other selected
`geometry_msgs/msg/PoseStamped` sources. The mapping YAML's `input_frame`
(default `world`) must match the source header; source coordinates are aligned
by F5. Keyboard jogs cannot change the topic target. Topic loss or mapper changes
stop Topic Pose execution, while an active Keyboard session remains independent.

Release F7 before changing modes. Mode changes pause MPC and discard cached
targets, then require a new F7 press. Input mode starts as Topic Pose unless
`teleop_input_mode:=keyboard` is set on the MPC node. The integrated launch enables
the hand controller with `vive_mapping:=true`, including when only Keyboard is
used; the Tracker publisher can remain stopped.

The selected goal is published on `/indy7/hand_target_pose`, and TF publishes
`world -> hand_target`. The default RViz template includes **Selected Hand
Target**. An existing saved RViz layout can show `hand_target` through TF, or add
a Pose display for that topic without replacing the saved layout. Keyboard input
uses `/indy7/keyboard/hand_target_pose`; mapped topic input retains
`/indy7/vive/hand_target_pose` and the `vive_hand_target` preview frame. Only the
selected input can reach MPC. Robot joint mapping and DCP3 controls remain in
their existing GUI sections.

## Run in Docker

Build the updated description and new mapper entry point in the existing MPC
container (from the framework root):

```bash
FORCE_COLCON_BUILD=1 make docker-shell-indy7-sim
```

## World Z floor

The MPC keeps the controlled `task_frame` at **world Z >= 0.10 m** by default.
In the minimal GUI this frame is `hand_tcp`, for both Keyboard and Topic Pose.
The Desired TCP and shared `hand_target` remain the exact requested poses, even
below the floor. Only the private MPC tracking/path reference projects a lower
Z goal up to 0.101 m; its X/Y and orientation still follow the requested pose.
Each running and terminal MPC stage also has a height penalty with 1 mm
clearance to guide feasible motion. The final joint-position filter certifies
the full joint-linear command step, including poses between its endpoints, and
shortens or holds a command that would cross the hard floor. Existing joint
position, rate, increment, and Cartesian path limits remain active.

Configure these startup parameters in `config/indy7_mpc.yaml` and restart the
launch after editing them:

```yaml
workspace_floor_enabled: true
workspace_min_z: 0.10             # metres in URDF world
workspace_floor_margin: 0.001     # planning clearance above the hard plane
workspace_floor_cost_weight: 1000000.0
```

F6 rejects alignment to robot joints whose controlled frame is already below
the floor. F7 also rejects a measured starting pose below the floor; direct MPC
execution holds measured joints rather than automatically moving out of that
state. Command status reports `workspace_floor_limited`, `workspace_floor_hold`,
or `workspace_floor_violation` when applicable. The limit concerns the TCP point,
not every robot/hand link. It bounds MPC commands; physical tracking error and
motion generated outside this MPC are not covered by this check.

## F6 robot connection (DCP3 TELE_OP)

Start the hardware adapter alongside the simulation workflow. Replace the example
IP below with the actual controller address. Stop any previous simulation/driver
launch before restarting updated nodes; the Vive publisher can stay running.

```bash
make docker-up-indy7 ROBOT_IP=192.168.0.7
```

Terminal 1 starts DCP3 feedback in the disarmed state:

```bash
FORCE_COLCON_BUILD=1 make docker-launch-indy7-dcp3 ROBOT_IP=192.168.0.7
```

Terminal 2 starts the simulation and GUI:

```bash
FORCE_COLCON_BUILD=1 make docker-launch-indy7-sim-minimal
```

Select Keyboard or configure/map Topic Pose with F5. Then press **F6 Connect
Robot**. F6 pauses MPC, copies measured robot joints through the saved joint
mapping into the simulation, confirms the resulting joint alignment, and
re-anchors the selected hand target. Topic Pose is mapped again after alignment;
Keyboard is reset to the newly measured hand pose. The GUI then supplies a
measured-position hold and a live Deadman, calls `/indy7_dcp3/arm`, and waits for
the adapter to confirm actual `TELE_OP`. The adapter's IDLE, URDF-limit, command,
and feedback checks remain active. F6 is unavailable until fresh robot feedback
and the adapter's arm service are present. Topic Pose also needs its input ready;
Keyboard does not need a Tracker.

The status becomes **Robot TELE_OP ON** only after that confirmation. F6 alone
does not run MPC or move toward a staged goal. **Hold F7** to move the simulation
and send mapped joint commands to the robot. **Release F7** to pause MPC and hold
the measured robot joints, including when hardware is lagging behind the
simulation. The next F7 press re-aligns simulation joints to fresh hardware
feedback before continuing toward the staged hand target. Releasing during that
alignment cancels the start. The robot connection stays on between presses.
**F6 Disconnect Robot** or
**Disarm / Stop** disables output and calls DCP3 stop/disarm. Changing the input
mode, input topic, or joint mapping requires disconnecting first.

F6 also works after clicking a child control and ignores OS key repeat. A lost
GUI focus, closed/hidden window, stale feedback, changed controller session, or
released managed Deadman stops the robot connection; reconnect explicitly with
F6. The managed Deadman is renewed by Qt, with a 0.2 s GUI lease plus the adapter's
own configured timeout, so a stalled GUI cannot be kept armed by the ROS thread.
Stop RPC failures are displayed as **F6 Retry Stop**, with joint output disabled.
The existing stream/checkbox/Arm controls remain available for manual operation
when an F6 session is off.

The connection tests run the production GUI, MPC, and DCP3 adapter using a fake
vendor SDK in isolated ROS domains. They verify no movement before F7, alignment
and start rejections, feedback/focus/GUI-heartbeat loss, and stop retries. They do
not exercise the physical robot.

If Indy7 simulation is already running, stop and restart its launch after the
build so `robot_state_publisher` loads the URDF with `hand_tcp`. Keep the Vive
publisher running in its own container. Both containers use `ROS_DOMAIN_ID=77`.

In a sourced MPC shell, run:

```bash
ros2 launch indy7_mpc indy7_vive_mapping.launch.py gui:=true
```

The default input is `/tracker/right/pose`, matching the current Vive example YAML.
Override it if your tracker uses a different namespace:

```bash
ros2 launch indy7_mpc indy7_vive_mapping.launch.py input_pose_topic:=/vive/tracker/pose
```

In another MPC shell, confirm that the robot hand frame exists, hold the robot
and Vive hand pose still, and synchronize:

```bash
ros2 run tf2_ros tf2_echo world hand_tcp
# Ctrl+C after checking the frame, then:
ros2 service call /indy7/vive_mapping/sync std_srvs/srv/Trigger '{}'
ros2 topic echo /indy7/vive/hand_target_pose --once
```

`success: true` means the reference was captured. At that instant the mapped
pose equals the current robot hand TCP. Subsequent motion is mapped 1:1.
Synchronize again after moving/recalibrating the Lighthouse setup or restarting
the mapper. The alignment is held in memory for this run.

| Output | Meaning |
| --- | --- |
| `/indy7/hand_tcp/current_pose` | Current robot hand TCP from TF, in `world` |
| `/indy7/vive/hand_target_pose` | Mapped Vive hand target, in `world` |
| `world -> tcp -> hand_tcp` | Robot frame chain from robot_state_publisher |
| `world -> vive_world -> vive_hand_target` | Synchronized Vive frame chain |

The existing Indy7 RViz configuration includes `Hand TCP` and `Vive Hand Target`
pose displays. Missing robot TF, invalid quaternions, stale input messages, and
stale robot TF cause synchronization to fail without replacing the prior
alignment. Input timeouts measure ROS message delivery, not Lighthouse tracking
quality; verify libsurvive calibration and actual pose updates before syncing.

## GUI mapping and F5

The mapping panel is available in the minimal simulation GUI and as a standalone
window. All labels are in English. It shows the actual mapper input topic, Vive
message availability, robot hand TF availability, current hand pose, mapped
target position/orientation, and the synchronization response. Pose displays
use metres and RPY degrees. F5 works throughout the active GUI window, including
after clicking a child control. Requests are asynchronous; repeated requests
are blocked while a response is pending, and timeouts are reported in the GUI.

Use **Vive pose topic** to enter an absolute topic name or choose a discovered
`geometry_msgs/msg/PoseStamped` topic. **Refresh Topics** updates the list and
preserves any text you entered. Click **Apply Topic** to change the live mapper
subscription without restarting the simulation or the Vive publisher. A new
topic can be applied before its publisher starts; the panel waits for input.

Changing the topic clears the previous Vive sample and mapping. Wait for the
new input to arrive, then press F5 to synchronize again. An unapplied topic edit
disables F5 so it cannot accidentally synchronize the old source. Typing in the
topic field does not trigger the simulation's letter-key jogging controls.
Invalid topic names, known incompatible message types, and the mapper's own
output topics are rejected while preserving the active subscription/mapping.
Live topic changes apply to the current mapper run. To retain a startup topic,
set it through the launch input-topic argument or mapping YAML for direct node
execution; restarting a launch restores that launch's input-topic argument.

From the framework root, start the combined simulation workflow:

```bash
make docker-up-indy7-sim
FORCE_COLCON_BUILD=1 make docker-launch-indy7-sim-minimal
```

This starts the simulator, minimal GUI, and mapper together. Keep the Vive ROS
publisher running in its separate container. Do not also start a second mapper
with `indy7_vive_mapping.launch.py`. Hold the robot and Tracker poses still,
then click **Map / Sync Current Pose (F5)**. The button becomes available only
when the mapper, synchronization service, Vive input, and robot hand TF are
ready. No terminal service call is needed. Mapper restarts and stale inputs
are reflected in the panel rather than leaving an old ready indication.

The minimal target also sets `mpc_start_paused:=true`. The MPC process loads its
model and publishes current-pose telemetry, but does not optimize or stream
joint commands until F7. After F5 synchronization, **hold F7** to follow the
mapped Vive pose; **release F7** to pause. The **Hold to Run (F7)** button has
the same press-and-hold behavior. The key works after clicking a child input
or control. OS key repeat does not toggle or release the run state. Losing
window/application focus, closing/hiding the window, or **Disarm / Stop** also
pauses following.
No robot driver connection, Arm, Deadman, or mapped joint stream is needed for
this simulation test. F7 does not connect or arm a physical robot.

With Vive following enabled, the MPC controls `hand_tcp` directly toward
`vive_hand_target`: `T_world_hand_tcp_goal = T_world_vive_hand_target`.
The SE(3) residual, Jacobian, Cartesian reference path, and path limits are all
evaluated at `hand_tcp`. Rotation-only input therefore rotates the hand about
the mapped hand origin. Pinocchio already includes the URDF's fixed TCP-to-hand
offset; the target is not transformed or offset a second time. The effective
`task_frame` ROS parameter and the GUI show `hand_tcp`, and `/mpc/current_pose`,
`/mpc/desired_pose`, and `/mpc/cartesian_reference_pose` describe that frame in
`world`. The resulting joint commands use the simulation controller's existing
position, velocity, and increment limits.

Pause before editing the input topic, remapping with F5, or using the manual
simulation TCP controls. Input loss, stale robot state, mapper restarts, or
mapping changes stop Vive following and pause the solver. Recovery requires
releasing F7 and holding it again. The input timeout checks ROS delivery/timestamps; Lighthouse
optical tracking quality still needs to be verified separately.

The GUI sends a hold heartbeat from its Qt event loop every 50 ms. If that
signal stops for `vive_hold_timeout` (default 0.35 s), the simulation pauses
even when Vive and ROS publishers continue running. Returning from an input
fault or GUI stall cannot resume motion while the old press remains held;
release and press again. `/indy7/sim/vive_mpc_hold` carries a `std_msgs/String`
JSON message with `held` and a per-press `token`. A new held token starts
following, further held messages renew it, and a matching release pauses.
The status echoes the token so a delayed response from an older press cannot
cancel a new press. The run service also requires a fresh hold for a true request;
false requests always pause.

For an existing simulation without a mapper, open the standalone mapping GUI
from a sourced MPC shell instead:

```bash
ros2 launch indy7_mpc indy7_vive_mapping.launch.py gui:=true
```

If a current-version mapper is already running, open only its GUI client:

```bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ros2 run indy7_mpc indy7_vive_mapping_gui
```

Both launch files run the mapper with UDPv4 to avoid cross-container shared
memory issues. For a different Vive topic in the combined launch, use
`vive_mapping:=true vive_input_pose_topic:=/your/tracker/pose` with
`indy7_mpc.launch.py`. The standalone mapper launch uses `input_pose_topic`.

The shortcut sequence is shown explicitly in the panel:

| Shortcut | Current behavior |
| --- | --- |
| F5 | Synchronize the current Vive pose to the current robot hand TCP |
| F6 | Align the simulation and enable DCP3 TELE_OP; press again to disconnect |
| F7 | Hold to follow the selected hand target in the simulation MPC; release to pause |

F5 prepares the selected hand target; F6 controls robot teleoperation and F7
holds execution on. Other launch
workflows keep their prior immediate-MPC behavior unless `mpc_start_paused:=true`
is explicitly selected. To enable F7 in a direct combined launch, use
`vive_mapping:=true mpc_start_paused:=true` with `simulate:=true`.

The simulation runtime publishes JSON at `/indy7/sim/vive_mpc_status`. Its
`SetBool` service `/indy7/sim/vive_mpc_run` accepts `true` to start and `false` to
pause. These endpoints are created only in task-space simulation with
`enable_vive_follow` enabled. The GUI shows readiness and stop reasons from
this runtime rather than depending on DCP3 feedback.

The mapper publishes versioned JSON status at 5 Hz on
`/indy7/vive_mapping/status` (`std_msgs/msg/String`), including input/TF readiness,
calibration state, a mapper session ID, and a successful-sync sequence number.
An input revision changes whenever the selected topic changes, allowing GUI
clients to discard previews from the previous source.
This lets the same panel be embedded in other GUIs without owning robot output.

## Persistent RViz layout

Docker development shells set `INDY7_RVIZ_CONFIG` to a file in the writable
development directory. The launch creates it from the packaged default only
when it does not exist. The initial window is 1000 x 720 at (40, 40); it does
not restore the old oversized dock state. Existing saved layouts are never
overwritten by launch, image rebuilds, or colcon builds.

| Location | RViz configuration |
| --- | --- |
| Host, relative to the framework root | `.docker-data/indy7-mpc/rviz/indy7.rviz` |
| MPC container | `/var/lib/robot-teleoperation-dev/rviz/indy7.rviz` |

Resize RViz and adjust the panels/camera, then use **File -> Save Config** or
**Ctrl+S** while RViz has focus. The host file updates through the existing
read/write directory mount. The next launch loads that same file, including
the saved window size, position, displays, and camera. Settings are local to
this checkout and ignored by Git. The packaged `config/indy7.rviz` is only the
initial template; editing that template does not replace an existing layout.

To choose a separate layout, pass an absolute writable container path to the
existing launch command:

```bash
ros2 launch indy7_mpc indy7_mpc.launch.py \
  rviz_config:=/var/lib/robot-teleoperation-dev/rviz/experiment.rviz
```

A missing selected file is seeded once. **Save Config As** can also create a
layout in this directory, but the next launch must select that filename with
`rviz_config`. Directory mounting is intentional so Qt can save by creating
and renaming a temporary file.

## Transform convention

Let `W` be Indy7 world, `V` the Lighthouse world, and `H` the hand frame. On sync:

```text
T_W_V = T_W_H(current robot) * inverse(T_V_H(current Vive))
T_W_H(mapped, t) = T_W_V * T_V_H(Vive, t)
```

This is a teleoperation reference alignment, not a measured physical registration
of the Lighthouse room to the robot base. It preserves a full rigid motion,
including orientation; translation is in metres.

The Vive input `/pose` already includes that driver's local translation/RPY hand
offset. The mapper does not apply it a second time. Its input `frame_id` is
currently also called `world`; `input_frame: world` in
`config/indy7_vive_mapping.yaml` explicitly treats that as a separate source
label. It is published as `vive_world` in TF after synchronization, so it is not
mistaken for an identity transform to the robot world.
