# Robot hand target teleoperation and pose mapping

## EIR simulation

The official EIR model is available through a separate simulation launch:

```bash
make docker-up-robot-sim
FORCE_COLCON_BUILD=1 make docker-launch-eir-sim
# Optional single-arm mode after stopping the EIR launch:
make docker-stop-eir-sim
make docker-launch-eir-sim EIR_ARM=left
```

This uses ROS domain **78**, independently of the Indy7 domain 77, and starts
RViz plus the EIR hands GUI. **Both arms and torso are enabled by default**
(`EIR_ARM=both`). Check **Run MPC** or hold **F7** to follow the targets; uncheck
Run MPC or release F7 to pause. **F5** resets both hand targets to the
current poses. Logs go to `/tmp/eir_mpc_logs` inside the MPC container.
Run MPC stays active while switching to Isaac Sim; closing or hiding the EIR GUI
pauses motion.

The whole 18-axis robot is displayed. One shared MPC and position controller
command the two torso joints (`Joint_L0/L1`) and both seven-axis arms together;
the two head joints remain at their initial pose. The dual controller is configured
for `dt=0.05`, 10 stages (500 ms preview), and at most 6 solver iterations. It
sums both SE(3) tasks in one optimization, avoiding conflicting torso commands
from separate arm solvers. In the centered 100 mm square test, solve-time p95
was about 36 ms, below the configured 50 ms period.

Startup uses a symmetric simulation ready pose: torso/head zero, left arm
`[-30,-60,0,-45,0,0,0]` degrees, right arm `[30,60,0,45,0,0,0]` degrees in
`Joint_L2` through `Joint_L8` order. Both hands start about 318 mm forward,
639 mm apart, at 876 mm height. The URDF mock initial values, MPC goals and
preferred postures agree. This is a project posture, not a factory Home:
the [official EIR SRDF](https://github.com/neuromeka-robotics/indy-ros2/blob/ca0353f077620c877c1edff2e26c8cd976773f27/indy_moveit/srdf/indy_macro.srdf.xacro#L14-L43)
only supplies all-zero arm states. The ready pose avoids their straight-arm
singularity and passed a static nonadjacent collision-mesh check. Changing
the initial values takes effect when the simulation is restarted.

The EIR simulation deliberately widens both elbow joints (`Joint_L5_L/R`) to
approximately **-145 to +145 degrees** (`[-2.5307, 2.5307]` rad). This removes
the previous -60-degree stop for the inward arm motion. It is a user-requested
experimental override of the upstream `[-1.0472, 2.5307]` rad range, stored in
both the URDF joint limits and ros2_control interface bounds. It does not
establish the real robot's mechanical travel. Other joint, velocity, per-cycle
increment, path and floor limits retain their current settings.

The **Move both hands** checkbox applies the same world-coordinate jog to both
targets; uncheck it to edit them independently. The GUI only sets hand TCP goals.
The MPC chooses torso yaw/pitch with the arm joints while tracking both paths;
the GUI displays the measured torso angles. In `config/eir_dual_mpc.yaml`,
`torso_cost_weight` and `torso_pitch_cost_weight` prefer the first two entries
of `nominal_posture` (both zero by default). Pitch centering also uses motion left free by the two hand
tasks, so either torso axis can still move when a hand target requires it.
The YAML is checked every 0.25 s. Online settings include all objective weights,
`nominal_posture`, Cartesian speeds and ramps, joint velocity/increment bounds,
smoothing, acceleration/torque bounds, joint-limit margin, floor settings,
path tolerance and hard-filter enable, feedforward settings, short-move settings,
IK settings, diagnostic settings, hold thresholds, solver iteration cap and input
timeouts. The running model and ROS parameter values are updated together.
Each live revision is validated before application; invalid values preserve the
complete last valid live settings. Unknown parameter names are rejected.

Robot identity, frames, topics/services, simulation/startup flags, input mode,
`dt`, `horizon` and `prediction_model` require restart. Editing these produces
`MPC YAML restart required (not applied live)`; supported tuning changes in the
same file are still applied. Positive finite weights are required. Removing an
online field restores its declared default if that default is valid for this robot.

In RViz, choose **Interact** and use **Hand TCP controls** on either hand:
XYZ arrows and plane handles change position; the three rings change orientation
about world axes. Each marker edits only that hand's Desired TCP, even with the
GUI's Move both checkbox on (that checkbox governs button jogs). The left center
is cyan and the right center is orange. The GUI target values stay synchronized.
Dragging does not start motion: use **Run MPC** to follow or pause. The display
uses the Jazzy **Interactive Markers Namespace** `/eir/goal_markers`; select
that namespace in an InteractiveMarkers display when using an older custom
RViz profile. Its update topic is `/eir/goal_markers/update`.

`left_hand_tcp` and `right_hand_tcp` sit on the actual CAD terminal flanges:
`xyz=[0,0,0.097] m`, `rpy=[0,-1.57075,0] rad` relative to each `arm_link_7`.
This is the official EIR gripper attachment transform; the STL end face is at
97 mm, and TCP +X points out of the flange. No gripper length is assumed. RViz
shows these two frames and the hand targets instead of every joint axis.

Optional `EIR_ARM=left` or `right` retains single-arm operation with the same
terminal TCP frames; the other joints are then locked at their initial pose.

For a sourced ROS shell, the equivalent command is:

```bash
ROS_DOMAIN_ID=78 ros2 launch robot_mpc eir_sim.launch.py arm:=both
```

Use `rviz:=false gui:=false` for headless tests. Cost and command parameters
are in `config/eir_dual_mpc.yaml` (`eir_mpc.yaml` for the right arm and
`eir_left_mpc.yaml` for the left arm). The dual configuration uses soft path
tracking (`cartesian_path_limit_enabled: false`). Each running and terminal
action adds `0.5 * cartesian_path_cost_weight * (d_left^2 + d_right^2)`, where
each distance is from the predicted TCP to its fixed start-to-goal segment,
in metres. The explicit weight is **250000**, independent of the optional
hard tube tolerance. Older configurations without this weight retain
`1 / cartesian_path_tolerance^2` (2500 for 20 mm). This is a quadratic penalty,
not a guaranteed maximum deviation. The weight reloads online with the other
costs every 0.25 s; speed caps and the hard-filter flag also reload online.

A changed desired pose starts a new segment at that hand's latest measured
TCP. Identical repeated targets preserve the segment and its elapsed time.
Tracking error alone cannot move the segment in soft mode. Continuously
changing goals therefore generate continuously updated segments; they do not
define one permanent straight line. Joint velocity and smoothing can round a
corner after an abrupt target change.

In soft mode, the output limiter scales the joint step uniformly to respect
all per-joint rate bounds while preserving the MPC's joint direction. Physical
joint bounds and the floor guard remain enforced. Joint limits are hard at the
position-command boundary and margin-normalized soft costs in the MPC horizon.
When a joint-limit guard activates, it reports the affected joints but does not
reject a safe command merely because immediate hand tracking error rises.
BoxFDDP's predicted state limits remain soft costs. No DLS/MPC command switching
is used.

With `cartesian_path_limit_enabled: true`, the controller additionally projects
the command onto both hand paths with bounded linear least squares and checks
the complete joint-linear steps. `mirror_path_hold` means this optional output
filter could not certify a progressing step within its configured tolerance. It checks
`p_right - diag(1,-1,1) p_left` against its start-to-goal segment. For symmetric
goals this also limits one hand advancing ahead of the other. It is independent
of joint limits; reachable endpoints alone do not guarantee this path condition.

The dual EIR mock simulation uses `prediction_model: joint_acceleration`:
`qddot = u`, with acceleration inputs bounded to 40 rad/s² per joint. This
matches its position-command interface and is a kinematic MPC example; it
does not establish dynamic feasibility or motor torque compliance. The
`rigid_body` option retains ABA prediction with total torque inputs and URDF
effort bounds. Restart the node to change the prediction model or time grid;
configured input bounds and soft margins reload online.
In the recorded stuck pose, the rigid-body model needed over 50 Nm of torso
gravity compensation, exceeding its 50 Nm effort bound even at rest.

The objective keeps both hand pose tracking (position `60000`, rotation `1500`), velocity tracking (`0.1`),
acceleration input regularization (`0.0001`, configured through the historical
`torque_cost_weight` key), arm posture (`0.05`), torso yaw posture (`0.01`), and
torso pitch posture (`150`). The normalized joint-limit weight is `10.0` with a
0.15 rad soft margin;
command joint limits remain enabled and the hard Cartesian path filter is off.
Actual TCP speed depends on the configured reference speed, posture and joint-rate
bounds. A local ROS mock comparison using a `0.15 m/s` reference from the same
initial states reduced peak segment deviation from 19.88 to 0.59 mm for a
141 mm diagonal move and from 18.84 to 0.34 mm for a 150 mm sideways move.
Both reached 3 mm / 1 degree in about 1.1--1.2 s. A 300 mm forward move took
4.44 s with a final 2.37 mm error and minimum joint clearance 1.53 degrees.
The measured callback maximum across these runs was 41.1 ms, below the 50 ms
period; these finite measurements are not a real-time guarantee.
The optional hard path tube is configured to 20 mm and disabled by default.
Moves shorter than 10 mm additionally use the `short_move_speed` cap, bounded
by the general Cartesian speed cap.
The two-hand DLS velocity reference remains inside the MPC objective; MPC
continues to produce every command. Both hands latch one joint pose once they
are within 0.5 mm, preventing posture drift during a hold.

The 2026-11-15 captured 86 mm stall was reproduced through ROS topics and
mock ros2_control. With bounded projection and joint-acceleration prediction,
two live runs reached 3 mm / 1 degree in 2.83–2.92 s. Final errors were
0.18–0.21 mm / below 0.02 degree, with over 10 degrees of joint-limit clearance.
The second run's sampled joint-linear command segments stayed within 1.45 mm
of the hand paths and 0.38 mm of mirror symmetry. Callback maxima were below
37 ms at a 50 ms period. These are measured simulation results, not a hard
real-time guarantee or a guarantee for arbitrary goals.

In a visible mock ros2_control run, a 300 mm forward/upward reach settled both
hands within 3 mm for 0.5 s in 4.41 s. The following mirrored 100 mm X-Z square
reached all five targets within 0.25 mm per hand; corner times were 1.11–2.29 s.
Using one joint-state sample for both TCPs, peak left/right mirror error was
3.15 mm during the reach and 0.81 mm during the square. Straight-line deviation
from the original 300 mm reach was at most 10.1 mm per hand; reanchored path
segments stayed within 5 mm. Torso pitch ranged from -6.77 to 3.78 degrees,
because its zero-degree preference is soft. The two separately published TCP
topics yielded up to 13.8 mm when compared without timestamp alignment;
use the joint-state diagnostics for simultaneous comparisons. A 2 s reach while
keeping this level of symmetry and path accuracy remains unverified.

The GUI's **Controller** line shows `joint_position_limit_limited` or
`joint_position_limit_hold` when the current motion is constrained, even while
Run MPC remains checked, and lists the joint names currently at or beyond a
position limit. A held position does not mean the requested goal was reached;
if the hold persists, the target may need a different feasible configuration.
Jog buttons
change the goal once per click, so leaving Run MPC checked does not create a
continuous jog. Duplicate F7 stop messages reset the solver only once.

Starting requires fresh feedback for both hands and torso; run readiness
requires fresh left/right hand targets. JSONL includes the left
hand pose/error and preferred torso posture under `eir`, alongside the primary right-hand
diagnostics. This simulation does not implement self-collision avoidance or
EIR hardware control.

Before the latest joint-limit cost and recovery changes, the shared controller
was checked with delayed 18-joint feedback (0/20 ms)
and hold disabled. Both-hand moves still passed with
the solver iteration cap reduced from 5 to 3. In a ros2_control/GUI integration
run, F7 remained held through repeated left-hand jogs, an elbow-limit stop and
a reversed target. The blocked interval held a constant joint command;
reversal reached 0.16/0.025 mm left/right hand error, and releasing F7 stopped
motion. Solver p95 was 13.7 ms and full callback p95 was 15.8 ms (maximum
20.25 ms); these measurements do not establish a hard real-time deadline.

Those measurements describe the previous limit-hold behavior. The current
regression replays the same constrained move and reversal while checking hard
joint bounds and hand-path limits; it no longer requires a constant command at
the limit. This local response is not a global reachable-workspace or
self-collision planner, and needs a fresh runtime run after the changes above.

With the default expanded elbows, the same inward 13 cm/upward 9 cm goal
was reached in a ros2_control/GUI run at approximately -94 degrees left elbow,
with 0.011 mm final left-hand error and a stationary held pose. Releasing F7
and reversing the target worked. The sampled 0/40 ms replay trajectories had
no nonadjacent CAD collisions, but the expanded interval as a whole is not
collision-free: a fixed-ready-pose elbow sweep intersects the left wrist and
torso near -123 degrees. No self-collision avoidance is added by widening limits.

## Isaac Sim visualization

Isaac Sim can display the EIR articulation by following the mock controller's
ROS 2 `/joint_states` topic. Start the existing EIR simulation, then launch
Isaac Sim from another terminal:

```bash
make docker-up-robot-sim
FORCE_COLCON_BUILD=1 make docker-launch-eir-sim
# In a second terminal; ISAAC_SIM_ROOT defaults to ~/isaacsim-6.0.1
make isaac-eir-viewer
```

The EIR launch also opens the live MPC monitor. It plots TCP error, cost terms,
solver and callback time, solver iterations/success, the MPC model `dt`, and the
measured joint-command publication period/rate. Pass `plot:=false` to
`ros2 launch robot_mpc eir_sim.launch.py` to disable it.

The viewer opens `isaac_sim/eir.usd` and subscribes on ROS domain 78. Keep the
EIR launch running, then check **Run MPC** in its GUI to move the Isaac model.
To use another Isaac Sim install, set
`ISAAC_SIM_ROOT=/path/to/isaac-sim` on the viewer command.

The official cart and wheels are already in `meshes/eir/visual/base_link.glb`.
The Isaac viewer opens the saved project stage at `isaac_sim/eir.usd` directly.
That saved stage omitted the GLB cart geometry, so the viewer adds the converted
official mesh from `isaac_sim/eir_base_link.usda`, then adds a ground surface,
fill/key lights and closer camera framing in memory. It removes the temporary
cart approximation and its invalid fixed joint from the loaded stage, without
overwriting `eir.usd`. The saved ROS joint-state graph is reused and retargeted
to the articulation.

This is a one-way visualization: the existing mock ros2_control publishes the
joint states that drive Isaac Sim. Isaac physics state is not fed back into the
MPC, so this checks the MPC motion and model rendering, not coupled simulation
or dynamic agreement between the two simulators.

The URDF defines `hand_tcp` as a child of `tcp`. Its local translation and RPY
are read from the `hand_tcp_fixed` joint in
[`indy7.urdf`](../robot_description/urdf/indy7.urdf); changes to that mount offset
are included in the model on restart. Translation is expressed in the original
TCP axes. Manual MPC uses `tcp` by default. Enabling Vive
simulation following selects `hand_tcp` as the MPC task frame.

`robot_vive_mapper` synchronizes the current Vive hand pose with the current
robot `hand_tcp` pose, then expresses subsequent Vive poses in robot `world`.
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

The selected goal is published on `/robot/hand_target_pose`, and TF publishes
`world -> hand_target`. The default RViz template includes **Selected Hand
Target**. An existing saved RViz layout can show `hand_target` through TF, or add
a Pose display for that topic without replacing the saved layout. Keyboard input
uses `/robot/keyboard/hand_target_pose`; mapped topic input retains
`/robot/vive/hand_target_pose` and the `vive_hand_target` preview frame. Only the
selected input can reach MPC. Robot joint mapping and DCP3 controls remain in
their existing GUI sections.

## Run in Docker

Build the updated description and new mapper entry point in the existing MPC
container (from the framework root):

```bash
FORCE_COLCON_BUILD=1 make docker-shell-robot-sim
```

## World Z floor

The experiment YAML keeps the controlled `task_frame` at **world Z >= 0.15 m**.
In the minimal GUI this frame is `hand_tcp`, for both Keyboard and Topic Pose.
The Desired TCP and shared `hand_target` remain the exact requested poses, even
below the floor. Only the private MPC tracking/path reference projects a lower
Z goal up to 0.151 m; its X/Y and orientation still follow the requested pose.
Each running and terminal MPC stage also has a height penalty with 1 mm
clearance to guide feasible motion. The final joint-position filter certifies
the full joint-linear command step, including poses between its endpoints, and
shortens or holds a command that would cross the hard floor. Existing joint
position, rate, increment, and Cartesian path limits remain active.

Configure these startup parameters in `config/indy7_mpc.yaml` and restart the
launch after editing them:

```yaml
workspace_floor_enabled: true
workspace_min_z: 0.15             # metres in URDF world
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

## MPC tracking and timing

The actuator interface accepts absolute joint angles. The full-dynamics MPC
generates a position trajectory; its optimized torques are internal predictions,
not measured or commanded motor torques. A physical torque bound cannot be
inferred from the internal box constraints without identifying the position
servo and its tracking error.

The default MPC runs at **100 Hz** (`dt: 0.01`, `horizon: 10`), preserving the
100 ms preview while halving the shooting stages. ros2_control remains at
200 Hz and DCP3 retains its own communication rate. Changing the MPC timer
does not change those rates. Previous state/control plans are shifted one stage
before each solve. A 10 ms timer is a requested period, not a hard execution
deadline; check callback time on the deployment machine.

The position-command boundary relaxes successive plans with
`command_smoothing_factor: 0.75`: 75% of the new position plan plus 25% of the
previous emitted command. During normal tracking, the generated path's joint
velocity compensates the smoothing lag by adding `0.25 * dt * v_reference`.
Hold and fallback commands omit this motion feedforward. All position, rate,
increment, path and floor filters run after this step. F6 synchronization and
F7 pause/resume re-anchor the previous command to the aligned/measured joints.
Changing the smoothing factor requires a restart; `1.0` disables relaxation.

By default, each stage tracks the joint velocity obtained from the generated
Cartesian path's translational and angular increments through a damped
Jacobian inverse. It becomes zero when the path ends and is bounded by the
joint velocity and command-increment limits. The optional
`pose_velocity_feedforward_enabled` experiment instead uses filtered raw input
motion; leave it false for generated-path velocity tracking.

The path uses cubic speed ramps with a constant-speed middle section, giving
zero speed and acceleration at the endpoints of a fixed-target segment.
Each ramp lasts half `cartesian_min_duration` (50 ms in the experiment YAML).
The planned duration includes the ramps so the original linear/angular speed
caps are respected. This adds braking time instead of reducing cruise speed;
changing a streamed goal can still change the current path.

Torque regularization is now `0.5 * w * ||tau - g(q)||^2`. Gravity compensation
has zero torque penalty at every predicted configuration. Dynamics and box
limits still use total torque. The cost derivatives include the gravity
Jacobian and state/control cross term.

The simulation hold requires the measured pose to remain within 2 mm and
0.01 rad of the projected goal for 100 ms. Over that window, each measured
joint position and each joint command must span at most
`stationary_hold_velocity_tolerance * stationary_hold_duration` (0.003 rad).
This rejects real joint/command motion without waiting indefinitely for the
mock feedback velocity filter to decay. Stable simulation windows also prevent
that velocity estimate alone from restarting the solver. Pose drift beyond
twice the entry tolerances or a changed target resumes optimization. Identical
repeated targets do not restart settling. Hardware mode additionally retains
the instantaneous 0.03 rad/s speed condition and stationary-solver check.
These are empirical switching thresholds, not terminal stability conditions.

The 5 mm path tube and floor command filters remain enforced. For example, a
forward command that also moves 6 mm sideways is corrected or shortened before
publication. If measured feedback is already 6 mm outside the tube, the
controller holds instead of inventing an uncertified return path. That is a
different failure from ordinary along-path tracking lag, which does not count
as lateral deviation. Look for `cartesian_path_limited`, `cartesian_path_hold`,
or `cartesian_path_violation` when diagnosing slow motion.

Terminal weights cannot be compared between papers without matching the cost
units, discretization, preview and low-level plant. In
[Nubert et al., RA-L 2020](https://arxiv.org/html/1912.10360), the terminal cost
is an LQR infinite-horizon value, combined with a terminal set and robust
constraint tightening. The manipulator paper
[Incremental MPC Exploiting Time-Delay Estimation, TCST 2022](https://doi.org/10.1109/TCST.2022.3142629)
instead develops tracking MPC without terminal ingredients, under its own
model and stability assumptions. Our scalar terminal multiplier is empirical
and does not reproduce either guarantee. Before the landing and position-command
updates, sweeping 10, 1 and 0.1 in the local
tracking comparison produced moving-target RMS errors of 2.364, 2.366 and
2.370 mm respectively, so **10 is retained** rather than changing it solely
because of its ratio to the integrated running cost.

Local before/after checks used the repository URDF, ideal joint-position
tracking, a fixed simulation step, and two seconds per scenario; no ROS or
hardware delay was simulated. The moving goal was updated at 50 Hz along a
0.5 Hz curve with 30 mm X and 20 mm Y amplitudes. Solver timing excludes
reference generation, diagnostics, command filtering and publishing. Hold
switching was disabled in this comparison to exercise the optimizer throughout.

| Metric | Previous 200 Hz / 20 stages | Updated 100 Hz / 10 stages |
| --- | ---: | ---: |
| Moving-goal RMS position error | 2.706 mm | 2.567 mm |
| Moving-goal solver median | 5.03 ms | 2.89 ms |
| Moving-goal solver 95th percentile | 5.58 ms | 2.93 ms |
| 50 mm X move, first error below 2 mm | 0.11 s | 0.14 s |
| Startup goal, first pose within 2 mm / 0.01 rad | 0.525 s | 0.560 s |

These results support lower computation latency and improved moving-target
tracking, not a claim that every point-to-point motion is faster. Physical
servo lag and the full callback execution time still need measurement. Rebuild
the package/container and restart the launch to apply the Python, timer and
hold changes; cost-YAML hot reload alone is insufficient.

The arrival update also has regression checks with residual estimated speeds
that alternate above the hold threshold, bounded measured/command motion, and
separate cases that must refuse hold while measured joints or commands move.
A zero-delay mock-feedback run settled with both versions and did not reproduce
the RViz problem. Adding a single 10 ms feedback delay reproduced sustained
oscillation with smoothing disabled, even with stationary hold disabled.
The regression in `test_position_servo_closed_loop.py` runs the real callback,
position plant, timestamped feedback and velocity filter over four seconds.
It checks fixed goals with 0/10/20 ms feedback delay, the recorded initial
posture, moving targets and rotation, with hold disabled throughout.

An independent headless ros2_control/GenericSystem run used a delayed
JointState relay in localhost-only ROS domain 98. With the same captured
starting joints and fixed goal, and hold disabled for the entire test, the
last two seconds of the 18-second 10 ms-delay runs gave:

| Metric | Smoothing disabled | Smoothing 0.75 |
| --- | ---: | ---: |
| Largest per-joint position span | 16.79 degrees | < 0.001 degree |
| Maximum TCP position error | 28.835 mm | 0.006 mm |

The 20 ms-delay run also converged without hold. These are ideal position-servo
tests, not physical-robot guarantees. The output relaxation addresses the
observed delayed-feedback instability; it does not identify the hardware servo
or make internal torque bounds equivalent to real motor torque bounds.

## F6 robot connection (DCP3 TELE_OP)

Start the hardware adapter alongside the simulation workflow. Replace the example
IP below with the actual controller address. Stop any previous simulation/driver
launch before restarting updated nodes; the Vive publisher can stay running.

```bash
make docker-up-robot ROBOT_IP=192.168.0.7
```

Terminal 1 starts DCP3 feedback in the disarmed state:

```bash
FORCE_COLCON_BUILD=1 make docker-launch-robot-dcp3 ROBOT_IP=192.168.0.7
```

Terminal 2 starts the simulation and GUI:

```bash
FORCE_COLCON_BUILD=1 make docker-launch-robot-sim-minimal
```

Select Keyboard or configure/map Topic Pose with F5. Then press **F6 Connect
Robot**. F6 pauses MPC, copies measured robot joints through the saved joint
mapping into the simulation, confirms the resulting joint alignment, and
re-anchors the selected hand target. Topic Pose is mapped again after alignment;
Keyboard is reset to the newly measured hand pose. The GUI then supplies a
measured-position hold and a live Deadman, calls `/robot_dcp3/arm`, and waits for
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

If robot simulation is already running, stop and restart its launch after the
build so `robot_state_publisher` loads the URDF with `hand_tcp`. Keep the Vive
publisher running in its own container. Both containers use `ROS_DOMAIN_ID=77`.

In a sourced MPC shell, run:

```bash
ros2 launch robot_mpc robot_vive_mapping.launch.py gui:=true
```

The default input is `/tracker/right/pose`, matching the current Vive example YAML.
Override it if your tracker uses a different namespace:

```bash
ros2 launch robot_mpc robot_vive_mapping.launch.py input_pose_topic:=/vive/tracker/pose
```

In another MPC shell, confirm that the robot hand frame exists, hold the robot
and Vive hand pose still, and synchronize:

```bash
ros2 run tf2_ros tf2_echo world hand_tcp
# Ctrl+C after checking the frame, then:
ros2 service call /robot/vive_mapping/sync std_srvs/srv/Trigger '{}'
ros2 topic echo /robot/vive/hand_target_pose --once
```

`success: true` means the reference was captured. At that instant the mapped
pose equals the current robot hand TCP. Subsequent motion is mapped 1:1.
Synchronize again after moving/recalibrating the Lighthouse setup or restarting
the mapper. The alignment is held in memory for this run.

| Output | Meaning |
| --- | --- |
| `/robot/hand_tcp/current_pose` | Current robot hand TCP from TF, in `world` |
| `/robot/vive/hand_target_pose` | Mapped Vive hand target, in `world` |
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
make docker-up-robot-sim
FORCE_COLCON_BUILD=1 make docker-launch-robot-sim-minimal
```

This starts the simulator, minimal GUI, and mapper together. Keep the Vive ROS
publisher running in its separate container. Do not also start a second mapper
with `robot_vive_mapping.launch.py`. Hold the robot and Tracker poses still,
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
release and press again. `/robot/sim/vive_mpc_hold` carries a `std_msgs/String`
JSON message with `held` and a per-press `token`. A new held token starts
following, further held messages renew it, and a matching release pauses.
The status echoes the token so a delayed response from an older press cannot
cancel a new press. The run service also requires a fresh hold for a true request;
false requests always pause.

For an existing simulation without a mapper, open the standalone mapping GUI
from a sourced MPC shell instead:

```bash
ros2 launch robot_mpc robot_vive_mapping.launch.py gui:=true
```

If a current-version mapper is already running, open only its GUI client:

```bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ros2 run robot_mpc robot_vive_mapping_gui
```

Both launch files run the mapper with UDPv4 to avoid cross-container shared
memory issues. For a different Vive topic in the combined launch, use
`vive_mapping:=true vive_input_pose_topic:=/your/tracker/pose` with
`robot_mpc.launch.py`. The standalone mapper launch uses `input_pose_topic`.

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

The simulation runtime publishes JSON at `/robot/sim/vive_mpc_status`. Its
`SetBool` service `/robot/sim/vive_mpc_run` accepts `true` to start and `false` to
pause. These endpoints are created only in task-space simulation with
`enable_vive_follow` enabled. The GUI shows readiness and stop reasons from
this runtime rather than depending on DCP3 feedback.

The mapper publishes versioned JSON status at 5 Hz on
`/robot/vive_mapping/status` (`std_msgs/msg/String`), including input/TF readiness,
calibration state, a mapper session ID, and a successful-sync sequence number.
An input revision changes whenever the selected topic changes, allowing GUI
clients to discard previews from the previous source.
This lets the same panel be embedded in other GUIs without owning robot output.

## Persistent RViz layout

Docker development shells set `ROBOT_RVIZ_CONFIG` to a file in the writable
development directory. The launch creates it from the packaged default only
when it does not exist. The initial window is 1000 x 720 at (40, 40); it does
not restore the old oversized dock state. Existing saved layouts are never
overwritten by launch, image rebuilds, or colcon builds.

| Location | RViz configuration |
| --- | --- |
| Host, relative to the framework root | `.docker-data/robot-mpc/rviz/indy7.rviz` |
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
ros2 launch robot_mpc robot_mpc.launch.py \
  rviz_config:=/var/lib/robot-teleoperation-dev/rviz/experiment.rviz
```

A missing selected file is seeded once. **Save Config As** can also create a
layout in this directory, but the next launch must select that filename with
`rviz_config`. Directory mounting is intentional so Qt can save by creating
and renaming a temporary file.

## Transform convention

Let `W` be robot world, `V` the Lighthouse world, and `H` the hand frame. On sync:

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
