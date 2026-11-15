# Robot Description

This package stores robot URDFs and visual/collision meshes. It currently
contains the Neuromeka Indy7 and EIR models and stays independent from MPC and driver
packages so each runtime can use the selected model.

Add each robot under `urdf/<model>.urdf` and `meshes/<model>/visual` and
`meshes/<model>/collision`; the package setup installs each model directory.

The assets are selected from the official `neuromeka-robotics/indy-ros2`
repository. See `THIRD_PARTY_NOTICE.md` for source revision and license.

The Indy7 `hand_tcp` frame is fixed to the existing `tcp` frame with translation
`[-0.1, 0, 0.08]` metres expressed in the original TCP axes and roll/pitch/yaw
`[0, -180, 0]` degrees. Thus `T_world_hand = T_world_tcp * T_tcp_hand`.
The frame adds no geometry, mass, or joint degrees of freedom.

EIR has 18 revolute joints: four body/head joints and seven per arm. The
ungripped hand endpoint frames are `left_hand_tcp` and `right_hand_tcp`.
Each is fixed to its corresponding `*_arm_link_7` with translation
`[0, 0, 0.097]` metres and roll/pitch/yaw `[0, -1.57075, 0]` radians,
matching the official EIR `eir_to_*_gripper` attachment transform. The
97 mm offset places the origin at the CAD terminal mounting flange center:
both link-7 visual meshes have their terminal planar face at local
`z = 97 mm` (mesh coordinates are scaled by 0.001 in the URDF).
TCP +X points outward along wrist +Z; TCP +Y follows wrist +Y and TCP +Z
points along wrist -X, up to the upstream angle's rounding.
These frames add no geometry, mass or degrees of freedom; no gripper/tool
length or Indy7 hand mount offset is assumed. Its ros2_control block uses
position-only mock hardware.

The official cart lower-body mesh is `meshes/eir/visual/base_link.glb`, attached
to the floor-level `base_link` by the URDF visual origin. It includes the cart
and wheels as one visual mesh; the wheels do not have independent joints or
drive control.

For this simulation experiment, both elbow joints (`Joint_L5_L/R`) use
`[-2.5307, 2.5307]` rad (approximately +/-145 degrees), replacing the upstream
`[-1.0472, 2.5307]` rad (approximately -60 to +145 degrees). The model limits
and ros2_control command-interface bounds agree. This is a user-requested
simulation override, not a verified mechanical range for real EIR hardware.
All other joint bounds retain their upstream values.
The expanded interval permits CAD self-intersections: with other joints fixed
at the ready pose, a sampled left-elbow sweep first intersects the wrist and
torso near -123 degrees. The controller does not avoid these collisions.

EIR starts in a project-specific simulation ready pose, with the torso/head
at zero and both hands forward and below the shoulders. In joint order
`Joint_L2` through `Joint_L8`, the left arm is
`[-30, -60, 0, -45, 0, 0, 0]` degrees and the right arm is
`[30, 60, 0, 45, 0, 0, 0]` degrees. Initial positions are stored in the
URDF's ros2_control position state interfaces; MPC posture references match.
The resulting hand positions are approximately `[0.318, +/-0.3195, 0.876]` m
in world coordinates.

This is not the manufacturer's calibrated Home. The upstream EIR
[SRDF](https://github.com/neuromeka-robotics/indy-ros2/blob/ca0353f077620c877c1edff2e26c8cd976773f27/indy_moveit/srdf/indy_macro.srdf.xacro#L14-L43)
provides only `LeftZero`/`RightZero`; all-zero arms form a lateral T-pose.
The chosen ready pose stays within the published joint/effort limits, has
full-rank hand Jacobians, and passed a check of 181 nonadjacent collision-mesh
pairs (minimum separation 11.66 mm). This static check does not add
self-collision avoidance during motion.
