# Third-party model notice

The `urdf/indy7.urdf` and `meshes/indy7/` files are the Indy7 description
selected from Neuromeka's official ROS2 repository:

`https://github.com/neuromeka-robotics/indy-ros2`

Source branch: `jazzy-indyDCP3`  
Source commit: `c4da2f969791c0bfff4b98f0003c5556663ea29e`

The original repository identifies `indy_description` as BSD-3-Clause. This
project keeps the model assets only; the driver, Gazebo, MoveIt, and interface
packages are intentionally not included.

## EIR

`urdf/eir.urdf` and `meshes/eir/` are selected from the same official repository,
branch `jazzy-indyDCP3`, commit `ca0353f077620c877c1edff2e26c8cd976773f27`.
The upstream `indy_description/package.xml` identifies the package as BSD-3-Clause.
Only meshes referenced by the ungripped EIR URDF are included.

Local adaptations replace machine-specific mesh paths with package URIs,
select `mock_components/GenericSystem` with position-only command interfaces,
use the URDF joint limits at that interface, remove Gazebo plugins, and set a
project-specific simulation ready pose: torso/head zero, left arm
`[-30,-60,0,-45,0,0,0]` degrees and right arm `[30,60,0,45,0,0,0]` degrees
in `Joint_L2` through `Joint_L8` order. This is not an official calibrated Home;
the upstream EIR SRDF only defines zero arm states. Geometry, inertias and joint
axes are retained from the upstream model. At the user's request, the simulation
elbow limits (`Joint_L5_L/R`) are widened from `[-1.0472, 2.5307]` rad to
`[-2.5307, 2.5307]` rad in both the joint and ros2_control declarations. These
experimental limits are not a manufacturer-certified mechanical range; all
other joint bounds retain their upstream values. Massless fixed
`left_hand_tcp` and `right_hand_tcp` frames use the official gripper
attachment transforms (`xyz="0 0 0.097"`, `rpy="0 -1.57075 0"`) from
`indy_description/urdf/config/eir/kinematics.yaml`. They mark the bare
mounting flange without adding gripper geometry.

The EIR cart lower-body visual is included in the upstream `base_link.glb`
mesh and is referenced from the local URDF at `base_link`. It is a single
visual mesh; the wheels are not separately actuated.

`isaac_sim/eir_base_link.usda` is a USD conversion of that same `base_link.glb`
for the Isaac viewer because the saved stage did not retain the GLB visual.
