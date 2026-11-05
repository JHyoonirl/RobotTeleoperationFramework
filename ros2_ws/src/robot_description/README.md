# Robot Description

This package stores robot URDFs and visual/collision meshes. It currently
contains the Neuromeka Indy7 model and stays independent from MPC and driver
packages so each runtime can use the selected model.

Add each robot under `urdf/<model>.urdf` and `meshes/<model>/visual` and
`meshes/<model>/collision`; the package setup installs each model directory.

The assets are selected from the official `neuromeka-robotics/indy-ros2`
repository. See `THIRD_PARTY_NOTICE.md` for source revision and license.

The Indy7 `hand_tcp` frame is fixed to the existing `tcp` frame with translation
`[-0.1, 0, 0.08]` metres expressed in the original TCP axes and roll/pitch/yaw
`[0, -180, 0]` degrees. Thus `T_world_hand = T_world_tcp * T_tcp_hand`.
The frame adds no geometry, mass, or joint degrees of freedom.
