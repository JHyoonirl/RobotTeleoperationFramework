# Indy7 Description

This package owns the Neuromeka Indy7 URDF and visual/collision meshes. It is
kept independent from MPC and hardware driver packages so both runtime images
can use the same robot model.

The assets are selected from the official `neuromeka-robotics/indy-ros2`
repository. See `THIRD_PARTY_NOTICE.md` for source revision and license.

The `hand_tcp` frame is fixed to the existing `tcp` frame with translation
`[-0.1, 0, 0.06]` metres expressed in the original TCP axes and roll/pitch/yaw
`[0, -180, 0]` degrees. Thus `T_world_hand = T_world_tcp * T_tcp_hand`.
The frame adds no geometry, mass, or joint degrees of freedom.
