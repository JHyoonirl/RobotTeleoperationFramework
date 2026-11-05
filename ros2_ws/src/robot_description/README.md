# Robot Description

This package stores robot URDFs and visual/collision meshes. It currently
contains the Neuromeka Indy7 model and stays independent from MPC and driver
packages so each runtime can use the selected model.

Add each robot under `urdf/<model>.urdf` and `meshes/<model>/visual` and
`meshes/<model>/collision`; the package setup installs each model directory.

The assets are selected from the official `neuromeka-robotics/indy-ros2`
repository. See `THIRD_PARTY_NOTICE.md` for source revision and license.
