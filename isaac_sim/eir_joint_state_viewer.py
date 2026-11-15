#!/usr/bin/env python3
"""Load the saved EIR USD and visualize ROS 2 joint_states in Isaac Sim."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
USD_PATH = REPO_ROOT / "isaac_sim/eir.usd"
BASE_LINK_USD = REPO_ROOT / "isaac_sim/eir_base_link.usda"
VISUALIZATION_DRIVE_STIFFNESS = 100.0
VISUALIZATION_DRIVE_DAMPING = 10.0


def start_ros_debug_monitor(ros_domain_id):
    """Print throttled /joint_states diagnostics from Isaac Sim's Python console."""
    import os
    import threading
    import time

    os.environ["ROS_DOMAIN_ID"] = str(ros_domain_id)
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import JointState

        context = Context()
        context.init(args=None, domain_id=ros_domain_id)
        node = rclpy.create_node(
            "eir_isaac_joint_state_debug", context=context
        )
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        state = {"count": 0, "last_message": None, "last_receive": None}

        def on_joint_state(message):
            state["count"] += 1
            state["last_message"] = message
            state["last_receive"] = time.monotonic()

        node.create_subscription(JointState, "/joint_states", on_joint_state, qos)
        previous_count = 0
        empty_reports = 0

        def report():
            nonlocal previous_count, empty_reports
            message = state["last_message"]
            if message is None:
                empty_reports += 1
                if empty_reports == 1 or empty_reports % 5 == 0:
                    print(
                        "[EIR ROS DEBUG] Waiting for /joint_states "
                        f"(ROS_DOMAIN_ID={ros_domain_id}, QoS=BEST_EFFORT/VOLATILE)"
                    )
                return

            empty_reports = 0
            received = state["count"]
            rate = received - previous_count
            previous_count = received
            age = time.monotonic() - state["last_receive"]
            names = list(message.name)
            positions = list(message.position)
            pairs = ", ".join(
                f"{name}={position:.3f}"
                for name, position in zip(names, positions)
            )
            print(
                "[EIR ROS DEBUG] /joint_states "
                f"rx={rate}/s age={age:.2f}s frame={message.header.frame_id!r} "
                f"names={len(names)} positions={len(positions)} "
                f"name_position_match={len(names) == len(positions)} [{pairs}]"
            )

        node.create_timer(1.0, report)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        print(
            "[EIR ROS DEBUG] Monitor started: "
            f"/joint_states, ROS_DOMAIN_ID={ros_domain_id}, "
            f"RMW_IMPLEMENTATION={os.environ.get('RMW_IMPLEMENTATION', '<default>')}"
        )
        return context, executor, node, thread
    except Exception as exc:
        print(f"[EIR ROS DEBUG] Could not start rclpy monitor: {type(exc).__name__}: {exc}")
        return None


def start_graph_debug_monitor(graph_path):
    """Report live ROS2-subscribe outputs and articulation inputs once per second."""
    import time

    import omni.graph.core as og
    import omni.kit.app

    subscribe_node = og.get_node_by_path(f"{graph_path}/SubscribeJointState")
    controller_node = og.get_node_by_path(f"{graph_path}/ArticulationController")
    if subscribe_node is None or controller_node is None:
        print(f"[EIR GRAPH DATA] Could not inspect nodes in {graph_path}")
        return None

    last_report = 0.0

    def sequence(node, port):
        value = node.get_attribute(port).get()
        return [] if value is None else list(value)

    def report(_event):
        nonlocal last_report
        now = time.monotonic()
        if now - last_report < 1.0:
            return
        last_report = now
        try:
            names = sequence(subscribe_node, "outputs:jointNames")
            positions = sequence(subscribe_node, "outputs:positionCommand")
            controller_names = sequence(controller_node, "inputs:jointNames")
            controller_positions = sequence(controller_node, "inputs:positionCommand")
            target = sequence(controller_node, "inputs:targetPrim")
            preview = ", ".join(f"{float(value):.3f}" for value in positions[:3])
            print(
                "[EIR GRAPH DATA] "
                f"subscriber_names={len(names)} subscriber_positions={len(positions)} "
                f"controller_names={len(controller_names)} "
                f"controller_positions={len(controller_positions)} "
                f"targetPrim={target} q0..2=[{preview}]"
            )
        except Exception as exc:
            print(f"[EIR GRAPH DATA] Probe error: {type(exc).__name__}: {exc}")

    stream = omni.kit.app.get_app().get_update_event_stream()
    subscription = stream.create_subscription_to_pop(report)
    globals()["_eir_graph_debug_subscription"] = subscription
    return subscription


def main():
    import os
    import numpy as np
    import omni.graph.core as og
    import omni.timeline
    import omni.usd
    import usdrt.Sdf
    from isaacsim.core.experimental.utils import app as app_utils
    from isaacsim.core.rendering_manager import ViewportManager
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

    app_utils.enable_extension("isaacsim.ros2.bridge")
    ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "78"))
    print(
        "EIR Isaac viewer ROS settings: "
        f"ROS_DOMAIN_ID={ros_domain_id}, "
        f"ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY', '<unset>')}, "
        f"RMW_IMPLEMENTATION={os.environ.get('RMW_IMPLEMENTATION', '<default>')}"
    )
    start_ros_debug_monitor(ros_domain_id)

    if not USD_PATH.is_file():
        raise FileNotFoundError(f"Saved EIR USD was not found: {USD_PATH}")
    if not omni.usd.get_context().open_stage(str(USD_PATH)):
        raise RuntimeError(f"Could not open saved EIR USD: {USD_PATH}")
    stage = omni.usd.get_context().get_stage()

    # This stage was saved before the URDF's temporary cart approximation was
    # removed. Replace it in memory with the original base_link.glb geometry,
    # including the fixed joint that used to attach that temporary geometry.
    temporary_cart = stage.GetPrimAtPath("/eir/Geometry/world/base_link/eir_cart_visuals")
    if temporary_cart.IsValid():
        stage.RemovePrim(temporary_cart.GetPath())
    temporary_cart_joint = stage.GetPrimAtPath("/eir/Physics/eir_cart_fixed_joint")
    if temporary_cart_joint.IsValid():
        stage.RemovePrim(temporary_cart_joint.GetPath())

    if not BASE_LINK_USD.is_file():
        raise FileNotFoundError(f"Converted official base_link CAD was not found: {BASE_LINK_USD}")
    base_link_visual_path = "/eir/Geometry/world/base_link/base_link"
    base_link_visual = stage.GetPrimAtPath(base_link_visual_path)
    if not base_link_visual.IsValid():
        raise RuntimeError(f"Saved USD is missing the base_link visual prim: {base_link_visual_path}")
    if not stage.GetPrimAtPath(f"{base_link_visual_path}/PaintedHousing").IsValid():
        base_link_visual.GetReferences().AddReference(str(BASE_LINK_USD.resolve()), Sdf.Path("/EIRBaseLink"))
    if not stage.GetPrimAtPath(f"{base_link_visual_path}/PaintedHousing").IsValid():
        raise RuntimeError(f"Could not attach the converted EIR base CAD from {BASE_LINK_USD}")
    print(f"Attached the official EIR cart CAD from {BASE_LINK_USD}")

    # Keep the user's saved USD file intact; add the viewer environment in memory.
    UsdGeom.Xform.Define(stage, "/ViewerEnvironment")
    UsdGeom.Xform.Define(stage, "/ViewerEnvironment/Lighting")
    floor = UsdGeom.Cube.Define(stage, "/ViewerEnvironment/Ground")
    floor.CreateSizeAttr(1.0)
    floor_xform = UsdGeom.XformCommonAPI(floor.GetPrim())
    floor_xform.SetTranslate(Gf.Vec3d(0.0, 0.0, -0.05))
    floor_xform.SetScale(Gf.Vec3f(8.0, 8.0, 0.1))
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.48, 0.50, 0.54)])
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

    floor_material = UsdShade.Material.Define(stage, "/ViewerEnvironment/Materials/Floor")
    floor_shader = UsdShade.Shader.Define(stage, "/ViewerEnvironment/Materials/Floor/PreviewSurface")
    floor_shader.CreateIdAttr("UsdPreviewSurface")
    floor_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.48, 0.50, 0.54))
    floor_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    floor_shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    floor_shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    floor_material.CreateSurfaceOutput().ConnectToSource(floor_shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(floor_material)

    dome = UsdLux.DomeLight.Define(stage, "/ViewerEnvironment/Lighting/Fill")
    dome.CreateIntensityAttr(800.0)
    dome.CreateColorAttr(Gf.Vec3f(0.82, 0.88, 1.0))

    key = UsdLux.DistantLight.Define(stage, "/ViewerEnvironment/Lighting/Key")
    key.CreateIntensityAttr(2500.0)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.94, 0.84))
    UsdGeom.XformCommonAPI(key.GetPrim()).SetRotate(Gf.Vec3f(-45.0, 0.0, 30.0))
    print(f"Loaded saved EIR USD with viewer lighting: {USD_PATH}")

    articulation_prims = [
        prim for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    ignored_joint_roots = [prim for prim in articulation_prims if prim.IsA(UsdPhysics.Joint)]
    roots = [prim for prim in articulation_prims if not prim.IsA(UsdPhysics.Joint)]
    if ignored_joint_roots:
        print(
            "[EIR USD DEBUG] Ignoring ArticulationRootAPI on joint prim(s): "
            + ", ".join(str(prim.GetPath()) for prim in ignored_joint_roots)
        )
    if len(roots) != 1:
        raise RuntimeError(f"Expected one EIR articulation root, found {len(roots)}")
    articulation_root = usdrt.Sdf.Path(str(roots[0].GetPath()))
    print(f"Using EIR articulation root for Isaac joint control: {articulation_root}")

    required_joints = {
        "Joint_L0", "Joint_L1", "Joint_L2_U", "Joint_L3_U",
        *(f"Joint_L{i}_{side}" for side in ("L", "R") for i in range(2, 9)),
    }
    imported_joints = {
        prim.GetName() for prim in stage.Traverse()
        if prim.IsA(UsdPhysics.RevoluteJoint)
    }
    missing = sorted(required_joints - imported_joints)
    if missing:
        raise RuntimeError(f"Imported USD is missing EIR joints: {', '.join(missing)}")
    print(
        "[EIR USD DEBUG] Revolute joints "
        f"({len(imported_joints)}): {', '.join(sorted(imported_joints))}"
    )
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.GetStiffnessAttr().Set(VISUALIZATION_DRIVE_STIFFNESS)
        drive.GetDampingAttr().Set(VISUALIZATION_DRIVE_DAMPING)
    print(
        "[EIR PHYSICS DEBUG] Set angular position drives in memory: "
        f"stiffness={VISUALIZATION_DRIVE_STIFFNESS}, "
        f"damping={VISUALIZATION_DRIVE_DAMPING}; existing maxForce values are preserved."
    )

    action_graph_path = "/ActionGraph"
    action_graph_prim = stage.GetPrimAtPath(action_graph_path)
    expected_nodes = {
        "OnPlaybackTick", "Context", "SubscribeJointState", "ArticulationController",
    }
    saved_nodes = {prim.GetName() for prim in action_graph_prim.GetChildren()} if action_graph_prim.IsValid() else set()
    qos_profile = (
        '{"history":"keepLast","depth":10,"reliability":"bestEffort",'
        '"durability":"volatile","deadline":0.0,"lifespan":0.0,'
        '"liveliness":"systemDefault","leaseDuration":0.0}'
    )
    graph_values = [
        ("Context.inputs:domain_id", ros_domain_id),
        ("Context.inputs:useDomainIDEnvVar", False),
        ("SubscribeJointState.inputs:topicName", "/joint_states"),
        ("SubscribeJointState.inputs:qosProfile", qos_profile),
        # Use the target picker directly so a stale robotPath cannot override it.
        ("ArticulationController.inputs:robotPath", ""),
        ("ArticulationController.inputs:targetPrim", [articulation_root]),
    ]
    if expected_nodes <= saved_nodes:
        existing_graph = og.get_graph_by_path(action_graph_path)
        if existing_graph is None:
            raise RuntimeError(f"Saved OmniGraph prim has no live graph: {action_graph_path}")

        # For a graph loaded from USD, SET_VALUES needs each saved Node object
        # paired with its input name; relative "Node.inputs:attr" strings are
        # not resolved against the Graph object.
        context_node = og.get_node_by_path(f"{action_graph_path}/Context")
        subscribe_node = og.get_node_by_path(f"{action_graph_path}/SubscribeJointState")
        controller_node = og.get_node_by_path(f"{action_graph_path}/ArticulationController")
        tick_node = og.get_node_by_path(f"{action_graph_path}/OnPlaybackTick")
        if context_node is None or not context_node.is_valid():
            raise RuntimeError(f"Saved ROS context node is unavailable in {action_graph_path}")
        if subscribe_node is None or not subscribe_node.is_valid():
            raise RuntimeError(f"Saved ROS joint-state node is unavailable in {action_graph_path}")
        if controller_node is None or not controller_node.is_valid():
            raise RuntimeError(f"Saved articulation controller node is unavailable in {action_graph_path}")
        if tick_node is None or not tick_node.is_valid():
            raise RuntimeError(f"Saved playback tick node is unavailable in {action_graph_path}")

        og.Controller.edit(
            existing_graph,
            {
                og.Controller.Keys.SET_VALUES: [
                    (("inputs:domain_id", context_node), ros_domain_id),
                    (("inputs:useDomainIDEnvVar", context_node), False),
                    (("inputs:topicName", subscribe_node), "/joint_states"),
                    (("inputs:qosProfile", subscribe_node), qos_profile),
                    (("inputs:robotPath", controller_node), ""),
                    (("inputs:targetPrim", controller_node), [articulation_root]),
                ]
            },
        )
        controller_exec_in = controller_node.get_attribute("inputs:execIn")
        subscribe_exec_out = subscribe_node.get_attribute("outputs:execOut")
        current_exec_sources = controller_exec_in.get_upstream_connections()
        if (
            len(current_exec_sources) != 1
            or current_exec_sources[0].get_path() != subscribe_exec_out.get_path()
        ):
            for source in current_exec_sources:
                og.Controller.disconnect(source, controller_exec_in)
            og.Controller.connect(subscribe_exec_out, controller_exec_in)
        for node_name, node, port in (
            ("SubscribeJointState", subscribe_node, "inputs:execIn"),
            ("SubscribeJointState", subscribe_node, "inputs:context"),
            ("ArticulationController", controller_node, "inputs:execIn"),
            ("ArticulationController", controller_node, "inputs:jointNames"),
            ("ArticulationController", controller_node, "inputs:positionCommand"),
            ("ArticulationController", controller_node, "inputs:targetPrim"),
            ("ArticulationController", controller_node, "inputs:robotPath"),
        ):
            attribute = node.get_attribute(port)
            sources = [str(source.get_path()) for source in attribute.get_upstream_connections()]
            try:
                value = attribute.get()
            except Exception:
                value = None
            print(
                f"[EIR GRAPH DEBUG] {action_graph_path}/{node_name}.{port} "
                f"value={value!r} "
                f"<- {', '.join(sources) if sources else '<unconnected input>'}"
            )
        print(
            f"[EIR GRAPH DEBUG] Reused {action_graph_path}; "
            f"topic=/joint_states, domain_id={ros_domain_id}, "
            f"articulation_root={articulation_root}."
        )
    else:
        if action_graph_prim.IsValid():
            action_graph_path = "/EIRJointStateGraph"
            suffix = 1
            while stage.GetPrimAtPath(action_graph_path).IsValid():
                action_graph_path = f"/EIRJointStateGraph_{suffix}"
                suffix += 1
        og.Controller.edit(
            {"graph_path": action_graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                og.Controller.Keys.SET_VALUES: graph_values,
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                    ("SubscribeJointState.outputs:execOut", "ArticulationController.inputs:execIn"),
                    ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                    ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ],
            },
        )

    start_graph_debug_monitor(action_graph_path)
    ViewportManager.set_camera_view(
        "/OmniverseKit_Persp",
        eye=np.array([1.8, -2.5, 1.7]),
        target=np.array([0.0, 0.0, 0.8]),
    )
    print(
        "EIR Isaac viewer configured for /joint_states. "
        "Watch [EIR ROS DEBUG] for received joint values; "
        "if values arrive but the robot does not move, inspect the Action Graph "
        "connections and articulation root."
    )
    omni.timeline.get_timeline_interface().play()


main()
