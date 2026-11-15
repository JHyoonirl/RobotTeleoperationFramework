"""Preserve robot-specific RViz templates and user-saved layouts."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext


def test_rviz_uses_selected_robot_template_and_preserves_saved_layout(tmp_path):
    path = Path(__file__).parents[1] / "launch/robot_mpc.launch.py"
    spec = importlib.util.spec_from_file_location("robot_mpc_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    template, saved = tmp_path / "eir.rviz", tmp_path / "profiles/eir.rviz"
    template.write_text("EIR layout\n")
    context = LaunchContext()
    context.launch_configurations.update(rviz_config=str(saved), rviz_config_file=str(template))
    module._launch_rviz(context)
    assert saved.read_text() == template.read_text()
    saved.write_text("User layout\n")
    module._launch_rviz(context)
    assert saved.read_text() == "User layout\n"


@pytest.mark.parametrize("arm,filename", [
    ("left", "eir_left_mpc.yaml"), ("right", "eir_mpc.yaml"), ("both", "eir_dual_mpc.yaml")])
@pytest.mark.parametrize("custom", [False, True])
def test_eir_selects_matching_arm_config_and_preserves_custom_override(arm, filename, custom, monkeypatch):
    package = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location("eir_sim_launch", package / "launch/eir_sim.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(
        package if name == "robot_mpc" else package.parent / "robot_description"))
    context = LaunchContext()
    context.launch_configurations.update(arm=arm, config_file="/tmp/custom-eir.yaml" if custom else "")
    include = module._launch(context)[0]
    arguments = dict(include.launch_arguments)
    expected = "/tmp/custom-eir.yaml" if custom else str(package / "config" / filename)
    assert arguments["config_file"] == expected
    assert arguments["mpc_executable"] == ("eir_mpc_node" if arm == "both" else "crocoddyl_mpc_node")


def test_eir_rviz_markers_use_jazzy_namespace_property():
    import yaml
    config = Path(__file__).parents[1] / 'config/eir.rviz'
    displays = yaml.safe_load(config.read_text())['Visualization Manager']['Displays']
    marker = next(d for d in displays if d['Class'] == 'rviz_default_plugins/InteractiveMarkers')
    assert marker['Enabled']
    assert marker['Interactive Markers Namespace'] == '/eir/goal_markers'
