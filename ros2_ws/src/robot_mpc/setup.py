from glob import glob
from setuptools import setup


package_name = "robot_mpc"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "crocoddyl_mpc_node = robot_mpc.crocoddyl_mpc_node:main",
            "keyboard_teleop = robot_mpc.keyboard_teleop:main",
            "robot_vive_mapper = robot_mpc.vive_pose_mapper:main",
            "robot_vive_mapping_gui = robot_mpc.vive_mapping_gui:main",
            "robot_teleop_gui = robot_mpc.teleop_gui:main",
            "robot_teleop_minimal_gui = robot_mpc.teleop_minimal_gui:main",
            "mpc_plotter = robot_mpc.mpc_plotter:main",
            "plot_mpc_log = robot_mpc.plot_mpc_log:main",
        ],
    },
)
