from glob import glob
from setuptools import setup


package_name = "indy7_mpc"

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
            "crocoddyl_mpc_node = indy7_mpc.crocoddyl_mpc_node:main",
            "keyboard_teleop = indy7_mpc.keyboard_teleop:main",
            "indy7_vive_mapper = indy7_mpc.vive_pose_mapper:main",
            "indy7_vive_mapping_gui = indy7_mpc.vive_mapping_gui:main",
            "indy7_teleop_gui = indy7_mpc.teleop_gui:main",
            "indy7_teleop_minimal_gui = indy7_mpc.teleop_minimal_gui:main",
            "mpc_plotter = indy7_mpc.mpc_plotter:main",
            "plot_mpc_log = indy7_mpc.plot_mpc_log:main",
        ],
    },
)
