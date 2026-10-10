from glob import glob
from pathlib import Path

from setuptools import setup


package_name = "indy7_mpc"
mesh_root = Path("meshes/indy7")

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}", ["THIRD_PARTY_NOTICE.md"]),
        (f"share/{package_name}/urdf", glob("urdf/*")),
        (f"share/{package_name}/launch", glob("launch/*")),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/meshes/indy7/visual", glob(str(mesh_root / "visual/*"))),
        (f"share/{package_name}/meshes/indy7/collision", glob(str(mesh_root / "collision/*"))),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "crocoddyl_mpc_node = indy7_mpc.crocoddyl_mpc_node:main",
            "keyboard_teleop = indy7_mpc.keyboard_teleop:main",
            "mpc_plotter = indy7_mpc.mpc_plotter:main",
            "plot_mpc_log = indy7_mpc.plot_mpc_log:main",
        ],
    },
)
