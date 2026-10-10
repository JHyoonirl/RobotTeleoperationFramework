from glob import glob
from pathlib import Path

from setuptools import setup


package_name = "indy7_description"
mesh_root = Path("meshes/indy7")

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "THIRD_PARTY_NOTICE.md"]),
        (f"share/{package_name}/urdf", glob("urdf/*.urdf")),
        (f"share/{package_name}/meshes/indy7/visual", glob(str(mesh_root / "visual/*"))),
        (f"share/{package_name}/meshes/indy7/collision", glob(str(mesh_root / "collision/*"))),
    ],
    zip_safe=True,
)
