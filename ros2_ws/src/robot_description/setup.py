from glob import glob
from pathlib import Path

from setuptools import setup


package_name = "robot_description"
mesh_data = []
for model_dir in Path("meshes").iterdir():
    if not model_dir.is_dir():
        continue
    for mesh_type in ("visual", "collision"):
        files = glob(str(model_dir / mesh_type / "*"))
        if files:
            mesh_data.append(
                (f"share/{package_name}/{model_dir}/{mesh_type}", files)
            )

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "THIRD_PARTY_NOTICE.md"]),
        (f"share/{package_name}/urdf", glob("urdf/*.urdf")),
        *mesh_data,
    ],
    zip_safe=True,
)
