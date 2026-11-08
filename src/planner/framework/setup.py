from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


package_name = "uav_planner_framework"
share = f"share/{package_name}"
registry_files = [Path(path) for path in glob("registry/**/*.yaml", recursive=True)]
registry_data = {}
for path in registry_files:
    registry_data.setdefault(f"{share}/{path.parent.as_posix()}", []).append(str(path))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (share, ["package.xml"]),
        (f"{share}/examples", glob("examples/*.yaml")),
        (f"{share}/launch", glob("launch/*.launch.py")),
    ] + list(registry_data.items()),
    install_requires=["setuptools", "PyYAML>=6"],
    zip_safe=True,
    maintainer="noah",
    maintainer_email="noah@example.com",
    description="Registry, lifecycle, and output gateway for interchangeable UAV planners.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "planner_framework_node = uav_planner_framework.ros_node:main",
            "gazebo_clock_bridge_node = uav_planner_framework.gazebo_clock:main",
        ],
    },
)
