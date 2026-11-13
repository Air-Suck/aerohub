from setuptools import find_packages, setup

package_name = "uav_dpv_slam_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="AeroHub maintainers",
    maintainer_email="maintainers@example.com",
    description="ROS 2 Humble online adapter for DPV-SLAM.",
    license="BSD-3-Clause",
    entry_points={"console_scripts": ["dpv_slam_node = uav_dpv_slam_ros2.node:main"]},
)
