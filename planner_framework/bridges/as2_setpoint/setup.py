from setuptools import find_packages, setup


package_name = "uav_as2_setpoint_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML>=6"],
    zip_safe=True,
    maintainer="noah",
    maintainer_email="noah@example.com",
    description="Translate PX4-local NED fleet setpoints to Aerostack2 references.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "as2_setpoint_adapter_node = uav_as2_setpoint_adapter.node:main",
            "super_as2_adapter_node = uav_as2_setpoint_adapter.super_node:main",
            "super_as2_mission = uav_as2_setpoint_adapter.super_mission:main",
        ],
    },
)
