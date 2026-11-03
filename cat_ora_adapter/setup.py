from setuptools import find_packages, setup


package_name = "uav_cat_ora_adapter"

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
    description="ROS 2 adapter from CAT-ORA trajectories to the shared planner contract.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cat_ora_adapter_node = uav_cat_ora_adapter.node:main",
        ],
    },
)
