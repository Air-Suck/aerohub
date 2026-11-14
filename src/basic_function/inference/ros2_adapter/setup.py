from setuptools import find_packages, setup

setup(
    name="uav_inference", version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/uav_inference"]),
        ("share/uav_inference", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="noah", maintainer_email="noah@example.com", license="Apache-2.0",
    description="ROS adapter for isolated model workers.",
)
