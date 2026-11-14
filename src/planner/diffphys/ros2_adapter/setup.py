from setuptools import setup,find_packages
setup(name='uav_diffphys_planner',version='0.1.0',packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/uav_diffphys_planner']),('share/uav_diffphys_planner',['package.xml'])],
 install_requires=['setuptools'],entry_points={'console_scripts':['diffphys_planner=uav_diffphys_planner.node:main']})
