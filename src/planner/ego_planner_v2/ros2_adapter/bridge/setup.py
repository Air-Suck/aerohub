from setuptools import setup
from glob import glob
setup(name='uav_ego_planner_adapter', version='0.1.0',
      packages=['uav_ego_planner_adapter'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/uav_ego_planner_adapter']),
                  ('share/uav_ego_planner_adapter', ['package.xml']),
                  ('share/uav_ego_planner_adapter/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], license='GPL-3.0',
      entry_points={'console_scripts': ['ego_as2_adapter = uav_ego_planner_adapter.node:main']})
