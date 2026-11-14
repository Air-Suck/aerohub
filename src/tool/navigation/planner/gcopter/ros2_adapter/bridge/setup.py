from setuptools import setup
setup(name='uav_gcopter_adapter', version='0.1.0', packages=['uav_gcopter_adapter'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/uav_gcopter_adapter']),
                  ('share/uav_gcopter_adapter', ['package.xml'])],
      entry_points={'console_scripts': ['gcopter_adapter = uav_gcopter_adapter.node:main']})
