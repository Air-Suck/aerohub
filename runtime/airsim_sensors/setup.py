from setuptools import setup

name = 'uav_as2_sensor_adapter'
setup(name=name, version='0.1.0', packages=[name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml'])],
      install_requires=['setuptools', 'msgpack'],
      entry_points={'console_scripts': ['airsim_sensor_node = ' + name + '.node:main',
                                       'airsim_clock_node = ' + name + '.clock:main']})
