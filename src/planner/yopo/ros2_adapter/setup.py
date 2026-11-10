from setuptools import setup, find_packages
setup(name='uav_yopo_planner', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/uav_yopo_planner']),
                  ('share/uav_yopo_planner', ['package.xml', 'LICENSE'])],
      install_requires=['setuptools'], maintainer='Aerohub', maintainer_email='dev@example.com',
      description='YOPO reference-mode single vehicle planner', license='MIT',
      entry_points={'console_scripts': ['yopo_planner = uav_yopo_planner.node:main']})
