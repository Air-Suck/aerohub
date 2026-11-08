from setuptools import setup,find_packages
setup(name='uav_super_adapter',version='0.1.0',packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',['resource/uav_super_adapter']),
                  ('share/uav_super_adapter',['package.xml'])],
      install_requires=['setuptools'],license='Apache-2.0',
      entry_points={'console_scripts':['super_timed_path_adapter = uav_super_adapter.node:main']})
