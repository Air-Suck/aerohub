from setuptools import find_packages, setup

setup(name='uav_object_detection', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/uav_object_detection']),
                  ('share/uav_object_detection', ['package.xml']),
                  ('share/uav_object_detection/licenses', ['LICENSES/gemini2-depth-NOTICE.txt',
                                                           'LICENSES/gemini2-depth-APACHE-2.0.txt'])],
      install_requires=['setuptools'], license='GPL-3.0-or-later',
      description='Object Detection Tool using Aerohub Inference.',
      entry_points={'console_scripts': ['object_detection = uav_object_detection.node:main']})
