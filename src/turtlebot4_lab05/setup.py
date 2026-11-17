from glob import glob
from setuptools import setup

package_name = 'turtlebot4_lab05'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/'+package_name]),
                  ('share/'+package_name, ['package.xml', 'README.md', 'VALIDATION.md', 'VIDEO_DEMO.md', 'requirements-yolo.txt']),
                  ('share/'+package_name+'/launch', glob('launch/*.launch.py')),
                  ('share/'+package_name+'/config', glob('config/*')),
                  ('share/'+package_name+'/maps', glob('maps/*')),
                  ('share/'+package_name+'/validation', glob('validation/*')),
                  ('share/'+package_name+'/scripts', glob('scripts/*.py')+glob('scripts/*.bash'))],
      install_requires=['setuptools', 'PyYAML'], zip_safe=True,
      maintainer='TurtleBot lab contributors', maintainer_email='maintainer@example.com',
      description='Sequential Nav2 navigation and RGB YOLO for TurtleBot 4 Humble',
      license='Apache-2.0',
      entry_points={'console_scripts': [
          'sequential_navigation = turtlebot4_lab05.sequential_navigation:main',
          'yolo_node = turtlebot4_lab05.yolo_node:main',
          'capture_waypoints = turtlebot4_lab05.capture_waypoints:main',
          'initialize_pose = turtlebot4_lab05.initialize_pose:main',
          'spawn_demo_objects = turtlebot4_lab05.spawn_objects:main']})
