from glob import glob
from setuptools import setup
name='turtlebot4_gesture_control'
setup(name=name,version='0.1.0',packages=[name],
    data_files=[('share/ament_index/resource_index/packages',['resource/'+name]),
                ('share/'+name,['package.xml','README.md','requirements.txt','LICENSE','NOTICE']),
                ('share/'+name+'/launch',glob('launch/*.launch.py')),
                ('share/'+name+'/config',glob('config/*')),
                ('share/'+name+'/worlds',glob('worlds/*')),
                ('share/'+name+'/scripts',glob('scripts/*.py')+glob('scripts/*.bash'))],
    install_requires=['setuptools'],zip_safe=True,
    maintainer='Lê Anh Tuấn Bằng',maintainer_email='maintainer@example.com',
    description='MediaPipe webcam gestures and bounded TurtleBot 4 behaviors',license='Apache-2.0',
    entry_points={'console_scripts':[
        'gesture_recognition_node = turtlebot4_gesture_control.gesture_recognition_node:main',
        'robot_behavior_node = turtlebot4_gesture_control.robot_behavior_node:main',
        'prepare_simulation = turtlebot4_gesture_control.prepare_simulation:main']})
