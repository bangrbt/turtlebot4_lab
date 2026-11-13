from glob import glob
from setuptools import setup

package_name = 'turtlebot4_facial_emotion_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'requirements.txt']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lê Anh Tuấn Bằng',
    maintainer_email='banglat.rbt@gmail.com',
    description='Facial Emotion Recognition via Webcam (DeepFace / FER / OpenCV) and TurtleBot 4 Trajectory Control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emotion_recognition_node = turtlebot4_facial_emotion_control.emotion_recognition_node:main',
            'robot_behavior_node = turtlebot4_facial_emotion_control.robot_behavior_node:main',
            'emotion_gui = turtlebot4_facial_emotion_control.emotion_gui:main',
            'emotion_pipeline = turtlebot4_facial_emotion_control.emotion_pipeline:main',
        ],
    },
)
