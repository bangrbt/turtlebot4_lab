from glob import glob
from setuptools import setup

package_name = 'turtlebot4_voice_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'VALIDATION.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py') + glob('scripts/*.bash')),
        ('share/' + package_name + '/sample_audio', glob('sample_audio/*')),
        ('share/' + package_name + '/docs', glob('docs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lê Anh Tuấn Bằng',
    maintainer_email='maintainer@example.com',
    description='Map segmentation and voice-controlled semantic navigation for TurtleBot 4',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'voice_recognizer_node = turtlebot4_voice_navigation.voice_recognizer_node:main',
            'voice_navigation_manager = turtlebot4_voice_navigation.voice_navigation_manager:main',
            'zone_visualizer_node = turtlebot4_voice_navigation.zone_visualizer_node:main',
        ],
    },
)
