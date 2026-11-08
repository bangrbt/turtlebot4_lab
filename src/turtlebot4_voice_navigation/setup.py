import os
from glob import glob
from setuptools import setup

package_name = 'turtlebot4_voice_navigation'

# Helper to recursively include all model files (meshes, materials, configs, sdfs)
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

model_files = []
for root, dirs, files in os.walk('models'):
    if files:
        install_dir = os.path.join('share', package_name, root)
        file_list = [os.path.join(root, f) for f in files]
        model_files.append((install_dir, file_list))

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'README.md', 'VALIDATION.md']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/config', glob('config/*')),
    ('share/' + package_name + '/maps', glob('maps/*')),
    ('share/' + package_name + '/worlds', glob('worlds/*')),
    ('share/' + package_name + '/scripts', glob('scripts/*.py') + glob('scripts/*.bash')),
    ('share/' + package_name + '/sample_audio', glob('sample_audio/*')),
    ('share/' + package_name + '/docs', glob('docs/*')),
] + model_files

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
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
            'prepare_voice_simulation = turtlebot4_voice_navigation.prepare_simulation:main',
        ],
    },
)
