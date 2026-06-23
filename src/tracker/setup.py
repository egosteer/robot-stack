from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'tracker'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EgoSteer Team',
    maintainer_email='egosteer@outlook.com',
    description='Vive tracker publisher for human-in-the-loop teleoperation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tracker_node = tracker.tracker_node:main',
            'robot_tracker_node = tracker.tracker_node:main',
        ],
    },
)
