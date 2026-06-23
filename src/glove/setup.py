from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'glove'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EgoSteer Team',
    maintainer_email='egosteer@outlook.com',
    description='Glove input nodes for EgoSteer teleoperation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'glove_node = glove.glove_node:main',
        ],
    },
)
