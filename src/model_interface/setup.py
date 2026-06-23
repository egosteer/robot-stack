from setuptools import setup
import os
from glob import glob

package_name = 'model_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.utils'], # include sub-package
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EgoSteer Team',
    maintainer_email='egosteer@outlook.com',
    description='VLA Model Interface',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'model_interface_node = model_interface.model_interface_node:main',
        ],
    },
)