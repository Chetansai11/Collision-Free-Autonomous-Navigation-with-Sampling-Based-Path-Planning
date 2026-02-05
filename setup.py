from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'projectz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        ('share/' + package_name + '/launch', ['launch/nav_launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/nav.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chetansai',
    maintainer_email='chetansai@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner = projectz.path_planner:main',
            'controller = projectz.controller:main'
        ],
    },
)
