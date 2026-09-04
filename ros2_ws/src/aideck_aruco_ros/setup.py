import os
from glob import glob

from setuptools import setup

package_name = 'aideck_aruco_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaewoo',
    maintainer_email='imdaniel01@naver.com',
    description='AI-Deck UDP ArUco streams published as ROS 2 topics.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aideck_aruco_node = aideck_aruco_ros.aideck_aruco_node:main',
        ],
    },
)
