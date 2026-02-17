from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup

package_name = 'robot_patrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='danielaugustin2027@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'square_move = robot_patrol.square_move:main',
            'ball_chaser = robot_patrol.ball_chaser:main',
            'servo_test_node = robot_patrol.servo_test_node:main',
            'servo_control = robot_patrol.servo_control:main',
        ],
    },
)
