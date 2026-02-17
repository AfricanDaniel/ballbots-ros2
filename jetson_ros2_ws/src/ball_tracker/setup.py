from setuptools import find_packages, setup

package_name = 'ball_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/tennis_ball_tracker.launch.xml']),
        ('share/' + package_name + '/config',
         ['config/tennis_visualization.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='msr',
    maintainer_email='msr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'number_publisher = ball_tracker.number_publisher:main',
            'tennis_ball_tracker = ball_tracker.tennis_ball_tracker:main',
        ],
    },
)
