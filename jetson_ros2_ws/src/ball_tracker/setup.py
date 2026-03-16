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
        ('share/' + package_name, ['ball_tracker/realsense_best_train_model.pt']),
        ('share/' + package_name, ['ball_tracker/zed_best_train_model.pt']),
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
            'hsv_tuner = ball_tracker.hsv_tuner:main',
            'realsense_tracker = ball_tracker.realsense_tracker:main',
            'realsense_tracker_yolo = ball_tracker.realsense_tracker_yolo:main',
            'zed_tracker = ball_tracker.zed_tracker:main',
            'zed_tracker_yolo = ball_tracker.zed_tracker_yolo:main',
        ],
    },
)