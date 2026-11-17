import os
from glob import glob

from setuptools import setup

package_name = 'tb3_point_to_point'


def files(pattern):
    return sorted(f for f in glob(pattern) if os.path.isfile(f))


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), files('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), files('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), files('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), files('worlds/*.world') + files('worlds/*.sdf')),
        (os.path.join('share', package_name, 'maps'), files('maps/*.yaml') + files('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},   # lets `colcon test` run the pytest suite
    zip_safe=True,
    maintainer='Bharathwaj Manoharan',
    maintainer_email='bharathwajm.works@gmail.com',
    description='Indoor point-to-point autonomous navigation for TurtleBot3 (ROS 2 + Gazebo + Nav2)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'send_goal = tb3_point_to_point.send_goal:main',
            'go_to = tb3_point_to_point.go_to:main',
            'mission_runner = tb3_point_to_point.mission_runner:main',
            'explore_and_map = tb3_point_to_point.explore_and_map:main',
            'room_labels = tb3_point_to_point.room_labels:main',
            'analyze_results = tb3_point_to_point.analyze_results:main',
        ],
    },
)
