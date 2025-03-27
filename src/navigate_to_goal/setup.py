from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'navigate_to_goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')), # Add world file
        (os.path.join('share', package_name), glob('launch/*.launch.py')),  # Add launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dyllon Preston',
    maintainer_email='dpreston9@gatech.edu',
    description='Simple navigation with object avoidance',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actual_odom = navigate_to_goal.actual_odom:main',
            'object_range = navigate_to_goal.object_range:main',
            'debug_range = navigate_to_goal.debug_range:main',
            'goal_manager = navigate_to_goal.goal_manager:main',
            'state_manager = navigate_to_goal.state_manager:main',
            'go_to_goal = navigate_to_goal.go_to_goal:main',
            'slam = navigate_to_goal.slam:main',
            'slam_debug = navigate_to_goal.slam_debug:main',
        ],
    },
)
