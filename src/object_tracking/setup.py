from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'object_tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),  # Add launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dyllon Preston',
    maintainer_email='dpreston9@gatech.edu',
    description='Simple object tracking package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'select_object = object_tracking.select_object:main',
            'find_object = object_tracking.find_object:main',
            'rotate_robot = object_tracking.rotate_robot:main',
        ],
    },
)
