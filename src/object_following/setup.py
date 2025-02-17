from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'object_following'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name), glob('launch/*.launch.py')),  # Add launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dyllon Preston',
    maintainer_email='dpreston9@gatech.edu',
    description='Simple object following package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'select_object = object_following.select_object:main',
            'detect_object = object_following.detect_object:main',
            'object_range = object_following.object_range:main',
            'chase_object = object_following.chase_object:main',
        ],
    },
)
