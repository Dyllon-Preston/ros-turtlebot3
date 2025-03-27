from setuptools import find_packages, setup

package_name = 'waypoint_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dyllon Preston',
    maintainer_email='dpreston9@gatech.edu',
    description='Simple waypoint navigation manager using Navigation2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_manager = waypoint_nav.waypoint_manager:main',
        ],
    },
)
