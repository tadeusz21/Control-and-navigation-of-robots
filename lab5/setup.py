from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lab5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),  
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),  
        (os.path.join('share', package_name, 'maps/flat'), glob(os.path.join('maps/flat', '*'))), 
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*'))), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_nav = lab5.simple_nav:main',
            'waypoint_nav = lab5.waypoint_nav:main',
            'waypoint_nav_client = lab5.waypoint_nav_client:main',
            'follow_waypoint_nav_client = lab5.follow_waypoint_nav_client:main',
            'follow_waypoint_nav_server = lab5.follow_waypoint_nav_server:main'
        ],
    },
)
