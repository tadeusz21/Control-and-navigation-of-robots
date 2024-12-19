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
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),  # Dodaj folder config
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),  # Zawiera folder worlds
        (os.path.join('share', package_name, 'maps/flat'), glob(os.path.join('maps/flat', '*'))), 
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
            'simple_nav = lab5.simple_nav:main'
        ],
    },
)
