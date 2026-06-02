from setuptools import setup
import os
from glob import glob

package_name = 'farm_twin_poc'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Terra Minds',
    maintainer_email='team5@student.tue.nl',
    description='Farm Twin PoC — TurtleBot3 Burger Digital Twin',
    license='MIT',
    entry_points={
        'console_scripts': [
            'safety_stop_node  = farm_twin_poc.safety_stop_node:main',
            'twin_safety_node  = farm_twin_poc.twin_safety_node:main',
            'zone_monitor_node = farm_twin_poc.zone_monitor_node:main',
            'dt_logger_node    = farm_twin_poc.dt_logger_node:main',
            'navigator_node    = farm_twin_poc.navigator:main',
            'nav2_navigator    = farm_twin_poc.navigator_node:main',
        ],
    },
)