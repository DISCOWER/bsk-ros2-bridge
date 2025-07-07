from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'bsk_ros_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'examples'],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(package_name +'/launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyzmq', 'orjson'],
    zip_safe=True,
    maintainer='Elias Krantz',
    maintainer_email='eliaskra@kth.se',
    description='ROS2 Bridge for Basilisk',
    license='BSD-3',
    extras_require={
        'test': ['pytest']
    },
    entry_points={
        'console_scripts': [
            'bsk_ros_bridge = bsk_ros_bridge.bsk_ros_bridge:main',
            'example_data_processor = examples.ex_data_processor:main',
        ],
    },
)
