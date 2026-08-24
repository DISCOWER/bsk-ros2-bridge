from setuptools import setup
from glob import glob
import os

package_name = 'bsk-ros2-bridge'
package_dir = 'bsk_ros2_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[
        package_dir,
        f'{package_dir}.modules',
        'examples',
        'examples.utils',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(package_dir +'/launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyzmq>=24.0.0', 'orjson>=3.8.0'],
    zip_safe=True,
    maintainer='Elias Krantz',
    maintainer_email='eliaskra@kth.se',
    description='ROS 2 Bridge for Basilisk Astrodynamics Framework',
    license='BSD-3-Clause',
    extras_require={
        'test': ['pytest']
    },
    entry_points={
        'console_scripts': [
            'bsk-ros2-bridge = bsk_ros2_bridge.bsk_ros2_bridge:main',
        ],
    },
)
