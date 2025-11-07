from setuptools import setup
from glob import glob
import os

package_name = 'bsk-ros2-bridge'
package_dir = 'bsk_ros2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_dir, 'examples'],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(package_dir +'/launch/*.launch.py')),
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
            'bsk-ros2-bridge = bsk_ros2_bridge.bsk_ros2_bridge:main',
            'dummy-data-processor = examples.dummyDataProcessor:main',
            'stresstest_node = examples.stresstest_node:main',
        ],
    },
)
