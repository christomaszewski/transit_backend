from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'transit_backend'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'websockets', 'aiohttp'],
    zip_safe=True,
    maintainer='transit',
    maintainer_email='dev@transit.local',
    description='TRANSIT path planning backend',
    license='MIT',
    entry_points={
        'console_scripts': [
            'transit_node = transit_backend.transit_node:main',
        ],
    },
)
