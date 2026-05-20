from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aurelia_x6_mavlink'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Hunt',
    maintainer_email='david.hunt@duke.edu',
    description='MAVROS telemetry bridge for the Aurelia X6 hexacopter',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_tf_node = aurelia_x6_mavlink.odom_tf_node:main',
            'diagnostics_node = aurelia_x6_mavlink.diagnostics_node:main',
        ]
    },
)
