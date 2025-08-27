import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'launch_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*')))

        # (os.path.join('share', package_name, 'config'),
        #  glob(os.path.join('config', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stivietnam',
    maintainer_email='stivietnam@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = launch_pkg.publisher_member_function:main',
            'listener = launch_pkg.subscriber_member_function:main',
            'param_node = launch_pkg.param_python:main',

            'check_physical = launch_pkg.checkPhysical:main',
            'kickoff = launch_pkg.kickoff:main',
            'nuc_info= launch_pkg.checkCpu:main',
            'watchdog_node = launch_pkg.watchdog_node:main',
        ],
    },
)
