from setuptools import find_packages, setup

package_name = 'mecanum_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/mecanum_joy.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='doanh',
    maintainer_email='doanh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_mecanum = mecanum_joy.joy_mecanum:main',
        ],
    },
)
