from glob import glob
import os
from setuptools import setup
package_name = 'robot_base_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line grabs all launch files:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # This line grabs all config yaml files:
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_driver = robot_base_driver.base_driver:main',
            'bno085_node = robot_base_driver.bno085_node:main',
        ],
    },
)
