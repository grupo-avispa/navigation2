from glob import glob
import os

from setuptools import setup

package_name = 'nav2_planner_example_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steve',
    maintainer_email='stevenmacenski@gmail.com',
    description='Nav2 planner server package in python3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

        ],
    },
)
