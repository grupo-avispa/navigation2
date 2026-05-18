from glob import glob
import os

from setuptools import setup, find_packages

package_name = 'nav2_planner_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name, 'params'),
            glob(os.path.join('params', '*.yaml'))),
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
            'nav2_planner = nav2_planner_py.planner_server:main',
        ],
    },
)
