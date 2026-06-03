from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'nav2_controller_py'

setup(
    name=package_name,
    version='1.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugins.xml']),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alberto Tudela',
    maintainer_email='ajtudela@gmail.com',
    description='Python port of the Nav2 controller server',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_server = nav2_controller_py.controller_server:main',
        ],
    },
)
