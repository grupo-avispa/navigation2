from setuptools import find_packages, setup

package_name = 'nav2_costmap_2d_py'

setup(
    name=package_name,
    version='1.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}', ['costmap_plugins.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alberto Tudela',
    maintainer_email='ajtudela@gmail.com',
    description='Implementation of a 2D costmap that takes in sensor data from the world',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'costmap_2d = nav2_costmap_2d_py.Costmap2DROS:main',
        ],
    },
)
