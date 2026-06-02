from setuptools import setup, find_packages

package_name = 'nav2_core_py'

setup(
    name=package_name,
    version='1.4.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alberto Tudela',
    maintainer_email='ajtudela@gmail.com',
    description='Nav2 core interfaces and exceptions for Python plugins',
    license='Apache-2.0',
    tests_require=['pytest'],
)
