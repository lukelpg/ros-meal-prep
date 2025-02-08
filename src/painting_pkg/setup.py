from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'painting_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # Ensure launch files are installed
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luke',
    maintainer_email='20lapg1@queensu.ca',
    description='Package for generating painting strokes and publishing instructions to the robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'painting_node = painting_pkg.painting_node:main',  # Add executable nodes here
        ],
    },
)
