from setuptools import find_packages, setup

package_name = 'serial_comm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luke',
    maintainer_email='20lapg1@queensu.ca',
    description='This package handles all serial communication',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_comm = serial_comm_pkg.serial_comm:main',
        ],
    },
)
