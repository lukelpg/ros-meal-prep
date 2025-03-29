from setuptools import find_packages, setup

package_name = 'image_processing_pkg'

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
    description='ROS2 package for image processing that publishes strokes to painting_pkg',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_processing_node = image_processing_pkg.image_processing_node:main'
        ],
    },
)
