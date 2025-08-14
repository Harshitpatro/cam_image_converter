from setuptools import setup
import os
from glob import glob

package_name = 'image_converter_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'cv_bridge'],
    zip_safe=True,
    maintainer='Harshit Patro',
    maintainer_email='harshitpatro123@gmail.com',
    description='ROS2 package for video conversion using usb_cam',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_converter_node = image_converter_pkg.image_converter_node:main',
        ],
    },
)