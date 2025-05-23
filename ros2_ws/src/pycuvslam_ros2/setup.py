from setuptools import find_packages, setup
from glob import glob

package_name = 'pycuvslam_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='domlee@utexas.edu',
    description='ROS2 wrapper for VisualWheelOdometry',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vwo_node = pycuvslam_ros2.main_node:main',
        ],
    },
)
