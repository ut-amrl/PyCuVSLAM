from setuptools import find_packages, setup

package_name = 'pycuvslam_ros2'

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
    maintainer='root',
    maintainer_email='domlee@utexas.edu',
    description='ROS2 wrapper for PyCuVSLAM',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_node = pycuvslam_ros2.main_node:main',
        ],
    },
)
