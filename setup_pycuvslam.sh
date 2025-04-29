#!/bin/bash
#  Run this script inside the Docker container to set up the environment for PyCuVSLAM.

# Install pycuvslam and requirements
pip3 install -e /pycuvslam/cuvslam/x86
pip3 install -r /pycuvslam/examples/requirements.txt

# Build ros2_ws
source /opt/ros/humble/setup.bash
cd /pycuvslam/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash