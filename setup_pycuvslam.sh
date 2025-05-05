#!/bin/bash
#  Run this script inside the Docker container to set up the environment for PyCuVSLAM.

# Install pycuvslam and requirements
pip3 install -e /pycuvslam/cuvslam/x86
pip3 install -r /pycuvslam/examples/requirements.txt