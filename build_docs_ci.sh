#!/bin/bash
# This script can run inside a Docker container using the ros:humble-ros-base-jammy image
# Run container:
# sudo docker run --rm -v $PWD:/repo -w /repo ros:humble-ros-base-jammy bash build_docs_ci.sh
# Artifacts will be put to "docs_output" directory
set -eo pipefail

# Install dependencies
apt update
apt install python3-pip -y
pip install beamngpy rosdoc2

# Cleanup
rm -rf install log build docs_output docs_build cross_reference

# ROS build
colcon build --symlink-install
source install/setup.bash

# Create docs
rosdoc2 build -p beamng_ros2
rosdoc2 build -p beamng_msgs
cp media/root_template.html docs_output/index.html
