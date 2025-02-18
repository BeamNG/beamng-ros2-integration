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

# Option 1: Old workflow - this directly uses the rosdoc2 CLI interface to generate docs for single ROS packages
# Disadvantages: it generates two sets of independent HTML files (one for each ROS package)
# Advantages: out of the box solution
# This option is currently disabled.
if false; then
    rosdoc2 build -p beamng_ros2
    rosdoc2 build -p beamng_msgs
fi

# Option 2: New workflow - this uses a customized Sphinx config and builds upon some rosdoc2 internal functions
# Advantages: Single (combined) documentation for multiple ROS packages, fully customizable, compatible with sphinx-multiversion
# Disadvantages: uses a more manual setup
sphinx-build docs docs_output
