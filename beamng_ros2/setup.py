import os
from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "beamng_ros2"


def generate_data_files():
    data_files = [(os.path.join("share", package_name, "config"), ["config/sensors.json"])]
    data_dir = "config"
    for dir in glob(data_dir + "/*"):
        if Path(dir).is_file():
            continue
        data_files.append((os.path.join("share", package_name, dir), glob(dir + "/**")))
    return data_files


setup(
    name=package_name,
    packages=find_packages(),
    version="1.0.0",
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *generate_data_files(),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BeamNG GmbH",
    maintainer_email="tech@beamng.gmbh",
    description="Integration of BeamNG.tech into the ROS2 ecosystem.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "beamng_bridge = beamng_ros2.beamng:main",
            "example_client = beamng_ros2.examples.example_client:main",
        ],
    },
)
