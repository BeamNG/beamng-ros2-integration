from setuptools import setup

package_name = "drone_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@example.com",
    description="ROS2 drone controller node",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["drone_controller = drone_controller.drone_controller:main"],
    },
)


