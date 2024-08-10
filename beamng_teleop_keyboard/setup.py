from setuptools import setup

package_name = 'beamng_teleop_keyboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer="BeamNG GmbH",
    maintainer_email="tech@beamng.gmbh",
    description="Integration of BeamNG.tech into the ROS2 ecosystem.",
    license="MIT",
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = beamng_teleop_keyboard.teleop:main',
        ],
    },
)
