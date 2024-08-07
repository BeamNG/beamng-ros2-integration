from setuptools import setup

package_name = 'beamng_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'beamng_agent.beamng_agent',
        'beamng_agent.twist2bng'
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
            'beamng_agent = beamng_agent.beamng_agent:main'
        ],
    },
)
