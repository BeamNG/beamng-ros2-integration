# BeamNG ROS2 Integration

## About

This repository contains packages to support the interoperability between [BeamNG.tech](https://beamng.tech/) and [ROS2](https://www.ros.org/).
BeamNG.tech is a driving simulation platform, suitable for commercial and academic use.
Inquiries for academic use can be made through our [registration form](https://register.beamng.tech/).
For inquiries regarding commercial use, contact us at <licensing@beamng.gmbh>.

## Table of Contents

- [BeamNG ROS2 Integration](#beamng-ros2-integration)
  - [About](#about)
  - [Table of Contents](#table-of-contents)
  - [Documentation](#documentation)
  - [Features](#features)
  - [Prerequisites](#prerequisites)
    - [Steps](#steps)
    - [Optional Dependencies](#optional-dependencies)
  - [Getting Started](#getting-started)
    - [Steps](#steps-1)
  - [Compatibility](#compatibility)
  - [Troubleshooting](#troubleshooting)
    - [Known Issues](#known-issues)
  - [Contributions](#contributions)

<a name="docs"></a>

## Documentation
The documentation is generated using `rosdoc2` and can be found at [https://beamngpy.readthedocs.io/en/latest/_static/beamng_ros2](https://beamngpy.readthedocs.io/en/latest/_static/beamng_ros2).

The [list of ROS2 messages, services and actions](https://beamngpy.readthedocs.io/en/latest/_static/beamng_msgs/) is also available online.

## Features

The BeamNG ROS2 integration includes support for the **remote** control of the simulation platform, attaching sensors to vehicles (using scenario files), and getting the sensor data from vehicles.

<a name="prereqs"></a>

## Prerequisites

For using the BeamNG ROS2 integration, a BeamNG.tech build and a Python environment with [BeamNGpy][1] installed are required. BeamNGpy has to have compatible version with the BeamNG.tech installation.

For sensor shared memory support, BeamNG.tech and ROS have to be running on the **same machine** and the **same operating system** (Windows Subsystem for Linux will not work with shared memory). Shared memory on Linux is **not working** in the current BeamNG.tech release and the support will come in the next major update.

The BeamNG ROS2 integration is tested with the following ROS2 distributions: [Foxy Fitzroy](https://docs.ros.org/en/foxy) and [Humble Hawksbill](https://docs.ros.org/en/humble/index.html).

### Steps
1. Setup your ROS2 Humble/Foxy workspace.
2. Clone this repository into your ROS2 workspace.
3. Use [rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html#rosdep-operation) to install the dependencies for the package.
4. Install `beamngpy` using pip or conda in the Python environment you use to run the ROS2 nodes.
5. To run the bridge, run `ros2 run beamng_ros2 beamng_bridge`.

### Optional Dependencies
If available, BeamNG-ROS2 will use the `python-rapidjson` library to support JSON files which do not strictly follow the standard:
  ```bash
  pip install json5
  ```

<a name="getstart"></a>

## Getting Started

To use this project, a basic knowledge of the BeamNG.tech simulator and the BeamNGpy is neccessary. We recommend to familiarize yourself first with [BeamNGpy][1] to get a basic understanding of the platform before using the BeamNG ROS2 Integration.

### Steps
1. After setting up BeamNG.tech and BeamNGpy with a Python environment, the simulation needs to be listening on the BeamNGpy socket. To do that, you can run the executable with the following arguments:

  Command prompt (`cmd.exe`):
  ```bat
  Bin64\BeamNG.tech.x64.exe -console -nosteam -tcom-listen-ip <LISTEN_IP> -lua extensions.load('tech/techCore');tech_techCore.openServer(64256)
  ```

  PowerShell:
  ```posh
  Bin64\BeamNG.tech.x64.exe -console -nosteam -tcom-listen-ip <LISTEN_IP> -lua "extensions.load('tech/techCore');tech_techCore.openServer(64256)"
  ```

  `<LISTEN_IP>` will be `127.0.0.1` in the case of running BeamNG.tech on the same machine and operating system as the ROS2 interface. If you are running ROS2 using WSL or want to connect to a remote machine, you need to set the listen IP to the one of the corresponding network interface.

2. The ROS2 packages from this repository need to be added and built in your colcon workspace. See the [ROS2 tutorials](https://docs.ros.org/en/humble/Tutorials.html) for more information on how to set up a colcon workspace.

  ```bash
  source /opt/ros/humble/setup.bash
  cd ~/ros2_ws/src
  git clone https://github.com/BeamNG/beamng-ros2-integration.git
  cd ~/ros2_ws
  rosdep install -i --from-path src --rosdistro humble -y
  colcon build # or colcon build --symlink-install
  ```

3. A node connecting ROS2 to the simulation can be started using the `ros2 run` command:

  ```bash
  ros2 run beamng_ros2 beamng_bridge
  ```

  It needs to be configured to contain the correct IP address of the machine hosting the simulation.

4. After running the bridge, you can set the IP address of the BeamNG host by calling:

  ```bash
  ros2 param set /beamng_bridge host <BEAMNG_ADDRESS>
  ```

5. To operate the node, you can use the exposed services, you can find their list in [https://beamngpy.readthedocs.io/en/latest/_static/beamng_msgs/interfaces/service_definitions.html](https://beamngpy.readthedocs.io/en/latest/_static/beamng_msgs/interfaces/service_definitions.html). Getting access to sensor data requires you to launch a scenario using the [StartScenario](https://beamngpy.readthedocs.io/en/latest/_static/beamng_msgs/interfaces/srv/StartScenario.html) service:

   ```bash
   ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario "{path_to_scenario_definition: '/config/scenarios/example_tech_ground.json'}"

Using it will start up a node that connects to the simulation and starts up a scenario as defined in the `beamng_ros2/config/scenarios/example_tech_ground.json`.
Other scenario specifications are available in the same directory.

## Compatibility

Running the BeamNG ROS2 integration requires three individual software components, here is a list of compatible versions.

| BeamNG.tech | BeamNGpy | BeamNG ROS2 Integration |
|-------------|----------|-------------------------|
| 0.32        | 1.29     |  1.0.0                  |

## Troubleshooting

This section lists common issues with  BeamNG ROS2 Integration in particular. Since this
library is closely tied to BeamNG.tech and thus BeamNG.drive, it is also
recommended to consult the documentation on BeamNG.drive here:

[https://documentation.beamng.com/][8]

### Known Issues
These are issues that we are working on and there is no need to report them:

- Lidar sensor may cause the simulation to crash.
- The integration is using a single thread causing unoptimal performance.
  - BeamNGpy is not threadsafe at the moment.

## Contributions

We always welcome user contributions, be sure to check out our [contribution guidelines][9] first, before starting your work.

[1]: https://github.com/BeamNG/BeamNGpy
[8]: https://documentation.beamng.com/
[9]: https://github.com/BeamNG/BeamNG-ROS2-Integration/blob/main/CONTRIBUTING.md
