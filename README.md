# BeamNG ROS2 Integration

[![Documentation](https://img.shields.io/badge/Documentation-blue?logo=googledocs&logoColor=white)](https://documentation.beamng.com/api/ros2)
[![Repository](https://img.shields.io/badge/Repository-grey?logo=github&logoColor=white)](https://github.com/BeamNG/beamng-ros2-integration)

This integration is for ROS2, for ROS1 check [BeamNG ROS Integration](https://github.com/BeamNG/beamng-ros-integration).

## About

This repository contains packages to support the interoperability between [BeamNG.tech](https://beamng.tech/) and [ROS2](https://www.ros.org/).
BeamNG.tech is a driving simulation platform, suitable for commercial and academic use.
Inquiries for academic use can be made through our [registration form](https://register.beamng.tech/).
For inquiries regarding commercial use, contact us at <licensing@beamng.gmbh>.

## Table of Contents

- [Features](#features)
- [Prerequisites](#prereqs)
  - [Steps](#steps)
  - [Optional Dependencies](#optional-dependencies)
- [Getting Started](#getstart)
  - [Steps](#steps)
- [Compatibility](#compatibility)
- [Troubleshooting](#troubleshooting)
  - [Known Issues](#known-issues)
- [Contributions](#contributions)

## Features

The BeamNG ROS2 integration includes support for the **remote** control of the simulation platform, attaching sensors to vehicles (using scenario files), and getting the sensor data from vehicles.

<a name="prereqs"></a>

## Prerequisites

For using the BeamNG ROS2 integration, a BeamNG.tech build and a Python environment with [BeamNGpy][1] installed are required. BeamNGpy has to have compatible version with the BeamNG.tech installation.

For sensor shared memory support, BeamNG.tech and ROS have to be running on the **same machine** and the **same operating system** (Windows Subsystem for Linux will not work with shared memory). Shared memory on Linux is **not working** in the current BeamNG.tech release and the support will come in the next major update.

The BeamNG ROS2 integration is tested with the following ROS2 distributions: [Foxy Fitzroy](https://docs.ros.org/en/foxy) and [Humble Hawksbill](https://docs.ros.org/en/humble/index.html).

<a name="steps"></a>

### Steps
1. Setup your ROS2 Humble/Foxy workspace. See
   * [Install ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
   * [Setup ROS workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
2. Clone this repository into your ROS2 workspace.
3. Use [rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html#rosdep-operation) to install the dependencies for the package. See:
   * [How do I use the rosdep tool](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html#how-do-i-use-the-rosdep-tool)
4. Install `beamngpy` using pip or conda in the Python environment you use to run the ROS2 nodes.
5. To run the bridge, run `ros2 run beamng_ros2 beamng_bridge`.

### Optional Dependencies
If available, BeamNG-ROS2 will use the `python-rapidjson` library to support JSON files which do not strictly follow the standard:

  ```bash
  pip install python-rapidjson
  ```

<a name="getstart"></a>

## Getting Started

To use this project, a basic knowledge of the BeamNG.tech simulator and the BeamNGpy is neccessary. We recommend to familiarize yourself first with [BeamNGpy][1] to get a basic understanding of the platform before using the BeamNG ROS2 Integration.

### Steps
1. After setting up BeamNG.tech and BeamNGpy with a Python environment, the simulation needs to be listening on the BeamNGpy socket. To do that, you can run the executable with the following arguments:

  Command prompt (`cmd.exe`):
  ```bat
  Bin64\BeamNG.tech.x64.exe -console -nosteam -tcom-listen-ip <LISTEN_IP> -lua extensions.tech_techCore.openServer(25252)
  ```

  PowerShell:
  ```posh
  Bin64\BeamNG.tech.x64.exe -console -nosteam -tcom-listen-ip <LISTEN_IP> -lua "extensions.tech_techCore.openServer(25252)"
  ```

  Linux Terminal:
  ```bash
  BinLinux/BeamNG.tech.x64 -nosteam -tcom-listen-ip <LISTEN_IP> -lua "extensions.tech_techCore.openServer(25252)"
  ```

  `<LISTEN_IP>` will be `127.0.0.1` in the case of running BeamNG.tech on the same machine and operating system as the ROS2 interface. If you are running ROS2 using WSL or want to connect to a remote machine, you need to set the listen IP to the one of the corresponding network interface.

2. The ROS2 packages from this repository need to be added and built in your colcon workspace. See the [ROS2 tutorials](https://docs.ros.org/en/humble/Tutorials.html) for more information on how to set up a colcon workspace.
   If you use ROS2 on Windows (without WSL), the `rosdep` command does not work, you will have to skip installing dependencies or install them manually. The integration should still work on that configuration.

  ```bash
  source /opt/ros/humble/setup.bash
  cd ~/ros2_ws/src
  git clone https://github.com/BeamNG/beamng-ros2-integration.git
  cd ~/ros2_ws
  rosdep install -i --from-path src --rosdistro humble -y
  colcon build # or colcon build --symlink-install
  ```

1. A node connecting ROS2 to the simulation can be started using the `ros2 run` command:

  ```bash
  ros2 run beamng_ros2 beamng_bridge
  ```

  It needs to be configured to contain the correct IP address of the machine hosting the simulation.

4. After running the bridge, you can set the IP address of the BeamNG host by calling:

  ```bash
  ros2 param set /beamng_bridge host <BEAMNG_ADDRESS>
  ```

5. To operate the node, you can use the exposed services, you can find their list in [Service Definitions](https://documentation.beamng.com/api/ros2/main/beamng_msgs/interfaces/service_definitions.html). Getting access to sensor data requires you to launch a scenario using the [StartScenario](https://documentation.beamng.com/api/ros2/main/beamng_msgs/interfaces/srv/StartScenario.html) service:

  ```bash
  ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario "{path_to_scenario_definition: '/config/scenarios/example_tech_ground.json'}"
  ```

Using it will start up a node that connects to the simulation and starts up a scenario as defined in the `beamng_ros2/config/scenarios/example_tech_ground.json`.
Other scenario specifications are available in the same directory.

## Compatibility

Running the BeamNG ROS2 integration requires three individual software components, here is a list of compatible versions.

| BeamNG.tech | BeamNGpy | BeamNG ROS2 Integration |
|-------------|----------|-------------------------|
| 0.34        | 1.31     |  1.2.0                  |
| 0.33        | 1.30     |  1.1.0                  |
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

The Python code is formatted using [Black](https://github.com/psf/black), please use it to
format the code you want to contribute.
