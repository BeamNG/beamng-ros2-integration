# BeamNG ROS2 Integration

## About

This repository contains packages to support the interoperability between [BeamNG.tech](https://beamng.tech/) and [ROS2](https://www.ros.org/).
BeamNG.tech is a driving simulation platform, suitable for commercial and academic use.
Inquiries for academic use can be made through our [registration form](https://register.beamng.tech/).
For inquiries regarding commercial use, contact us at <licensing@beamng.gmbh>.

## Table of Contents

<!--
 - [Documentation](#docs)
 -->
- [BeamNG ROS2 Integration](#beamng-ros2-integration)
  - [About](#about)
  - [Table of Contents](#table-of-contents)
  - [Documentation](#documentation)
  - [Features](#features)
  - [Prerequirements](#prerequirements)
    - [Optional Dependencies](#optional-dependencies)
  - [Getting Started](#getting-started)
  - [Compatibility](#compatibility)
  - [Troubleshooting](#troubleshooting)
  - [Contributions](#contributions)

<a name="docs"></a>

## Documentation
The documentation is generated using `rosdoc2` and can be found at [https://beamngpy.readthedocs.io/en/latest/_static/beamng_ros2](https://beamngpy.readthedocs.io/en/latest/_static/beamng_ros2).

## Features

As of now the BeamNG ROS2 integration supports one package for the **remote** control of the simulation platform and one package for the control of a driving agent. A third package manages custom messages.

<a name="prereqs"></a>

## Prerequirements

For using the BeamNG ROS2 integration, a BeamNG.tech build and a Python environment with [BeamNGpy][1] installed are required. BeamNGpy has to have compatible version with the BeamNG.tech installation.

For sensor shared memory support, BeamNG.tech and ROS have to be running on the **same machine** and the **same operating system** (Windows Subsystem for Linux will not work). You can run ROS2

That means that BeamNG.tech needs to run on a separate Windows machine, or that ROS2 needs to run on on [WSL2](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support#2-install-wsl).

The BeamNG ROS2 integration is tested with the following ROS2 distributions [Foxy Fitzroy](https://docs.ros.org/en/foxy) and [Humble Hawksbill](https://docs.ros.org/en/humble/index.html) .

### Optional Dependencies
If available, BeamNG-ROS2 will use the `python-rapidjson` library
```bash
pip install json5
```

<a name="getstart"></a>

## Getting Started

To use this project, a basic knowledge of the BeamNG.tech simulator and the BeamNGpy is neccessary. We recommend to familiarize yourself first with [BeamNGpy][1] to get a basic understanding of the platform before using the BeamNG ROS2 Integration.

After setting up BeamNG.tech and BeamNGpy with a Python environment, the simulation needs to be listening on the BeamNGpy socket. To do that, you can run the executable with the following arguments:
```bash
Bin64\BeamNG.tech.x64.exe -console -nosteam -tcom-listen-ip <LISTEN_IP> -lua extensions.load('tech/techCore');tech_techCore.openServer(64256)
```

`<LISTEN_IP>` will be `127.0.0.1` in the case of running BeamNG.tech on the same machine and operating system as the ROS2 interface. If you are running ROS2 using WSL or want to connect to a remote machine, you need to set the listen IP to the one of the corresponding network interface.

The ROS2 packages from this repository need to be added and build in your colcon workspace.
See the [ROS2 tutorials](https://docs.ros.org/en/humble/Tutorials.html) for more information on how to set up a colcon workspace.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src
git clone https://github.com/BeamNG/beamng-ros2-integration.git
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```

A node connecting ROS2 to the simulation can then be started using the `ros2 run` command:

```shell
ros2 run beamng_ros2 beamng_bridge
```

<img src="https://github.com/BeamNG/beamng-ros2-integration/blob/main/media/ROS2_bridge.png" alt="ROS2_bridge" width="900" />

It needs to be configured to contain the correct IP address of the machine hosting the simulation.
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

## Contributions

We always welcome user contributions, be sure to check out our [contribution guidelines][9] first, before starting your work.

[1]: https://github.com/BeamNG/BeamNGpy
[8]: https://documentation.beamng.com/
[9]: https://github.com/BeamNG/BeamNG-ROS2-Integration/blob/main/CONTRIBUTING.md
