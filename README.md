# BeamNG ROS2 Integration

## About

This repository contains packages to support the interoperability between [BeamNG.tech](https://beamng.tech/) and ROS2.
BeamNG.tech is a driving simulation platform, suitable for commercial and academic use.
Free licenses are available for non-commercial and academic use.
Inquiries can be made through our [registration form](https://register.beamng.tech/).
For inquiries regarding commercial use, contact us at <licensing@beamng.com>.
## Table of Contents

<!--
 - [Documentation](#docs)
 -->
 - [Features](#features) 
 - [Prerequirements](#prereqs)
 - [Getting Started](#getstart)
 - [Compatibility](#compatibility)
 - [Troubleshooting](#troubleshooting)
 
<!-- 
<a name="docs"></a>
## Documentation
[![](https://raw.githubusercontent.com/ChristianBirchler/sdc-scissor/main/docs/images/readthedocs.png)](https://beamngpy.readthedocs.io/en/latest/bngros.html)
-->
## Features

As of now the BeamNG ROS2 integration supports one package for the **remote** control of the simulation platform and one package for the control of a driving agent. A third package manages custom messages.

<a name="prereqs"></a>

## Prerequirements

For using the BeamNG ROS2 integration, a BeamNG.tech build and a python environment with [BeamNGpy][1] installed are required.

Note that BeamNG.tech **only runs on Window**, although Linux support is on its way.
That means that BeamNG.tech needs to run on a separate Windows machine, or that ROS2 needs to run on on [WSL2](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support#2-install-wsl).

The BeamNG ROS2 integration is compatible with the ROS2 distributions [humble hawksbill](https://docs.ros.org/en/humble/index.html) .  

<a name="getstart"></a>

## Getting Started

To use this project, a basic knowledge of the BeamNG.tech simulator and the BeamNGpy is neccessary. We recommend to familiarize yourself first with [BeamNGpy][1] to get a basic understanding of the platform before using the BeamNG ROS2 Integration.

After setting up BeamNG.tech and BeamNGpy with a python environment, the simulation needs to be started through BeamNGpy.

The ROS2 packages from this repository need to be added and build in your colcon workspace.
See the [ROS2 tutorials](https://docs.ros.org/en/humble/Tutorials.html) for more information on how to set up a colcon workspace.

A node connecting ROS2 to the simulation can then be started with the help of the `bridge.py` file in the `beamng_control` package through the command:

```shell
ros2 run beamng_control bridge
```

It needs to be configured to contain the correct IP address of the machine hosting the simulation.
Using it will start up a node that connects to the simulation and starts up a scenario as defined in the `beamng_control/config/simple_scenario.json`.
Other scenario specifications are available in the same directory.

## Compatibility  

Running the BeamNG ROS2 integration requires three individual software components, here is a list of compatible versions.

| BeamNG.tech | BeamNGpy | BeamNG ROS2 Integration |
|-------------|----------|-------------------------|
| 0.31        |1.28      |  0.1.0                  |



## Troubleshooting

This section lists common issues with  BeamNG ROS2 Integration in particular. Since this
library is closely tied to BeamNG.tech and thus BeamNG.drive, it is also
recommended to consult the documentation on BeamNG.drive here:

[https://documentation.beamng.com/][8]

<!--
## Contributions

We always welcome user contributions, be sure to check out our [contribution guidelines][9] first, before starting your work.
-->
[1]: https://github.com/BeamNG/BeamNGpy
[8]: https://documentation.beamng.com/
[9]: https://github.com/BeamNG/BeamNG-ROS-Integration/blob/master/contributing.md
