C++ Bridge and ROS2 Editor
==========================

Alongside the Python bridge documented in the rest of these pages, BeamNG.tech
also ships a **native C++ integration**. Instead of running a separate ROS 2
node that talks to the simulator over the BeamNGpy socket, the C++ bridge is
loaded **inside BeamNG.tech itself** as a plugin and is configured through a
dedicated **ROS2 Editor** in the in-game World Editor.

.. note::

   The C++ integration lives on the ``cpp`` branch of the
   `beamng-ros2-integration <https://github.com/BeamNG/beamng-ros2-integration>`_
   repository and is published on GitHub as-is. This page summarizes how it
   works; for the full in-simulator tool reference, see the **ROS2 Editor**
   page in the `BeamNG.tech documentation <https://documentation.beamng.com/>`_.

Python bridge vs. C++ bridge
----------------------------

The two integrations expose vehicles and sensors to ROS 2, but differ in how
they run:

.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Aspect
     - Python bridge (this branch)
     - C++ bridge (``cpp`` branch)
   * - Runtime
     - Standalone ROS 2 node (``beamng_bridge``)
     - Native plugin (``.dll`` / ``.so``) loaded inside BeamNG.tech
   * - Configuration
     - Scenario/sensor JSON files + ROS 2 services
     - Interactive **ROS2 Editor** in the World Editor
   * - Connection
     - BeamNGpy socket (can be remote)
     - In-process; same machine and OS required
   * - Best for
     - Scripted/remote setups, cosimulation, teleop examples
     - High-performance, low-latency sensor streaming

Prerequisites
-------------

For the C++ bridge, BeamNG.tech and ROS 2 must run on the **same machine** and
the **same operating system**. Windows Subsystem for Linux (WSL) is strongly
discouraged because of significant performance limitations.

Installation (pre-built binaries)
---------------------------------

Pre-built binaries are provided so you do not have to compile the integration
yourself:

1. Download the latest build from the
   `releases <https://github.com/BeamNG/beamng-ros2-integration/releases>`_ page.
2. Extract the archive to a directory of your choice.
3. Copy the plugin into your BeamNG.tech installation:

   - **Windows**: copy the ``.dll`` file into the ``Bin64`` directory.
   - **Linux**: copy the ``.so`` file into the ``BinLinux`` directory.

Running BeamNG.tech with the C++ bridge
---------------------------------------

To use BeamNG.tech with the C++ ROS 2 integration you need to source both your
ROS 2 environment and the bundled ROS 2 extension, then start the simulator:

1. Source your ROS 2 distribution as described in its documentation.
2. Source the bundled extension by running ``install/local_setup.*``:

   - **Windows**: ``install\local_setup.ps1`` or ``install\local_setup.bat``
   - **Linux**: ``install/local_setup.sh``, ``install/local_setup.bash``,
     ``install/local_setup.zsh`` or ``install/local_setup.ps1``

3. Start BeamNG.tech with the sandbox disabled:

   .. code-block:: bash

      Bin64\BeamNG.drive.x64.exe -disable-sandbox

.. warning::

   Without the ``-disable-sandbox`` flag, the C++ ROS 2 integration will not
   work correctly.

Enabling the ROS2 Editor
------------------------

Once BeamNG.tech is running with ROS 2, the integration is controlled from the
World Editor:

1. Load a map and a vehicle.
2. Open the World Editor (default key: ``F11``).
3. Open the *Set Editor*:

   .. image:: ../../../media/World_Editor_Menu.png
      :alt: World Editor menu with the Set Editor
      :width: 100%

4. Go to *Edit Sets*.
5. Add the **ROS2 Editor** to the selected set:

   .. image:: ../../../media/ROS2_Set_Selected.png
      :alt: ROS2 Editor added to the selected set
      :width: 100%

Using the ROS2 Editor
---------------------

With the editor enabled, you can open it from the toolbar using its icon:

.. image:: ../../../media/ROS2_Editor_Icon.png
   :alt: ROS2 Editor toolbar icon

The ROS2 Editor window is organized into five sections:

1. **Library selection (top).** Choose which ``.dll`` / ``.so`` file to use.
   This is useful when you want to run different ROS 2 distributions with
   BeamNG.tech on the same machine.

   .. image:: ../../../media/ROS2_Editor_Library.png
      :alt: ROS2 Editor showing the selected ROS 2 library path at the top
      :width: 100%

   .. important::

      Different ROS 2 distributions are usually **not** inter-compatible; see
      `Compatibility`_ for the supported combinations.

2. **Vehicle list.** Lists every vehicle in the simulation; each node name is
   the vehicle's simulation id. Per-vehicle buttons let you:

   - center the camera on the vehicle,
   - load a configuration file for the vehicle,
   - save the vehicle's current configuration,
   - start (or restart) the ROS 2 node for the vehicle,
   - stop the ROS 2 node for the vehicle.

   A vehicle line is highlighted in **green** while its node is running.

3. **Available sensors.** A grid of sensors that can be attached to the
   selected vehicle. Click a sensor to add it.

   .. note::

      Changes to the sensor configuration are only applied after the vehicle
      node is restarted.

4. **Attached sensors.** The list of sensors currently attached to the
   vehicle. Select one to edit its parameters in the section below.

5. **Sensor parameters (bottom).** Edit the parameters of the selected sensor.
   For sensors that have a physical position or rotation on the vehicle, you can
   also use the World Editor gizmos to adjust them.

   .. note::

      Changes to sensor parameters are only applied after the vehicle node is
      restarted.

Compatibility
-------------

BeamNG.tech and ROS 2 must run on the **same machine** and the **same
operating system**. The C++ integration is compatible with the following
ROS 2 distributions and operating systems:

.. list-table::
   :header-rows: 1

   * - ROS 2 Distribution
     - Windows 10 / 11
     - Ubuntu 22.04
     - Ubuntu 24.04
     - Ubuntu 26.04
   * - Humble Hawksbill
     - Yes
     - Yes
     - No
     - No
   * - Jazzy Jalisco
     - Yes
     - Not officially supported
     - Yes
     - No
   * - Lyrical Luth
     - Yes
     - No
     - Not officially supported
     - Yes

.. important::

   We strongly discourage the use of Windows Subsystem for Linux (WSL) for the
   BeamNG ROS 2 integration due to significant performance limitations. Other
   operating systems may work, but are not officially supported.

Compiling from source
----------------------

Although pre-built binaries are provided, you can also compile the C++
integration yourself.

Windows
~~~~~~~

**Prerequisites**

- *Visual Studio 2019 x64 Native Tools Command Prompt for VS 2019* with CMake
  and Python installed.
- A ROS 2 environment (see `Compatibility`_).

**Steps**

1. Clone the repository and check out the ``cpp`` branch:

   .. code-block:: bash

      git clone https://github.com/BeamNG/beamng-ros2-integration.git
      cd beamng-ros2-integration
      git checkout cpp

2. Delete the ``build``, ``install`` and ``log`` folders if they exist.
3. Start a *Visual Studio 2019 x64 Native Tools Command Prompt for VS 2019*.
4. Source your ROS 2 environment.
5. From the repository root, build the packages:

   .. code-block:: bash

      colcon build --merge-install --packages-select beamng_msgs beamng_ros2
      cd build/beamng_ros2
      cmake --build . --config Release

6. Copy the ``.dll`` file from ``build\beamng_ros2\Release`` into the ``Bin64``
   directory of your BeamNG.tech installation.

Linux
~~~~~

**Prerequisites**

Install the required build tools (Ubuntu one-liner):

.. code-block:: bash

   apt-get install -y --no-install-recommends locales build-essential cmake git python3-colcon-common-extensions

**Steps**

1. Clone the repository and check out the ``cpp`` branch:

   .. code-block:: bash

      git clone https://github.com/BeamNG/beamng-ros2-integration.git
      cd beamng-ros2-integration
      git checkout cpp

2. Delete the ``build``, ``install`` and ``log`` folders if they exist.
3. From the repository root, build the packages:

   .. code-block:: bash

      colcon build --merge-install --packages-select beamng_msgs beamng_ros2
      cd build/beamng_ros2
      cmake --build . --config Release

4. Copy the ``.so`` file from ``build/beamng_ros2/Release`` into the
   ``BinLinux`` directory of your BeamNG.tech installation.

Known issues
------------

- The ultrasonic sensor currently suffers from a crash caused by a Vulkan
  issue.
