Drone Testing
=============

Full setup and testing commands for the BeamNG ROS 2 drone integration. The
:mod:`drone_controller` node converts ``/cmd_vel`` and ``/cmd_vel_stamped`` into
``/drone_control`` for the BeamNG drone.

Throughout this page, ``<ROS2_WORKSPACE>`` refers to your ROS 2 colcon workspace
(for example ``~/ros2_ws``). Use ``<BEAMNG_HOST_IP>`` and ``<BEAMNG_PORT>`` for your
BeamNG.tech instance (for WSL2, the host IP is typically the ``vEthernet (WSL)``
address from ``ipconfig`` on Windows).

Prerequisites
-------------

- BeamNG.tech installed and running with ``-tcom-listen-ip 0.0.0.0`` or your host IP.
- ROS 2 workspace built and sourced: ``source install/setup.bash``


All-in-one launch
-----------------

The :file:`beamng_drone_all.launch.py` launch file starts the bridge, scenario,
drone controller and cmd_vel converter in one command. You then choose one of two
control inputs (run in a separate terminal):

- **Keyboard teleop** – for manual testing
- **MAVROS or any cmd_vel publisher** – for UAV stacks (MAVLink, PX4, etc.)

**Launch (set host for your setup):**

.. code-block:: bash

   cd <ROS2_WORKSPACE>
   source install/setup.bash
   ros2 launch beamng_bringup beamng_drone_all.launch.py \
     host:=<BEAMNG_HOST_IP> port:=<BEAMNG_PORT> \
     scenario:=/config/scenarios/drone_scenario.json vehicle_id:=drone

Use the host IP that matches your setup: same-machine=127.0.0.1, WSL=vEthernet (WSL) from Windows ipconfig, remote=BeamNG machine IP.

**Override individual args:**

.. code-block:: bash

   ros2 launch beamng_bringup beamng_drone_all.launch.py host:=<YOUR_IP> port:=25252

If the drone controller times out (Connection timed out), the scenario may need
more time to load. Increase the delay:

.. code-block:: bash

   ros2 launch beamng_bringup beamng_drone_all.launch.py controller_delay:=25.0

**Control input** (choose one; run in a separate terminal):

- **Keyboard teleop** (for manual testing):

  .. code-block:: bash

     cd <ROS2_WORKSPACE>
     source install/setup.bash
     ros2 run beamng_teleop_keyboard drone_teleop

- **MAVROS or other cmd_vel source** (for MAVLink/UAV stack integration):

  The launch already includes the cmd_vel converter. Instead of keyboard teleop,
  run your MAVROS (or any node that publishes ``geometry_msgs/Twist`` to
  ``/cmd_vel``). The converter maps:

  - ``linear.x`` / ``linear.y`` → roll & pitch
  - ``linear.z`` → thrust (ascend/descend)
  - ``angular.z`` → yaw rate

  The drone will receive commands via ``/drone_control`` and fly normally.
  No need to run ``drone_teleop`` when using MAVROS.


Setup (alternative: run in order)
---------------------------------

**Terminal 1 – Bridge:**

.. code-block:: bash

   cd <ROS2_WORKSPACE>
   source install/setup.bash
   ros2 run beamng_ros2 beamng_bridge \
     --ros-args -p host:=<BEAMNG_HOST_IP> -p port:=<BEAMNG_PORT>

**Terminal 2 – Drone scenario:**

.. code-block:: bash

   cd <ROS2_WORKSPACE>
   source install/setup.bash
   ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario \
     "{path_to_scenario_definition: '/config/scenarios/drone_scenario.json'}"

**Terminal 3 – Drone controller (BeamNG agent):**

.. code-block:: bash

   cd <ROS2_WORKSPACE>
   source install/setup.bash
   ros2 run beamng_agent drone_controller \
     --ros-args -p host:=<BEAMNG_HOST_IP> -p port:=<BEAMNG_PORT> -p vehicle_id:=drone

**Terminal 4 – Keyboard teleop** (interactive):

.. code-block:: bash

   cd <ROS2_WORKSPACE>
   source install/setup.bash
   ros2 run beamng_teleop_keyboard drone_teleop

**Optional – cmd_vel converter** (this package):

.. code-block:: bash

   ros2 launch beamng_bringup beamng_drone_teleop


Verification and testing
------------------------

**Check nodes:**

.. code-block:: bash

   ros2 node list
   ros2 node info /beamng_bridge
   ros2 node info /beamng_drone_controller
   ros2 node info /cmd_vel_to_drone_control

**Check topics:**

.. code-block:: bash

   ros2 topic list
   ros2 topic list | grep -E 'drone|cmd_vel|control'

**Echo control traffic:**

.. code-block:: bash

   ros2 topic echo /drone_control
   ros2 topic echo /cmd_vel

**Check services:**

.. code-block:: bash

   ros2 service list | grep beamng

**Check parameters:**

.. code-block:: bash

   ros2 param list /beamng_bridge
   ros2 param list /beamng_drone_controller
   ros2 param list /cmd_vel_to_drone_control

**Manual publish** (no teleop):

.. code-block:: bash

   ros2 topic pub /drone_control beamng_msgs/msg/DroneControl \
     "{ascend: 0.5, descend: 0.0, pitch: 0.0, roll: 0.0, yaw: 0.0, \
       yaw_left: 0.0, yaw_right: 0.0, takeoff_land: false, toggle_mode: false}" --once


Teleop display
--------------

When you run ``ros2 run beamng_teleop_keyboard drone_teleop``, it prints the setup
commands and key bindings at startup. A live status line shows:

- **Key**: last key pressed (e.g. ``w``, ``a``, ``t``, ``m``, ``Space``)
- **Reading**: current control values being sent (Asc, Desc, Pit, Rol, Yaw)
- **Mode**: flight mode (``Crawl`` or ``Fixed Wing``)

Example status line::

   Key:     w  |  Reading: Asc:+0.50 Desc:+0.00 Pit:+0.00 Rol:+0.00 Yaw:+0.00  |  Mode:Crawl


Drone teleop keys
-----------------

- ``t``: toggle takeoff/land
- ``w`` / ``x``: ascend / descend
- ``a`` / ``d``: roll left / right
- ``q`` / ``e``: yaw left / right
- ``i`` / ``k``: pitch forward / backward
- ``m``: toggle flight mode (Crawl ↔ Fixed Wing); status line shows current mode
- ``space`` or ``s``: hover / stop
