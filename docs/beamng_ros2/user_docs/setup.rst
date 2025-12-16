Setup
=====

General setup
-------------

Before you use teleoperation, ensure:

- BeamNG.tech is installed and running on a host machine.
- Your ROS 2 workspace (referred to as ``<ROS2_WORKSPACE>``, for example ``~/ros2_ws``) is built with this repository.
- You know the IP and port of your BeamNG.tech instance (referred to as ``<BEAMNG_HOST_IP>`` and ``<BEAMNG_PORT>``).

To start the bridge node, run:

.. code-block:: bash

   ros2 run beamng_ros2 beamng_bridge --ros-args -p host:=192.168.1.165 -p port:=25252

Scenario files
~~~~~~~~~~~~~~

Scenarios live in :file:`beamng_ros2/config/scenarios/` and are defined as JSON files.
They specify:

- The map/level to load.
- One or more vehicles (with ids like ``ego``, ``car1``, etc.).
- Optional sensors and AI modes.

To start a scenario from ROS 2, call the :code:`/beamng_bridge/start_scenario` service
and pass the path to your scenario definition, for example:

.. code-block:: bash

   ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario \
     "{path_to_scenario_definition: '/config/scenarios/example_tech_ground.json'}"


Vehicle teleoperation
---------------------

This section shows how to teleoperate a ground vehicle using the provided keyboard teleop node
and the :mod:`beamng_agent` node. As an example, we use the :file:`example_italy.json` scenario,
which spawns an ``ETK800`` as the ``ego`` vehicle.

You typically use four terminals:

1. **BeamNG bridge**: connects ROS 2 to BeamNG.tech.
2. **Scenario loader**: loads the Italy scenario with an ETK800.
3. **Agent**: connects to the vehicle and converts :code:`/cmd_vel` into BeamNG controls.
4. **Keyboard teleop**: reads keyboard input and publishes :code:`geometry_msgs/Twist`.

Bridge
~~~~~~

In the first terminal, start the bridge node as described in the *General setup* section.

Scenario (ETK800 on Italy)
~~~~~~~~~~~~~~~~~~~~~~~~~~

In a second terminal, load the Italy scenario:

.. code-block:: bash

   cd <ROS2_WORKSPACE>
   source install/setup.bash
   ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario \
     "{path_to_scenario_definition: '/config/scenarios/example_italy.json'}"

This scenario spawns multiple vehicles on the Italy map, including an ``ETK800`` named ``ego``.

Agent for ``ego``
~~~~~~~~~~~~~~~~~

In a third terminal, start the agent node that controls the ``ego`` vehicle:

.. code-block:: bash

   cd <ROS2_WORKSPACE>
   source install/setup.bash
   ros2 run beamng_agent beamng_agent \
     --ros-args \
       -p host:=<BEAMNG_HOST_IP> \
       -p port:=<BEAMNG_PORT> \
       -p driving_mode:=keyboard \
       -p vehicle_id:=ego \
       -p vehicle_type:=sedan

Keyboard teleop (ground vehicle)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In a fourth **interactive** terminal, start the keyboard teleop for ground vehicles:

.. code-block:: bash

   cd <ROS2_WORKSPACE>
   source install/setup.bash
   ros2 run beamng_teleop_keyboard teleop

With this node running, you can control the ETK800 using:

- ``w`` / ``x``: increase / decrease linear speed (forward / reverse).
- ``a`` / ``d``: steer left / right.
- ``space`` or ``s``: brake / stop.


Drone teleoperation
-------------------

The BeamNG ROS2 integration also supports controlling the **BeamNG.tech drone vehicle** via ROS 2
using the provided bridge, controller and keyboard teleoperation nodes.

The overall setup (BeamNG.tech, workspace, IP/port) is the same as in the general setup above.
You again use four terminals:

1. **BeamNG bridge**: connects ROS 2 to BeamNG.tech.
2. **Scenario loader**: starts the drone scenario.
3. **Drone controller**: converts ROS commands into BeamNG drone control.
4. **Keyboard teleop**: reads keyboard input and publishes velocity commands.

Bridge
~~~~~~

In the first terminal, start the bridge node as described in the *General setup* section.

Scenario (drone)
~~~~~~~~~~~~~~~~

In a second terminal, load the drone scenario:

.. code-block:: bash

   cd <ROS2_WORKSPACE>
   source install/setup.bash
   ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario \
     "{path_to_scenario_definition: '/config/scenarios/drone_scenario.json'}"

Drone controller
~~~~~~~~~~~~~~~~

In a third terminal, start the drone controller:

.. code-block:: bash

   cd <ROS2_WORKSPACE>
   source install/setup.bash
   ros2 launch beamng_bringup beamng_drone_controller.launch.py \
     host:=<BEAMNG_HOST_IP> port:=<BEAMNG_PORT> vehicle_id:=drone

Keyboard teleop (drone)
~~~~~~~~~~~~~~~~~~~~~~~

In a fourth **interactive** terminal (do not use a launch file), start the keyboard teleop:

.. code-block:: bash

   cd <ROS2_WORKSPACE>
   source install/setup.bash
   ros2 run beamng_teleop_keyboard drone_teleop

Once running, focus the teleop terminal and use:

- ``t``: toggle takeoff/land (press once to take off, again to land).
- ``w`` / ``x``: ascend / descend.
- ``i`` / ``k``: pitch forward / backward.
- ``a`` / ``d``: roll left / right.
- ``q`` / ``e``: yaw left / right.
- ``space`` or ``s``: hover / stop.

