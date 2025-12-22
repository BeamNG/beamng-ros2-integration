Feature Overview and Package Tour
=================================

This page gives a quick **feature-oriented overview** of what you can do with the
BeamNG ROS 2 integration, followed by a short **package tour** for developers who
want to know where functionality lives in the repository.

Throughout this page, ``<ROS2_WORKSPACE>`` refers to the root of your ROS 2
colcon workspace (for example ``~/ros2_ws``).

Feature overview
----------------

The integration is built around a few core use cases:

- **Bridge and scenarios**

  - Connect ROS 2 to a running BeamNG.tech instance using the `beamng_ros2` bridge.
  - Load simulation scenarios (maps, vehicles, sensors) from JSON files via ROS 2 services.
  - Example: loading `example_tech_ground.json` or `example_italy.json` from
    :file:`beamng_ros2/config/scenarios/`.

- **Vehicle teleoperation (ground vehicles)**

  - Drive a car (for example an ETK800 on the Italy map) using a keyboard-driven
    ROS 2 node that publishes `geometry_msgs/Twist`.
  - The `beamng_agent` node converts `/cmd_vel` and brake commands into
    `beamng_msgs/VehicleControl` for the selected vehicle.
  - See :file:`setup.rst` for a complete terminal-by-terminal walkthrough.

- **Drone teleoperation**

  - Control the BeamNG.tech drone vehicle using a dedicated drone controller
    and keyboard teleop node.
  - Uses the same bridge and scenario loading flow as ground vehicles, but with
    a drone-specific controller and teleop mapping.
  - See :file:`setup.rst` for the full sequence of bridge, scenario, controller
    and teleop commands.

- **Sensors and data streaming**

  - Attach sensors (camera, lidar, radar, IMU, etc.) to vehicles via scenario
    configuration and receive sensor data on ROS 2 topics.
  - Sensor definitions are in :file:`beamng_ros2/config/sensors.json` and the
    scenario JSON files.
  - The `beamng_ros2` package contains the publishers that expose these
    sensors into ROS 2 messages under `beamng_ros2/publishers/`.

- **Cosimulation**

  - Run co-simulation experiments where BeamNG.tech is coupled with an
    external controller or environment via ROS 2.
  - Configuration for cosimulation is stored under
    :file:`beamng_ros2/config/cosim/`.

For detailed step-by-step setup and teleoperation instructions, start with
:file:`setup.rst` in this `user_docs` folder.


Node execution per feature
--------------------------

This section summarizes the **main nodes/commands to run** for each feature.
Use :file:`setup.rst` for more detailed, terminal-by-terminal tutorials.

Bridge and scenarios
~~~~~~~~~~~~~~~~~~~~

- **Bridge node** (terminal 1):

  .. code-block:: bash

     cd <ROS2_WORKSPACE>
     source install/setup.bash
     ros2 run beamng_ros2 beamng_bridge \
       --ros-args -p host:=<BEAMNG_HOST_IP> -p port:=<BEAMNG_PORT>

- **Scenario loading** (terminal 2, example scenario):

  .. code-block:: bash

     cd <ROS2_WORKSPACE>
     source install/setup.bash
     ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario \
       "{path_to_scenario_definition: '/config/scenarios/example_tech_ground.json'}"


Vehicle teleoperation (ETK800 example)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Minimal set of nodes/commands:

1. **Bridge** (terminal 1) – as in *Bridge and scenarios* above.
2. **Italy scenario with ETK800** (terminal 2):

   .. code-block:: bash

      cd <ROS2_WORKSPACE>
      source install/setup.bash
      ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario \
        "{path_to_scenario_definition: '/config/scenarios/example_italy.json'}"

3. **Agent for ``ego`` vehicle** (terminal 3):

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

4. **Keyboard teleop for ground vehicle** (terminal 4, interactive):

   .. code-block:: bash

      cd <ROS2_WORKSPACE>
      source install/setup.bash
      ros2 run beamng_teleop_keyboard teleop


Drone teleoperation
~~~~~~~~~~~~~~~~~~~

Minimal set of nodes/commands:

1. **Bridge** (terminal 1) – as in *Bridge and scenarios* above.
2. **Drone scenario** (terminal 2):

   .. code-block:: bash

      cd <ROS2_WORKSPACE>
      source install/setup.bash
      ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario \
        "{path_to_scenario_definition: '/config/scenarios/drone_scenario.json'}"

3. **Drone controller** (terminal 3):

   .. code-block:: bash

      cd <ROS2_WORKSPACE>
      source install/setup.bash
      ros2 launch beamng_bringup beamng_drone_controller.launch.py \
        host:=<BEAMNG_HOST_IP> port:=<BEAMNG_PORT> vehicle_id:=drone

4. **Keyboard teleop for drone** (terminal 4, interactive):

   .. code-block:: bash

      cd <ROS2_WORKSPACE>
      source install/setup.bash
      ros2 run beamng_teleop_keyboard drone_teleop


Sensors and data streaming
~~~~~~~~~~~~~~~~~~~~~~~~~~

Sensors are configured in the scenario JSON files (and :file:`sensors.json`) and
are managed by the bridge. Typically, you only need:

1. **Bridge** (terminal 1) – as in *Bridge and scenarios* above.
2. **Scenario with sensors attached** (terminal 2) – via
   :code:`/beamng_bridge/start_scenario` as shown earlier.

Once the scenario is running, the relevant sensor topics are published
automatically (see the *How to test the main features* section below for
example commands).


Cosimulation
~~~~~~~~~~~~

Cosimulation is configured via JSON files under :file:`beamng_ros2/config/cosim/`.
The main ROS 2 commands are:

1. **Bridge** (terminal 1) – as in *Bridge and scenarios* above.
2. **Start cosimulation** (terminal 2):

   .. code-block:: bash

      cd <ROS2_WORKSPACE>
      source install/setup.bash
      ros2 service call /beamng_bridge/start_cosimulation beamng_msgs/srv/StartCosimulation \
        "{path_to_cosim_definition: '/config/cosim/default.json'}"

3. **Stop cosimulation** when done:

   .. code-block:: bash

      ros2 service call /beamng_bridge/stop_cosimulation beamng_msgs/srv/StopCosimulation "{}"


How to test the main features
-----------------------------

This section shows **minimal commands** to verify that each feature works.
It is meant as a quick sanity check rather than a full tutorial.

- **Bridge and scenarios**

  - Check that the bridge node is running and connected:

    .. code-block:: bash

       ros2 node list
       ros2 node info /beamng_bridge

  - After calling :code:`/beamng_bridge/start_scenario`, verify that the service
    completed successfully and that vehicles/sensors appear in BeamNG.tech.

- **Vehicle teleoperation**

  - With the bridge, scenario, agent and `teleop` node running (see
    :file:`setup.rst`), confirm that `/cmd_vel` and `/control` are active:

    .. code-block:: bash

       ros2 topic list | grep cmd_vel
       ros2 topic list | grep control

  - While pressing keys in the teleop terminal, you should see messages:

    .. code-block:: bash

       ros2 topic echo /cmd_vel
       ros2 topic echo /control

  - In BeamNG.tech, the selected vehicle (for example the ETK800 `ego`) should
    respond to your keyboard input.

- **Drone teleoperation**

  - With the bridge, drone scenario, drone controller and `drone_teleop` node
    running, verify the topics:

    .. code-block:: bash

       ros2 topic list | grep drone

  - While flying the drone with the keyboard, you should see command activity
    on the relevant control topics (for example `/drone_control` or
    `/cmd_vel`) and observe motion in BeamNG.tech.

- **Sensors and data streaming**

  - After loading a scenario with sensors, list topics and identify sensor
    outputs (for example camera, lidar, radar):

    .. code-block:: bash

       ros2 topic list

  - Echo a sensor topic to verify that data is being published:

    .. code-block:: bash

       ros2 topic echo <sensor_topic_name>

  - For image-type topics, you can also use tools like `rqt_image_view` to
    visualize camera output.


Package tour (for developers)
-----------------------------

If you need to modify or extend the integration, this high-level package tour
shows where the main functionality is implemented:

- **beamng_ros2**

  - Core bridge between ROS 2 and BeamNG.tech (connection management,
    scenario loading, sensor publishers, vehicle handling).
  - Key directories:
    - :file:`beamng_ros2/beamng_ros2/beamng.py` – main bridge logic.
    - :file:`beamng_ros2/beamng_ros2/publishers/` – sensor and state publishers.
    - :file:`beamng_ros2/beamng_ros2/scenario_loader.py` – scenario loading
      utilities.

- **beamng_msgs**

  - ROS 2 message, service and action definitions used by the integration.
  - Messages live in :file:`beamng_msgs/msg/`, services in
    :file:`beamng_msgs/srv/`, and actions in :file:`beamng_msgs/action/`.
  - The Sphinx reference docs for these are under :file:`docs/beamng_msgs/`.

- **beamng_agent**

  - Implements the `beamng_agent` node that translates high-level control
    commands (such as `/cmd_vel` or `/control`) into BeamNG vehicle input.
  - Supports both ground vehicles and drones.
  - Main logic is in :file:`beamng_agent/beamng_agent/beamng_agent.py`.

- **beamng_bringup**

  - Contains launch files that compose bridge, agents, controllers and teleop
    nodes into ready-to-run configurations.
  - Launch files are under :file:`beamng_bringup/launch/` (e.g.
    `beamng_drone_controller.launch.py`, `beamng_agent_ego.launch.py`).

- **beamng_teleop_keyboard**

  - Provides keyboard teleoperation nodes for ground vehicles and the drone.
  - Source is in :file:`beamng_teleop_keyboard/beamng_teleop_keyboard/` with
    `teleop.py` (ground vehicles) and `drone_teleop.py` (drone).

- **drone_controller**

  - Contains a dedicated ROS 2 node and PID controller implementation for the
    BeamNG.tech drone.
  - Main files:
    - :file:`drone_controller/drone_controller/drone_controller.py`
    - :file:`drone_controller/drone_controller/pid_controller.py`

This overview is intended as a **side note for developers**: end users can
follow the feature-oriented guides in :file:`setup.rst` and other
`user_docs`, while contributors can use this section to quickly find where
specific behavior is implemented in the codebase.
