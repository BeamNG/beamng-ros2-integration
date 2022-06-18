BeamNG Control
==============

This guide describes the feature set of the **beamng_control** package.
It connects ROS to the BeamNG.tech simulation platform and converts sensor information into ROS and,
where appropriate, Rviz compatible ROS messages.

Starting the Node
-----------------

To start the node a running instance of BeamNG.tech is needed
that has been started through the BeamNGpy library.
A list of compatible versions can be found on the home page or in the README.md.

Defining Scenarios
------------------

The BeamNG ROS Integration supports JSON-based scenario definition.
Example files can be found in the ``beamng_control/config/scenarios/`` directory.

General Setup
^^^^^^^^^^^^^

Scenarios are defined through JSON objects, here is a list of possible keys and values.

+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|Key                   |Value Type        | Value Specification                                                                 | Entry Type |
+======================+==================+=====================================================================================+============+
|``"version"``         |String            | BeamnG ROS Integration version, f.ex. ``1``                                         | Mandatory  |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"level"``           |String            | BeamNG.tech level name, f.ex. ``"west_coast_usa"``.                                 | Mandatory  |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"mode"``            |String            | Value                                                                               | Optional   |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"vehicles"``        |Array             | At least one vehicle needs to be specified in order to obtain a valid scenario.     | Mandatory  |
|                      |                  | See the table below for the Specification.                                          |            |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"name"``            |String            | Name of the level.                                                                  | Mandatory  |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"time_of_day"``     |Float             | Value between ``0`` and ``1`` where the range ``[0, .5]`` corresponds               | Optional   |
|                      |                  | to the times between 12 a.m. and 12 p.m. and ``[.5], 1]`` corresponds to            |            |
|                      |                  | the time range between 12 p.m. and 12 a.m.                                          |            |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"weather_presets"`` |String            | Weather presets are level specific, **ToDo**                                        | Optional   |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+

Vehicle Definition
^^^^^^^^^^^^^^^^^^

Vehicles are also defined as JSON objects.

+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|Key                   |Value Type        | Value Specification                                                                 | Entry Type |
+======================+==================+=====================================================================================+============+
|``"name"``            |String            |Name of the vehicle, used for identification                                         | Mandatory  |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"model"``           |String            |Name of the vehicle type, f.ex. ``etk800``                                           | Mandatory  |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"position"``        |Array             |Array of 3 floats, specifying the ``x``, ``y``, and ``x`` position of the vehicle.   | Mandatory  |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"rotation"``        |Array             |Array of 4 floats, specifying the vehicle rotation quaternion.                       | Mandatory  |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"sensors"``         |Array             |Array of JSON objects, specifying the vehicles sensor parameters.                    | Optional   |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+

Sensor Definition
^^^^^^^^^^^^^^^^^

Sensors are also defined with the help of JSON objects.
To avoid redundancy, the sensor definition of complex sensors has been split into two parts, where the default values are stored in an extra JSON file.
Default values can be overridden by adding them to the JSON object with a new value.
All sensors automatically publish their data to the topic ``/beamng_control/<vehicle name>/<unique_sensor_id>``.


Damage Sensor
"""""""""""""

Message type: ``beamng_msgs.msg.DamagSensor``

+----------------------+------------------+------------------------------------+------------+
|Key                   | Value Type       | Value Specification                | Entry Type |
+======================+==================+====================================+============+
|``"type"``            | String           | ``"Damage"``                       | Mandatory  |
+----------------------+------------------+------------------------------------+------------+
|``"name"``            | String           | Unique sensor id.                  | Mandatory  |
+----------------------+------------------+------------------------------------+------------+

Timer
"""""

Message type: ``beamng_msgs.msg.TimeSensor``

+----------------------+------------------+------------------------------------+------------+
|Key                   | Value Type       | Value Specification                | Entry Type |
+======================+==================+====================================+============+
|``"type"``            | String           | ``"Timer"``                        | Mandatory  |
+----------------------+------------------+------------------------------------+------------+
|``"name"``            | String           | Unique sensor id.                  | Mandatory  |
+----------------------+------------------+------------------------------------+------------+


GForces
"""""""

Message type: ``beamng_msgs.msg.GForceSensor``

+----------------------+------------------+------------------------------------+------------+
|Key                   | Value Type       | Value Specification                | Entry Type |
+======================+==================+====================================+============+
|``"type"``            | String           | ``"GForces"``                      | Mandatory  |
+----------------------+------------------+------------------------------------+------------+
|``"name"``            | String           | Unique sensor id.                  | Mandatory  |
+----------------------+------------------+------------------------------------+------------+

Electrics
"""""""""

Message type: ``beamng_msgs.msg.ElectricsSensor``

+----------------------+------------------+------------------------------------+------------+
|Key                   | Value Type       | Value Specification                | Entry Type |
+======================+==================+====================================+============+
|``"type"``            | String           | ``"Electrics"``                    | Mandatory  |
+----------------------+------------------+------------------------------------+------------+
|``"name"``            | String           | Unique sensor id.                  | Mandatory  |
+----------------------+------------------+------------------------------------+------------+

Camera
""""""

Contrary to other sensors, the Camera sensor may publish to multiple topics.
If the camera sensor is configured to collect color, depth, annotation, and instance data, it is published to the respective topics:

    * ``/beamng_control/<vehicle_id>/<camera_id>/color``
    * ``/beamng_control/<vehicle_id>/<camera_id>/depth``
    * ``/beamng_control/<vehicle_id>/<camera_id>/annotation``
    * ``/beamng_control/<vehicle_id>/<camera_id>/instance``

The message type for all topics is ``sensor_msgs.msg.Image``.
Note that although the bounding_box option is given, this feature is still under development and will automatically be disabled.

+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|Key                   | Value Type       | Value Specification                                                                 | Entry Type |
+======================+==================+=====================================================================================+============+
|``"type"``            | String           | ``"Camera.default"``                                                                | Mandatory  |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"name"``            | String           | Unique sensor id.                                                                   | Mandatory  |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"position"``        | Array            | Array of 3 floats, specifying the ``x``, ``y``, and ``x`` position of the sensor.   | Mandatory  |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"orientation"``     | Array            | Array of 4 floats, specifying the vehicle rotation quaternion                       | Mandatory  |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"resolution"``      | Array            | Tuple of ints, defining the ``x`` and ``y`` resolution of                           | Optional   |
|                      |                  | the resulting images.                                                               |            |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"fov"``             | Integer          | Camera field of view.                                                               | Optional   |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"colour"``          | Boolean          | Dis-/Enables color image generation.                                                | Optional   |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"depth"``           | Boolean          | Dis-/Enables depth image generation.                                                | Optional   |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"annotation"``      | Boolean          | Dis-/Enables ground truth generation for object type annotation.                    | Optional   |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"instance"``        | Boolean          | Dis-/Enables ground truth generation for instance annotation.                       | Optional   |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``"bounding_box"``    | Boolean          | This feature is not supoprted at the moment                                         | Optional   |
|                      |                  | and will be **automatically disabled**.                                             |            |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+

Lidar
"""""

Message type: ``sensor_msgs.msg.PointCloud2``

+-------------------------------------+------------------+------------------------------------------------------------------------+------------+
| Key                                 | Value Type       | Value Specification                                                    | Entry Type |
+=====================================+==================+========================================================================+============+
|``"type"``                           | String           | ``"Lidar.default"``                                                    | Mandatory  |
+-------------------------------------+------------------+------------------------------------------------------------------------+------------+
|``"name"``                           | String           | Unique sensor id.                                                      | Mandatory  |
+-------------------------------------+------------------+------------------------------------------------------------------------+------------+
|``"vertical_resolution"``            | Integer          | Vertical sensor resolution.                                            | Mandatory  |
+-------------------------------------+------------------+------------------------------------------------------------------------+------------+
|``"vertical_angle"``                 | Float            | Vertical angle.                                                        | Mandatory  |
+-------------------------------------+------------------+------------------------------------------------------------------------+------------+
|``"hz"``                             | Integer          | Laser frequency.                                                       | Mandatory  |
+-------------------------------------+------------------+------------------------------------------------------------------------+------------+
|``"angle"``                          | Integer          |                                                                        | Mandatory  |
+-------------------------------------+------------------+------------------------------------------------------------------------+------------+
|``"max_distance"``                   | Integer          | Maximal distance for data collection.                                  | Mandatory  |
+-------------------------------------+------------------+------------------------------------------------------------------------+------------+
|``"visualized"``                     | Integer          | Dis-/Enable in-simulation Lidar visualization.                         | Mandatory  |
+-------------------------------------+------------------+------------------------------------------------------------------------+------------+


Controlling the Simulation
--------------------------

Various services  to control the state of the simulation are available.

+-----------------------------------------------+-----------------------------------------------+------------------------------------------------------------------------------------------+
|Name                                           | Type                                          | Purpose                                                                                  |
+===============================================+===============================================+==========================================================================================+
|``/beamng_control/get_scenario_state``         | ``bng_msgs.msg.GetScenarioState.srv``         | Determining the current state of the scenario.                                           |
+-----------------------------------------------+-----------------------------------------------+------------------------------------------------------------------------------------------+
|``/beamng_control/start_scenario``             | ``bng_msgs.msg.StartScenario.srv``            | Starting a loaded scenario.                                                              |
+-----------------------------------------------+-----------------------------------------------+------------------------------------------------------------------------------------------+
|``/beamng_control/pause``                      | ``bng_msgs.msg.ChangeSmulationState.srv``     | Pause the simulation.                                                                    |
+-----------------------------------------------+-----------------------------------------------+------------------------------------------------------------------------------------------+
|``/beamng_control/resume``                     |``bng_msgs.msg.ChangeSmulationState.srv``      | Resume the simulation.                                                                   |
+-----------------------------------------------+-----------------------------------------------+------------------------------------------------------------------------------------------+

While the simulation is paused, it can be advanced a given amount of steps through the action server:

+-----------------------------------------------+-----------------------------------------------+------------------------------------------------------------------------------------------+
|Name                                           | Type                                          | Purpose                                                                                  |
+===============================================+===============================================+==========================================================================================+
|``/beamng_control/step``                       | ``bng_msgs.msg.Step``                         | Determining the current state of the scenario.                                           |
+-----------------------------------------------+-----------------------------------------------+------------------------------------------------------------------------------------------+


Vehicle Creation and Control
----------------------------

+-----------------------------------------------+-----------------------------------------------+------------------------------------------------------------------------------------------+
|Name                                           | Type                                          | Purpose                                                                                  |
+===============================================+===============================================+==========================================================================================+
|``/beamng_control/get_current_vehicles``       |``beamng_msgs.srv.GetCurrentVehiclesInfo``     | Get list of spawned vehicles.                                                            |
+-----------------------------------------------+-----------------------------------------------+------------------------------------------------------------------------------------------+
|``/beamng_control/spawn_vehicle``              |``beamng_msgs.srv.SpawnVehicle``               | Spawn new vehicle.                                                                       |
+-----------------------------------------------+-----------------------------------------------+------------------------------------------------------------------------------------------+
|``/beamng_control/teleport_vehicle``           |``beamng_msgs.srv.TeleportVehicle``            | Teleport vehicle.                                                                        |
+-----------------------------------------------+-----------------------------------------------+------------------------------------------------------------------------------------------+





