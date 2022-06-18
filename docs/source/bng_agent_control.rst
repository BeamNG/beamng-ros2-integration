BeamNG Agent Control
====================

This guide documents the feature set of the **beamng_agent** package.
Its main purpose is to provide an interface to steer individual vehicles.

Starting the Node
-----------------

For starting the node a running instance of BeamNG.tech is needed
that has been started through the BeamNGpy library.
Then a ``beamng_control`` node needs to be launched to start a scenario of your choice.
After adjusting its parameter, the node can then be run by typing ``roslaunch beamng_agent example.launch``.

Vehicle Control
---------------

The node establishes a connection to an already spawned vehicle in the simulation and subscribes to the
``beamng_agent/<vehicle_id>/control`` topic with message type ``beamng_msgs.msg.VehicleControl``.

+---------------+------------+-----------------------------------------------------------------------------+
|Field          | Type       | Definition                                                                  |
+===============+============+=============================================================================+
|steering       |``float32`` |Rotation of the steering wheel, from -1.0 to 1.0.                            |
+---------------+------------+-----------------------------------------------------------------------------+
|throttle       |``float32`` |Intensity of the throttle, from 0.0 to 1.0.                                  |
+---------------+------------+-----------------------------------------------------------------------------+
|brake          |``float32`` |Intensity of the brake, from 0.0 to 1.0.                                     |
+---------------+------------+-----------------------------------------------------------------------------+
|parkingbrake   |``float32`` |Intensity of the parkingbrake, from 0.0 to 1.0.                              |
+---------------+------------+-----------------------------------------------------------------------------+
|clutch         |``float32`` |Clutch level, from 0.0 to 1.0.                                               |
+---------------+------------+-----------------------------------------------------------------------------+
|gear           |``int8``    |Gear index, -1 equals backwards, 0 equals neutral, 1 to X equals Nth gear.   |
+---------------+------------+-----------------------------------------------------------------------------+

