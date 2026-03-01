Overview
========

The drone_controller package provides a ROS 2 node for commanding drones in BeamNG.tech.
It subscribes to :code:`/cmd_vel` and :code:`/cmd_vel_stamped` (geometry commands) and
publishes :code:`/drone_control` (BeamNG drone control) to the simulation.

For full setup and testing commands, see :doc:`drone_testing`.
