BeamNG Teleop Keyboard Control
====================
[beamng_teleop_keyboard](https://github.com/BeamNG/beamng-ros-integration/tree/master/beamng_teleop_keyboard) is a generic Keyboard Packages is built for teleoperating ROS robots using Twist message from [geometry_messages](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html). 


Running beamng_teleop_keyboard: 
^^^^^^^^^^^^^^^^^^
- Loading BeamNG-ROS bridge:
``roslaunch beamng_control example.launch``
 
- Calling Twist_message converter node:
``rosrun beamng_teleop_keyboard converter``
 
- Calling Teleop node:
``rosrun beamng_teleop_keyboard teleop_key``
 
- Loading beamng_agent node:
``roslaunch beamng_agent example.launch ``
