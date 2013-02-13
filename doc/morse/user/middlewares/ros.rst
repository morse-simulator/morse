ROS
===

Installation
------------

Please follow the instructions in the
:doc:`installation procedure  <../installation/mw/ros>`.


Generation of ROS-node and topics
----------------------------------

The ROS middleware creates one ROS-node called "morse" and on ROS topic for
every sensor and every actuator. The names of the ROS-topics are generated in
the following way: ``<name_of_parent_blender_object>/<name_of_blender_object>``.

For instance, if you have an odometry sensor called "odometry" on a robot
called "atrv", the ROS Odometry messages will be published on ``/atrv/odometry``.


Available methods
-----------------

Please refer to :py:mod:`morse.middleware.ros` for a list of supported publishers
and subscribers.
