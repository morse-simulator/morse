ROS
===

Installation
------------

Because ROS is not currently Python 3 compatible, you need to patch ROS.

Please follow the instructions here: :doc:`ROS installation <ros/ros_installation>`.

Files
-----

- Blender: ``$MORSE_ROOT/data/morse/components/middleware/ros_empty.blend``
- Python: ``$MORSE_ROOT/src/morse/modifiers/ros_mw.py``

Generation of ROS-node and topics
----------------------------------

The ROS middleware creates one ROS-node called "morse" and on ROS topic for every sensor and every actuator. 
The names of the ROS-topics are generated in the following way:

``<name_of_parent_blender_object>/<name_of_blender_object>``

Available methods
-----------------

- ``read_message``: Gets information from a ROS-topic and stores it in the
  ``local_data`` dictionary of the associated component. 
- ``post_message``: Formats the contents of ``local_data`` into a string,
  and sends it to a ROS-topic
  
Available extensions
--------------------

- ``read_waypoint``: Reads a geometry_msgs/Pose2D from a ROS-topic and stores it in the
  ``local_data`` dictionary of the associated component. 
  
Extensions for posting nav_msgs/Odometry and sensor_msgs/LaserScan are on their way...

TODO

- finish post_laserScan and post_odometry
