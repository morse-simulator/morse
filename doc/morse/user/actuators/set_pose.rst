SetPose actuator
================

This actuator sets the robot to the absolute position and orientation with respect to
the origin of the Blender coordinate reference.
The orientation is set as a unit quaternion.

Files
-----

- Blender: ``$MORSE_ROOT/data/actuators/set_pose.blend``
- Python: ``$MORSE_ROOT/src/morse/actuators/set_pose.py``

Local data
----------

- **x**: (float) X coordinate of the robot
- **y**: (float) Y coordinate of the robot
- **z**: (float) Z coordinate of the robot
- **qw**: (float) scalar part of the quaternion
- **qx**: (float) x of vector part of the quaternion
- **qy**: (float) y of vector part of the quaternion
- **qz**: (float) z of vector part of the quaternion

.. note:: Coordinates are given with respect to the origin of Blender's coordinate axis.

Applicable modifiers
--------------------

- :doc:`NED <../modifiers/ned>`: Changes the coordinate reference to use North
  (X), East (Y), Down (Z)
