Teleport actuator
=================

This actuator teleports the robot to the absolute position and orientation
with respect to the origin of the Blender coordinate reference.
Angles are expected in radians.

Files
-----

- Blender: ``$MORSE_ROOT/data/actuators/teleport.blend``
- Python: ``$MORSE_ROOT/src/morse/actuators/teleport.py``

Local data
----------

- **x**: (float) X coordinate of the robot
- **y**: (float) Y coordinate of the robot
- **z**: (float) Z coordinate of the robot
- **yaw**: (float) rotation around Z axis
- **pitch**: (float) rotation around Y axis
- **roll**: (float) rotation around X axis

.. note:: Coordinates are given with respect to the origin of Blender's coordinate axis.

Applicable modifiers
--------------------

- :doc:`NED <../modifiers/ned>`: Changes the coordinate reference to use North
  (X), East (Y), Down (Z)
