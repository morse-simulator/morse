Orientation actuator
====================

This actuator reads the values of angles of rotation around the 3 axis
and applies them to the associated robot.
This rotation is applied instantly (not realistic).
Angles are expected in radians.

Files 
-----

-  Blender: ``$MORSE_ROOT/data/actuators/orientation.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/orientation.py``

Local data 
----------

-  **yaw**: (float) rotation around Z axis
-  **pitch**: (float) rotation around Y axis
-  **roll**: (float) rotation around X axis

Applicable modifiers 
--------------------

- :doc:`NED <../modifiers/ned>`: Changes the angles reference to use North (X), East (Y), Down (Z)
