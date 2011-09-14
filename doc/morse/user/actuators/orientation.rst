Orientation actuator
====================

This actuator reads the values of angles of rotation around the 3 axis
and applies them to the associated robot.
This rotation is applied instantly (not realistic).
Angles are expected in radians.

Files 
-----

-  Blender: ``$MORSE_ROOT/data/morse/actuators/orientation.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/orientation.py``

Local data 
----------

-  **rx**: (float) rotation around X axis
-  **ry**: (float) rotation around Y axis
-  **rz**: (float) rotation around Z axis

Applicable modifiers 
--------------------

No available modifiers
