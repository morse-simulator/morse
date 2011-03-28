Pan-Tilt unit control actuator
==============================

This actuator reads the rotation values for pan and tilt, and applies
them to the pan-tilt unit that must be set as children of this actuator.
Angles are expected in radians.

Files 
-----

-  Blender: ``$MORSE_ROOT/data/morse/components/controllers/morse_platine_control.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/platine.py``

Local data 
----------

-  **pan**: (float) rotation around the Z axis
-  **tilt**: (float) rotation around the Y axis

Configurable parameters
-----------------------

-  **speed**: (float) rotation speed for the movements of the platine

Applicable modifiers 
--------------------

No available modifiers
