Pan-Tilt unit control actuator
==============================

This actuator reads the rotation values for pan and tilt, and applies
them to the pan-tilt unit that must be set as children of this actuator.
Angles are expected in radians.

This component can be configured to be used manually as well.
In this case, the pan and tilt segments can be rotated using the following keys:

-  :kbd:`Page Up` tilt up
-  :kbd:`Page Down` tilt down
-  :kbd:`Home` pan left
-  :kbd:`Insert` pan right


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

-  **Speed**: (float) rotation speed for the movements of the pan-tilt unit
-  **Manual**: (boolean) select whether to use control from an external program or direct control using the **Logic Bricks**

Applicable modifiers 
--------------------

No available modifiers
