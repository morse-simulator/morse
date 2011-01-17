KUKA LWR arm actuator
=====================

This actuator reads a list of angles for the segments of the LWR arm
and applies them as local rotations.
Angles are expected in radians.

Files 
-----

-  Blender: No associated blender file, but must work with the model in
    ``$MORSE_ROOT/data/morse/components/robots/kuka_arm.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/kuka.py``

Local data 
----------

-  **seg0**: (float) rotation for the first segment. Around Z axis.
-  **seg1**: (float) rotation for the second segment. Around Y axis.
-  **seg2**: (float) rotation for the third segment. Around Z axis.
-  **seg3**: (float) rotation for the fourth segment. Around Y axis.
-  **seg4**: (float) rotation for the fifth segment. Around Z axis.
-  **seg5**: (float) rotation for the sixth segment. Around Y axis.
-  **seg6**: (float) rotation for the seventh segment. Around Z axis.

Configurable parameters
-----------------------

-  **speed**: (float) rotation speed for the movements of the segments

Applicable modifiers 
--------------------

No available modifiers
