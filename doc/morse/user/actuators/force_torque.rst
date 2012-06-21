Force and torque actuator 
=============================================

This actuator takes force and torque inputs and applies them to the
parent robot as local forces and torques in the robot coordinate system.

Physics needs to be enabled for the robot blender object.

Files 
-----

-  Blender: ``$MORSE_ROOT/data/actuators/force_torque.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/force_torque.py``

Local data 
----------

-  **force**: (float array) force along x, y, z
-  **torque**: (float array) torque around x, y, z

Applicable modifiers 
--------------------

No available modifiers
