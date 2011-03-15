Linear and angular speed (V, W) actuator 
========================================

This actuator reads the values of forwarts movement x, sidewarts movement y and angular speed w and applies
them to the robot as direct tranlation. This controller is supposed to be used with robots that allow for sidewarts movements.

The speeds provided are internally adjusted to the Blender time measure,
following the formula: ``blender_speed = given_speed * tics``, where
**tics** is the number of times the code gets executed per second.
The default vaule in Blender is ``tics = 60``.

Files 
-----

-  Blender: ``$MORSE_ROOT/data/morse/components/controllers/morse_xy_control.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/xy_omega.py``

Local data 
----------

-  **x**: (float) linear velocity in x direction (forward movement)
-  **y**: (float) linear velocity in y direction (sidewarts movement)
-  **w**: (float) angular velocity

Applicable modifiers 
--------------------

No available modifiers
