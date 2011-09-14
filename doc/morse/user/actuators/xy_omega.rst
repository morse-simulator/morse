Linear and angular speed (Vx, Vy, W) actuator 
=============================================

This actuator reads the values of forwards movement x, sidewards movement y and
angular speed w and applies them to the robot as direct translation. This
controller is supposed to be used with robots that allow for sidewards
movements.

The speeds provided are internally adjusted to the Blender time measure,
following the formula: ``blender_speed = given_speed * tics``, where
**tics** is the number of times the code gets executed per second.
The default value in Blender is ``tics = 60``.

Files 
-----

-  Blender: ``$MORSE_ROOT/data/morse/actuators/xy_omega.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/xy_omega.py``

Local data 
----------

-  **x**: (float) linear velocity in x direction (forward movement)
-  **y**: (float) linear velocity in y direction (sidewards movement)
-  **w**: (float) angular velocity

Applicable modifiers 
--------------------

No available modifiers
