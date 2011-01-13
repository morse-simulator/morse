Keyboard actuator
=================

This actuator does not require a connection with external data. It simply
responds to the keyboard arrows to generate movement instructions for the robot
attached.

When parented to a robot, the user can press the arrow keys to modify the
linear and angular velocities (V, W) of the robot.

-  :kbd:`Up` forward
-  :kbd:`Down` backwards
-  :kbd:`Left` turn left
-  :kbd:`Right` turn right

Files
-----

-  Blender: ``$ORS_ROOT/data/morse/components/controllers/morse_manual_control.blend``
-  Python: ``$ORS_ROOT/src/morse/actuators/keyboard.py``

Applicable modifiers
--------------------

No available modifiers
