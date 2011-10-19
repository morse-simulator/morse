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

-  Blender: ``$MORSE_ROOT/data/morse/actuators/keyboard.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/keyboard.py``

Configurable parameters
-----------------------

-  **Speed**: (float) Movement speed of the parent robot

Applicable modifiers
--------------------

No available modifiers
