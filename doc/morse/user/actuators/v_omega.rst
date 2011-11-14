Linear and angular speed (V, W) actuator 
========================================

This actuator reads the values of linear and angular speed and applies
them to the robot as direct translation.
The speeds provided are internally adjusted to the Blender time measure,
following the formula: ``blender_speed = given_speed * tics``, where
**tics** is the number of times the code gets executed per second.
The default value in Blender is ``tics = 60``.

Files 
-----

-  Blender: ``$MORSE_ROOT/data/morse/actuators/v_omega.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/v_omega.py``

Local data 
----------

-  **v**: (float) linear velocity in meters per second
-  **w**: (float) angular velocity in radians per second

Services
--------

- **set_speed**: (synchronous service) Modifies **v** and **w** according to the
  parameters given.

    +------------+---------------+------------------+
    | Parameters | ``v``         | float            |
    |            +---------------+------------------+
    |            | ``w``         | float            |
    +------------+---------------+------------------+

    Parameters: ``(v, w)``


- **stop**: (synchronous service) Stop the robot (sets **v** and **w** to 0.0)

Applicable modifiers 
--------------------

No available modifiers
