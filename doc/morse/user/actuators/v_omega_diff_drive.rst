Differential drive actuator: Linear and angular speed (V, W)
============================================================

This actuator reads the values of linear and angular speed and applies them to
the robot as speeds for the wheels. It only works with robots of the type
``WheeledRobot``, such as the :doc:`Segway RMP 400 <../robots/segwayrmp400>`
and the :doc:`Pioneer 3-DX <../robots/pioneer3dx>`.  The movement of the robot
is more realistic, but also depends on more factors,
such as the friction between the wheels and the surface.

The speeds for the left and right wheels are calculated as::

    left_speed = (v - e w) / R

    right_speed = (v + e w) / R

Where:

- **v** is the linear velocity given as parameter
- **w** is the angular velocity given as parameter
- **e** is half of the distance between the left and right wheels
- **R** is the radius of the wheels


Files 
-----

-  Blender: ``$MORSE_ROOT/data/actuators/v_omega_diff_drive.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/v_omega_diff_drive.py``

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


Example in Builder API
----------------------

This kind of actuator should only be added to a robot created from the
``WheeledRobot`` class, such as the :doc:`Segway RMP 400
<../robots/segwayrmp400>` robot.

.. code-block:: python

    from morse.builder import *

    # Append Segway robot to the scene
    robot = WheeledRobot('segwayrmp400')
    robot.unparent_wheels()

    # Add the actuator
    motion = Actuator('v_omega_diff_drive')
    robot.append(motion)
