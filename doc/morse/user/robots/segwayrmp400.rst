Segway RMP 400 platform
=======================

The base of the **MANA** robot at LAAS.

This robot uses the Physics Constraints in Blender to allow the wheels to
behave more realistically. The wheels turn as the robot moves, and they have
``Rigid Body`` physics, so that they can also have collisions with nearby
objects.

.. warning::
  Because of the physics constraints used in this robot, its wheels must NOT be
  children of the robot when the simulation is started. To properly include an
  RMP 400 robot in a scene, it must be done using a special class of the
  Builder API, and then calling a method to unparent the wheels from the robot.
  See below for an example.

Files
-----

- Blender: ``$MORSE_ROOT/data/robots/segwayrmp400.blend``
- Python: ``$MORSE_ROOT/src/morse/robots/segwayrmp400.py``

Configurable parameters
-----------------------

The robot itself has several properties that describe its physical behaviour.
None of these properties have an effect in the current version of the robot,
but may be used in future releases.
These can be changed using the Builder API:

- **HasSuspension**: (Boolean) flag that determines if the wheels move
  independently of the body of the robot. For the case of the Segway RMP 400,
  this should always be ``False``
- **HasSteering**: (Boolean) flag
  that determines if the wheels turn independently of the body of the robot.
  For the case of the Segway RMP 400, this should always be ``False``
- **Influence**: (double)
- **Friction**: (double)



Adjustable parameters
---------------------

Use the **Properties >> Physics** panel in Blender to adjust the **Mass** of the robot.

The friction coefficient of the robot can be adjusted in the **Properties >> Material** panel.


Example in Builder API
----------------------

To add a Segway RMP 400 robot, it is necessary to use the ``WheeledRobot``
class. Also, after giving any transformations to the robot (translation or
rotation), you **MUST** call the method ``unparent_wheels`` of the robot
instance. Otherwise the robot will not move.

.. code-block:: python

    from morse.builder.morsebuilder import *

    # Append Segway robot to the scene
    robot = WheeledRobot('segwayrmp400')
    robot.rotate(z=0.73)
    robot.translate(x=-2.0, z=0.2)
    robot.unparent_wheels()
