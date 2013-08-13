Pioneer 3-DX platform
=====================

This robot uses the Physics Constraints in Blender to allow the wheels to
behave more realistically. The wheels turn as the robot moves, and they have
``Rigid Body`` physics, so that they can also have collisions with nearby
objects.

It has two differential drive wheels, and an additional caster wheel for
stability.  Since the wheels of this robot use the ``Rigid Body`` physics, it
must be controlled with the :doc:`v_omega_diff_drive
<../actuators/v_omega_diff_drive>` actuator.

.. image:: ../../../media/robots/pioneer3dx.png 
  :align: center
  :width: 600

Files
-----

- Blender: ``$MORSE_ROOT/data/robots/pioneer3dx.blend``
- Python: ``$MORSE_ROOT/src/morse/robots/pioneer3dx.py``


Adjustable parameters
---------------------

These parameters are inherent to the Blender model for the robot, and must be
modified directly in the Blender file:

- The **Mass** of the robot can be changed in the **Properties >> Physics**
  panel
- The **Friction** coefficient of the robot can be adjusted in the
  **Properties >> Material** panel

Configurable parameters
-----------------------

The robot itself has several properties that describe its physical behaviour.
None of these properties have an effect in the current version of the robot,
but may be used in future releases.
These can be changed using the Builder API:

- **HasSuspension**: (Boolean) flag that determines if the wheels move
  independently of the body of the robot. For the case of the Pioneer 3-DX,
  this should always be ``False``
- **HasSteering**: (Boolean) flag
  that determines if the wheels turn independently of the body of the robot.
  For the case of the Pioneer 3-DX, this should always be ``False``
- **Influence**: (double)
- **Friction**: (double)
