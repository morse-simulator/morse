LAAS Jido robot
===============

This robot is build on a NeoBotix base with a Kuka lightweight arm

Files
-----

- Blender: ``$MORSE_ROOT/data/morse/robots/jido.blend``
- Python: ``$MORSE_ROOT/src/morse/robots/jido.py``

Adjustable parameters
---------------------

The arm is rigged with an armature, following the mechanics of the arm. It is
set to use Inverse Kinematics to follow an Empty object called
**Target_Empty**. The IK solver is only available during execution of
the simulation in Blender version 2.5 or higher.

Use the **Properties >> Physics** panel in Blender to adjust the **Mass** of the robot.

The friction coefficient of the robot can be adjusted in the **Properties >> Material** panel.
