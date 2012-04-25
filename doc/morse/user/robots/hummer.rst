Hummer car robot
================

This is a generic car like robot. It is driven using steering, power and braking as provided by the :doc:`steer/force actuator <../actuators/steer_force>`.
This vehicle uses the Blender `vehicle wrapper <http://www.blender.org/documentation/blender_python_api_2_59_0/bge.types.html#bge.types.KX_VehicleWrapper>`_ constraint, to give it a realistic behaviour, including the interaction of the wheels with the ground and suspension.

.. image:: ../../../media/robots/hummer.png 
  :align: center
  :width: 600

Files
-----

- Blender: ``$MORSE_ROOT/data/robots/hummer.blend``
- Python: ``$MORSE_ROOT/src/morse/robots/hummer.py``

Adjustable parameters
---------------------

Use the **Properties >> Physics** panel in Blender to adjust the **Mass** of the robot.

The friction coefficient of the robot can be adjusted in its .blend file. When the robot
is selected, the **Logic Editor** panel will display its **Game Properties**.

- **friction**: (float) Wheel's friction to the ground. Determines how fast the robot can accelerate from a standstill.
    Also affects steering wheel's ability to turn the vehicle.
    A value of ``0`` gives very low acceleration. Higher values permit a higher acceleration.
