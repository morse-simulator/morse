Force and torque actuator 
=============================================

This actuator takes force and torque inputs and applies them to the
parent robot as local forces and torques.
The forces and torques are transformed from the actuator frame to the
parent robot frame and then applied to the robot blender object.
If the property RobotFrame is set to True it will be applied
directly in the robot frame without changes.

Physics needs to be enabled for the robot blender object.

Files 
-----

  -  Blender: ``$MORSE_ROOT/data/actuators/force_torque.blend``
  -  Python: ``$MORSE_ROOT/src/morse/actuators/force_torque.py``

Local data 
----------
 
  -  **force**: (float array) force along x, y, z
  -  **torque**: (float array) torque around x, y, z
  
.. note:: Inputs with respect to the actuator coordinate axis.

Properties
----------

  - **RobotFrame**: (bool) If set to true the inputs are applied in the Robot coordinate frame instead of the actuator frame.

Applicable modifiers 
--------------------

No available modifiers
