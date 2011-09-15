Steer/Force actuator
====================

This actuator reads the values of the steering angle, the engine power and the braking force to drive a car like vehicle.
It is meant to work with robots implementing the `Blender Vehicle Wrapper 
<http://www.tutorialsforblender3d.com/Game_Engine/Vehicle/Vehicle_1.html>`_,
such as the :doc:`Hummer robot <../robots/hummer>`.

.. note:: Robots implementing the Vehicle Wrapper must be pointing towards their local Y axis.
    This means the robots will be oriented differently with respect to all other MORSE components

Files 
-----

-  Blender: ``$MORSE_ROOT/data/morse/actuators/steer_force.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/steer_force.py``

Local data 
----------

-  **steer**: (float) the angle (in radians) of the wheels with respect to the vehicle
-  **force**: (float) the force applied to the traction wheels. A negative force will make the vehicle move forward. A positive force will make it go backwards.
-  **brake**: (float) the amount of force applied to the brakes. It opposes the **force**.

Services
--------

No available services

Applicable modifiers 
--------------------

No available modifiers
