Gripper actuator
================

Actuator capable of grabbing objects marked with the ``Graspable`` Game Property.
Currently it only works using services: **grab** and **release**.
When instructed to grab an object, it will check if it is within range,
and if so, will parent the grabbed object to itself.

.. note:: For objects to be detected and grabbed by the gripper, they must have the following settings
    in the **Physics Properties** panel:

    - **Actor** must be checked
    - **Collision Bounds** must be checked
    - **Physics Type** must be ``Rigid Body``

    This will work even for Static objects

This actuator is normally used together with robotic arms, such as the 
:doc:`kuka actuator <kuka_lwr>`.
To install the gripper on a robotic arm using the Morse Builder API,
it is necessary to make the gripper a child of the arm, and to place the gripper
in the correct position with respect to the arm. Example::

    kuka_arm = Actuator('kuka_lwr')
    kuka_arm.translate(x=0.1850, y=0.2000, z=0.9070)
    kuka_arm.rotate(x=1.5708, y=1.5708)
    Jido.append(kuka_arm)

    gripper = Actuator('gripper')
    gripper.translate(z=1.2800)
    kuka_arm.append(gripper)


.. warning:: This actuator does not simulate the physical interaction of the gripper
    fingers with the objects it grabs. Its purpose is to abstract the action of
    taking an object, for human-robot interaction experiments.

Files
-----

-  Blender: ``$MORSE_ROOT/data/actuators/gripper.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/gripper.py``

Local data
----------

-  **grab**: (boolean) Currently not used

Configurable parameters
-----------------------

The following parameters can be adjusted within the **Logic Bricks** of the ``Gripper`` object in Blender, in the properties of the ``Radar`` sensor.

- **Angle**: (float) aperture angle of the radar capable of detecting the graspable objects.
- **Distance**: (float) detection distance of the radar. Graspable objects further away from the gripper than this distance can not be held


Services
--------

- **grab**: (Synchronous service) Make the nearby object a child of the gripper, so that it will move around with the robot.

- **release**: (Synchronous service) Drop the object being held. When the object is released, its rotation will be restored to zero (this helps to avoid objects falling and rolling around, but only when the object is dropped on a surface near below)

Applicable modifiers
--------------------

No applicable modifiers
