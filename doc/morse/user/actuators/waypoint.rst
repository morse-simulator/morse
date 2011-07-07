Waypoint target movement
========================

This actuator reads the coordinates of a destination point, and moves the robot
towards the given point, with the robot restricted to moving only forward,
backwards or turning around its Z axis.
This controller is meant to be used mainly by non-holonomic robots.  

While a robot is moving towards a given waypoint, a property of the **MorseRobotClass** will be changed in indicate the status of the robot. The ``movement_status`` property will change value from **Stop** to **Transit**.

The movement speed of the robot is internally adjusted to the Blender time measure,
following the formula: ``blender_speed = given_speed * tics``, where
**tics** is the number of times the code gets executed per second.
The default value in Blender is ``tics = 60``.

This actuator also implements a simple obstacle avoidance mechanism. The blend file contains
the **Motion_Controller** empty, as well as two additional Empty objects: **Radar.L** and **Radar.R**.
These detect nearby objects to the left or right of the robot, and will instruct the robot to
turn in the opposite direction of the obstacle.
If the radar objects are not present, the controller will not have any obstacle avoidance,
and the robot can get blocked by any object between it and the target.

.. note:: For objects to be detectable by the radars, they must have the following settings
    in the **Physics Properties** panel:

    - **Actor** must be checked
    - **Collision Bounds** must be checked

    This will work even for Static objects


Files
-----

-  Blender: ``$MORSE_ROOT/data/morse/components/controllers/morse_destination_control.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/destination.py``

Local data
----------

-  **x**: (float) Destination X coordinate
-  **y**: (float) Destination Y coordinate
-  **z**: (float) Destination Z coordinate
-  **tolerance**: (float) Radius around the target destination where the robot is considered
    to have reached the goal
-  **speed**: (float) Movement speed. Rotation speed is used as **speed**/2

.. note:: Coordinates are given with respect to the origin of Blender's coordinate axis.

Configurable parameters
-----------------------

-  **Speed**: (float) movement speed for the robot, given in meters per second
-  **Target**: (string) name of a blender object in the scene. When specified, this object will
    be placed at the coordinates given to the actuator, to indicate the expected destination of the robot

Services
--------

- **goto**: (Asynchronous service) This method can be used to give a one time instruction to the actuator.
    When the robot reaches the destination, it will send a reply, indicating that the new status of the robot
    is "Stop". The function can be called with 3 to 5 parameters, corresponding to the variables in ``local_data``.
    The coordinates are mandatory, while the values for **tolerance** and **speed** can be omitted.

    +------------+---------------+------------------+
    | Parameters | ``x``         | float            |
    |            +---------------+------------------+
    |            | ``y``         | float            |
    |            +---------------+------------------+
    |            | ``z``         | float            |
    |            +---------------+------------------+
    |            | ``tolerance`` | float (optional) |
    |            +---------------+------------------+
    |            | ``speed``     | float (optional) |
    +------------+---------------+------------------+

    Parameters: ``(x, y, z[, tolerance[, speed]])``


- **stop**: (Synchronous service) This method will instruct the robot to go to its current position, and reply immediately. If a **goto** request is ongoing, it will finish at the next tick, and it will itself also reply indicating it has finished.

- **get_status**: (Synchronous service) Ask the actuator to send a message indicating the current movement status of the parent robot. There are two possible states: **Transit** and **Stop**.

Applicable modifiers
--------------------

- :doc:`UTM modifier <../modifiers/utm>`: Will add an offset to the Blender coordinates according to the parameters set on the scene.
- :doc:`NED <../modifiers/ned>`: Changes the coordinate reference to use North (X), East (Y), Down (Z)
