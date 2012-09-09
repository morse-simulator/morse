Rotorcraft Waypoint actuator
============================

This actuator reads the coordinates of a destination waypoint, and flies a
robot with ``Rigid Body`` physics to the given point by controlling attitude
and thrust.
This controller is meant to be used by quadrotors and similar flying robots.
It is a simple cascaded PD-controller where the outer PD loop
controls the attitude setpoint based on the position. The inner loop
applies torques to the robot to achieve that attitude. Thrust (force in
z-direction of the robot) is also applied by a simple PD altitude controller
with a feed-forward part for the nominal hover thrust.

While a robot is moving towards a given waypoint, a property of the
**MorseRobotClass** will be changed in indicate the status of the robot.
The ``movement_status`` property will take one of these values: 
**Transit** or **Arrived**.

Files
-----

-  Blender: ``$MORSE_ROOT/data/actuators/rotorcraft_waypoint.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/rotorcraft_waypoint.py``

Local data
----------

- **x**: (float) Destination X coordinate
- **y**: (float) Destination Y coordinate
- **z**: (float) Destination Z coordinate
- **yaw**: (float) Desired yaw angle (heading) in radians
- **tolerance**: (float) Radius around the target destination where the robot
  is considered to have reached the goal

.. note:: Coordinates are given with respect to the origin of Blender's
    coordinate axis.

Configurable parameters
-----------------------

- **HorizontalPgain**: (float) proportional gain for the outer horizontal
  position [xy] loop.
- **HorizontalDgain**: (float) derivative gain for the outer horizontal
  position [xy] loop.
- **VerticalPgain**: (float) proportional gain for the altitude loop.
- **VerticalDgain**: (float) derivative gain for the altitude loop.
- **YawPgain**: (float) proportional gain for yaw control of the inner loop.
- **YawDgain**: (float) derivative gain for yaw control of the inner loop.
- **RollPitchPgain**: (float) proportional gain for roll/pitch control of
  the inner loop.
- **RollPitchDgain**: (float) derivative gain for roll/pitch control of
  the inner loop.
- **MaxBankAngle**: (float) limit the maximum roll/pitch angle of the robot.
  This effectively limits the horizontal acceleration of the robot.
- **Target**: (string) name of a blender object in the scene.
  When specified, this object will be placed at the coordinates given to the
  actuator, to indicate the expected destination of the robot.
  Make sure that this object has ``NO_COLLISION`` set.

These properties can be changed directly from a Builder API script, as in the
following example:

.. code-block:: python

    from morse.builder.morsebuilder import *
    from math import radians

    # Create a waypoint controller
    waypoint = Actuator('rotorcraft_waypoint')
    waypoint.properties(Target='myTargetObject')
    waypoint.properties(MaxBankAngle=radians(20))


Services
--------

- **goto**: (Asynchronous service) This method can be used to give a one time
  instruction to the actuator.  When the robot reaches the destination, it will
  send a reply, indicating that the new status of the robot is "Stop". The
  function can be called with 4 or 5 parameters, corresponding to the variables
  in ``local_data``.  The coordinates are mandatory, while the value for
  **tolerance** can be omitted.

    +------------+---------------+------------------+
    | Parameters | ``x``         | float            |
    |            +---------------+------------------+
    |            | ``y``         | float            |
    |            +---------------+------------------+
    |            | ``z``         | float            |
    |            +---------------+------------------+
    |            | ``yaw``       | float            |
    |            +---------------+------------------+
    |            | ``tolerance`` | float (optional) |
    +------------+---------------+------------------+

    Parameters: ``(x, y, z, yaw[, tolerance])``


- **get_status**: (Synchronous service) Ask the actuator to send a message
  indicating the current movement status of the parent robot. There are two
  possible states: **Transit** and **Arrived**.

Applicable modifiers
--------------------

- :doc:`UTM modifier <../modifiers/utm>`: Will add an offset to the Blender
  coordinates according to the parameters set on the scene.
- :doc:`NED <../modifiers/ned>`: Changes the coordinate reference to use North
  (X), East (Y), Down (Z)
