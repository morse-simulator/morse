Waypoint target movement
========================

This actuator reads the coordinates of a destination point, and moves the robot
towards the given point, with the robot restricted to moving only forward,
backwards or turning over its Z axis.

This controller is meant to be used mainly by non-holonomic robots.  

The speeds provided are internally adjusted to the Blender time measure,
following the formula: ``blender_speed = given_speed * tics``, where
**tics** is the number of times the code gets executed per second.
The default vaule in Blender is ``tics = 60``.

Files
-----

-  Blender: ``$MORSE_ROOT/data/morse/components/controllers/morse_destination_control.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/destination.py``

Local data
----------

-  **x**: (float) Destination X coordinate
-  **Y**: (float) Destination Y coordinate
-  **Z**: (float) Destination Z coordinate
-  **speed**: (float) Movement speed. Rotation speed is used as **speed**/2

.. note:: Coordinates are given with respect to the origin of Blender's coordinate axis.

Applicable modifiers
--------------------

- :doc:`UTM modifier <../modifiers/utm>`: Will add an offset to the Blender coordinates according to the parameters set on the scene.
- :doc:`NED <../modifiers/ned>`: Changes the coordinate reference to use North (X), East (Y), Down (Z)
