Straight line movement
======================

This actuator reads the coordinates of a destination point, and moves the robot
in a straight line towards the given point, without turning.  It provides a
very simplistic movement, and can be used for testing or for robots with
holonomic movement.  The speeds provided are internally adjusted to the Blender
time measure.

Files
-----

  - Blender: ``$MORSE_ROOT/data/actuators/destination.blend``
  - Python: ``$MORSE_ROOT/src/morse/actuators/destination.py``

Local data 
----------

  - **x**: (float) Destination X coordinate
  - **y**: (float) Destination Y coordinate
  - **z**: (float) Destination Z coordinate

.. note:: Coordinates are given with respect to the origin of Blender's coordinate axis.


Properties
----------

  - **Speed**: (float) The robot speed in Blender units per second
  - **Tolerance**: (float) The tolerance to determine if the target has been reached


.. note:: Properties are modifiable in the Builder API. Example:

	.. code-block:: python

		atrv = Robot('atrv')
		dest = Actuator('destination')
		atrv.append(dest)
		dest.properties(Speed=2.5, Tolerance=0.5)


Applicable modifiers
--------------------

- :doc:`UTM modifier <../modifiers/utm>`: Will add an offset to the Blender coordinates according to the parameters set on the scene.
- :doc:`NED <../modifiers/ned>`: Changes the coordinate reference to use North (X), East (Y), Down (Z)
