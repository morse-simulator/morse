Gyroscope sensor
================

This sensor emulates a Gyroscope, providing the yaw, pitch and roll angles of
the sensor object with respect to the Blender world reference axes.

The angles are given in radians.

Files
-----

- Blender: ``$MORSE_ROOT/data/sensors/gyroscope.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/gyroscope.py``

Local data
----------

- **yaw**: (float) rotation angle with respect to the Z axis
- **pitch**: (float) rotation angle with respect to the Y axis
- **roll**: (float) rotation angle with respect to the X axis

.. note:: Coordinates are given with respect to the origin of Blender's coordinate axis.

Applicable modifiers
--------------------

This sensor always provides perfect data.
To obtain more realistic readings, it is recommended to add modifiers.

- :doc:`NED <../modifiers/ned>`: Changes the coordinate reference to use North
  (X), East (Y), Down (Z)
- :doc:`Noise modifier <../modifiers/gauss_noise>`: Adds random Gaussian noise to the data
