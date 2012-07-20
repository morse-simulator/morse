GPS sensor
==========

This sensor emulates a GPS, providing the exact coordinates in the Blender
scene. The coordinates provided by the GPS are with respect to the origin of
the Blender coordinate reference.

Files
-----
- Blender: ``$MORSE_ROOT/data/sensors/gps.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/gps.py``

Local data
~~~~~~~~~~
- **x**: (float) X coordinate of the sensor
- **y**: (float) Y coordinate of the sensor
- **z**: (float) Z coordinate of the sensor

.. note:: Coordinates are given with respect to the origin of Blender's
  coordinate axis.

Applicable modifiers
--------------------

This sensor always provides perfect data, with respect to an arbitrary point.
To obtain more realistic readings, it is recommended to add modifiers.
The two which are specially used for the GPS information are:

- :doc:`UTM modifier <../modifiers/utm>`: Will add an offset to the Blender
  coordinates according to the parameters set on the scene.
- :doc:`NED <../modifiers/ned>`: Changes the coordinate reference to use North
  (X), East (Y), Down (Z)
- :doc:`Noise modifier <../modifiers/gauss_noise>`: Adds random Gaussian noise to the data
