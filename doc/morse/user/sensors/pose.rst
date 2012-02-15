Pose sensor
===========

This sensor provides the absolute position and orientation with respect to
the origin of the Blender coordinate reference.
It basically does the same job as both of the
:doc:`GPS <gps>` and :doc:`Gyroscope <gyroscope>` sensors.
The position and orientation data is taken from the sensor's ``position_3d`` structure.
The angles for yaw, pitch and roll are given in radians in the range (-pi, pi).

Files
-----

- Blender: ``$MORSE_ROOT/data/sensors/pose.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/pose.py``

Local data
----------

- **x**: (float) X coordinate of the sensor
- **y**: (float) Y coordinate of the sensor
- **z**: (float) Z coordinate of the sensor
- **yaw**: (float) rotation angle with respect to the Z axis
- **pitch**: (float) rotation angle with respect to the Y axis
- **roll**: (float) rotation angle with respect to the X axis

.. note:: Coordinates are given with respect to the origin of Blender's coordinate axis.

Applicable modifiers
--------------------

This sensor always provides perfect data.
To obtain more realistic readings, it is recommended to add modifiers.

- :doc:`UTM modifier <../modifiers/utm>`: Will add an offset to the Blender
  coordinates according to the parameters set on the scene.
- :doc:`NED <../modifiers/ned>`: Changes the coordinate reference to use North
  (X), East (Y), Down (Z)
- :doc:`Noise modifier <../modifiers/gauss_noise>`: Adds random Gaussian noise to the data
