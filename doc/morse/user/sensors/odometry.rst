Odometry sensor
================

This sensor produces relative displacement with respect to the position
and rotation in the previous Blender tick.
The position and orientation data is taken from the sensor's ``position_3d`` structure

The angles for yaw, pitch and roll are given in radians.

Files
-----

- Blender: ``$MORSE_ROOT/data/morse/sensors/odometry.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/odometry.py``

Local data
----------

- **dx**: (float) delta of X coordinate of the sensor
- **dy**: (float) delta of Y coordinate of the sensor
- **dz**: (float) delta of Z coordinate of the sensor
- **dyaw**: (float) delta of rotation angle with respect to the Z axis
- **dpitch**: (float) delta of rotation angle with respect to the Y axis
- **droll**: (float) delta of rotation angle with respect to the X axis


Applicable modifiers
--------------------

This sensor always provides perfect data.
To obtain more realistic readings, it is recommended to add modifiers.

- **Noise modifier**: Adds random Gaussian noise to the data
