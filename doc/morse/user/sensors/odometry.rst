Odometry sensor
================

This sensor produces relative displacement with respect to the position and
rotation in the previous Blender tick. It can compute too the position of the
robot with respect to its original position, and the associated speed.

The angles for yaw, pitch and roll are given in radians.

Files
-----

- Blender: ``$MORSE_ROOT/data/sensors/odometry.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/odometry.py``

Local data
----------

- **dS**: (float) curvilign distance since last tick

- **dx**: (float) delta of X coordinate of the sensor 
- **dy**: (float) delta of Y coordinate of the sensor 
- **dz**: (float) delta of Z coordinate of the sensor 
- **dyaw**: (float) delta of rotation angle with respect to the Z axis 
- **dpitch**: (float) delta of rotation angle with respect to the Y axis 
- **droll**: (float) delta of rotation angle with respect to the X axis 

- **x**: (float) X coordinate of the sensor
- **y**: (float) Y coordinate of the sensor
- **z**: (float) Z coordinate of the sensor
- **yaw**: (float) rotation angle with respect to the Z axis
- **pitch**: (float) rotation angle with respect to the Y axis
- **roll**: (float) rotation angle with respect to the X axis

- **vx**: (float) linear velocity related to the X coordinate of the sensor
- **vy**: (float) linear velocity related to the Y coordinate of the sensor
- **vz**: (float) linear velocity related to the Z coordinate of the sensor
- **wz**: (float) angular velocity related to the Z coordinate of the sensor
- **wy**: (float) angular velocity related to the Y coordinate of the sensor
- **wx**: (float) angular velocity related to the X coordinate of the sensor


Applicable modifiers
--------------------

This sensor always provides perfect data.
To obtain more realistic readings, it is recommended to add modifiers.

- **Noise modifier**: Adds random Gaussian noise to the data
- **Odometry Noise modifier**: Simulate scale factor error and gyroscope drift
