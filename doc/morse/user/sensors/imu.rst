Inertial measurement unit (IMU)
===============================

This sensor emulates an Inertial Measurement Unit, measuring the distance that
a robot has moved and the angles the robot has turned in 3D. Also the velocity
and acceleration for each of these values is available. The arrays are in the
following format: [x, y, z, roll, pitch, yaw]

Files
-----

- Blender: ``$MORSE_ROOT/data/sensors/imu.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/imu.py``

Local data
++++++++++

- **distance**: (float) The distance travelled/turned since the last tick
- **velocity**: (float) Computed as distance over time = distance * ticks
- **acceleration**: (float) Computed as velocity difference over time = velocity * ticks

Applicable modifiers
--------------------

- No applicable modifiers at the moment
