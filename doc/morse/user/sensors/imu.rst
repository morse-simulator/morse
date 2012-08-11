Inertial measurement unit (IMU)
===============================

This sensor emulates an Inertial Measurement Unit, measuring the angular
velocity and linear acceleration including acceleration due to gravity.

The robot needs to have a physics enabled.
Angular Velocity is read from the parent robot, linear acceleration is
computed by differentiation of the linear velocities of the robot.
The measurements are given in the IMU coordinate system, so the location
and rotation of the IMU with respect to the robot is taken into account.

Files
-----

- Blender: ``$MORSE_ROOT/data/sensors/imu.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/imu.py``

Local data
++++++++++

- **angular_velocity**: (float array ) rates in IMU x, y, z axes
- **linear_acceleration**: (float array ) acceleration in IMU x, y, z axes

Applicable modifiers
--------------------

- No applicable modifiers at the moment
