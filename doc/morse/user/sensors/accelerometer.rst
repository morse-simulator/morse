Accelerometer sensor
====================

This sensor emulates an Accelerometer/Podometer, measuring the distance that a
robot has moved, the current speed and current acceleration. Measurements are
done for the 3 axes (X, Y, Z) for velocity and acceleration.  The values for
velocity and acceleration are measured at each tic of the Game Engine,
measuring the difference in distance from the previous tic, and the
estimated time between tics (60 tics per second is the default in Blender).

Files
-----

- Blender: ``$MORSE_ROOT/data/sensors/accelerometer.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/accelerometer.py``

Local data
++++++++++

- **distance**: (float) The distance travelled since the last tick
- **velocity**: (float array) Instantaneous speed in X, Y, Z. Computed as
  distance over time = distance * ticks
- **acceleration**: (float array) Instantaneous change in speed in X, Y, Z.
  Computed as velocity difference over time = velocity * ticks

Applicable modifiers
--------------------

- No applicable modifiers at the moment
