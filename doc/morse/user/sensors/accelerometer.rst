Accelerometer sensor
====================

This sensor emulates an Accelerometer/Podometer, measuring the distance that a robot has moved, the current speed and current acceleration.

Files
-----

- Blender: ``$MORSE_ROOT/data/morse/components/sensors/morse_accelerometer.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/accelerometer.py``

Local data
++++++++++

- **distance**: (float) The distance travelled since the last tick
- **velocity**: (float) Computed as distance over time = distance * ticks
- **acceleration**: (float) Computed as velocity difference over time = velocity * ticks

Applicable modifiers
--------------------

- No applicable modifiers at the moment
