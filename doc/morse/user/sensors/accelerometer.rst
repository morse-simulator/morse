Accelerometer sensor
====================

This sensor emulates an Accelerometer/Podometer, measuring the distance that a robot has moved, the current speed and current acceleration.

Files
-----

- Blender: ``$ORS_ROOT/data/morse/components/sensors/morse_accelerometer.blend``
- Python: ``$ORS_ROOT/src/morse/sensors/accelerometer.py``

Local data
++++++++++

- **distance**: (float) The distance travelled since the last tick
- **velocity**: (float) Computed as distance over time = distance * ticks
- **accleration**: (float) Computed as velocity difference over time = velocity * ticks

Applicable modifiers
--------------------

- No applicable modifiers at the moment
