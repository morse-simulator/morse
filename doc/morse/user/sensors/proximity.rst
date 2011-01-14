Robot proximity sensor
======================

This sensor can be used to determine which other robots are within a
certain radius of the sensor. It performs its test based only on distance.


Files
-----

- Blender: ``$MORSE_ROOT/data/morse/components/sensors/morse_proximity.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/proximity.py``


Local data
----------

- **near_robots**: (Dictionary) A list of the robots located within the given radius.
  The keys of the dictionary are the robot names, and the values are the distances
  (in meters) from the sensor.

Configurable parameters
-----------------------

The Empty object corresponding to this sensor has the following parameters:

- **Range**: (Float) The distance, in meters beyond which this sensor is
  unable to locate other robots.


Applicable modifiers
--------------------

No proximity modifiers available at the moment
