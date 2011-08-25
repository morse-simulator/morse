Robot proximity sensor
======================

This sensor can be used to determine which other objects are within a
certain radius of the sensor. It performs its test based only on distance.
The type of tracked objects can be specified using the 'Track' property.


Files
-----

- Blender: ``$MORSE_ROOT/data/morse/components/sensors/morse_proximity.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/proximity.py``


Local data
----------

- **near_objects**: (Dictionary) A list of the tracked objects located within the given radius.
  The keys of the dictionary are the object names, and the values are the distances
  (in meters) from the sensor.
- **near_robots**: deprecated. Points to near_objects for compatibility sake.

Configurable parameters
-----------------------

The Empty object corresponding to this sensor has the following parameters:

- **Track**: (String) The type of tracked objects. This type is looked for as a
  game property of scene objects. You must then add a new game property to the objects
  you want to be detected by the proximity sensor. Default is "Robot_Tag".
- **Range**: (Float) The distance, in meters beyond which this sensor is
  unable to locate other robots.

Applicable modifiers
--------------------

No proximity modifiers available at the moment
