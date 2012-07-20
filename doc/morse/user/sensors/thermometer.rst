Thermometer sensor
==================

This sensor emulates a Thermometer, measuring the temperature with respect to
the distance to heat sources.  It defines a default temperature throughout the
scenario, which is affected by local fire sources. The temperature rises
exponentially when the distance between the sensor and the heat source
decreases. Temperature is given in degrees Celsius.

The default temperature is specified as a parameter of the
``Scene_Script_Holder`` Empty object in the simulation file. It is expressed in 
degrees Celsius.

Files
-----
- Blender: ``$MORSE_ROOT/data/sensors/thermometer.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/thermometer.py``

Local data
~~~~~~~~~~
- **temperature**: (float) The temperature in Celsius

Applicable modifiers
--------------------

- Noise modifier: Adds random Gaussian noise to the data
