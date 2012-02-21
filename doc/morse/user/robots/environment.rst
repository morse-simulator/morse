Environment robot
=================

This is a special case of component in MORSE. Since all sensors or actuators
must be attached to one robot, it would not normally be possible to use
"stand-alone" sensors in the environment.

If you need to use a sensor in this way, (*i.e.* for motion capture sensors,
or independent cameras) you should add an **environment robot** to the scene,
and make it the parent of your stand-alone sensors.

This robot has no visual representation, and consists of a single Blender
Empty. Its only purpose is to provide the base to attach sensors. A single
**environment robot** should be the parent of as many sensors as needed.

Files
-----

- Blender: ``$MORSE_ROOT/data/robots/environment.blend``
- Python: ``$MORSE_ROOT/src/morse/robots/environment.py``

Adjustable parameters
---------------------

No adjustable parameters
