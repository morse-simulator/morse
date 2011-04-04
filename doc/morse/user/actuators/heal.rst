Victim heal actuator
======================

This actuator is specific for the ROSACE scenario, where the robot must be able
to aid human victims.
In the test scenarios, human victims are shown in *red*. When a robot approaches,
it can help the victims, which will gradually change their colour to *green*,
and change their status to healthy.
This actuator works only with the `human victim <../others/victim>` object.
It will detect if a victim is in front of the robot. When instructed to heal the victim,
it will change the Game Properties of the object to reduce its ``injured`` value.

Files
-----

  - Blender: ``$MORSE_ROOT/data/morse/components/controllers/morse_healer_beam.blend``
  - Python: ``$MORSE_ROOT/src/morse/actuators/healer.py``

Local data 
----------

  - **heal**: (int) Flag to indicate whether the robot is trying to help a victim or not

Applicable modifiers
--------------------

No available modifiers
