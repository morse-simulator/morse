KUKA LWR arm Robot
==================

The MORSE model of the KUKA LWR arm.

The KUKA LWR uses the :doc:`armature_actuator <../actuators/armature_actuator>` for control of the armatures.


Model Info
----------

This model is a modified version of:
``$MORSE_ROOT/data/morse/components/robots/kuka_arm-rig.blend``


KUKA LWR related Files
----------------------

- Blender model: ``$MORSE_ROOT/data/morse/components/robots/kuka_arm-rig.blend``
- Python file: ``$MORSE_ROOT/src/morse/robots/kuka_lwr.py``
- Armature actuator: ``$MORSE_ROOT/src/morse/actuators/armature_actuator.py``


Use of the KUKA LWR
-------------------

A sample python script of how to access the KUKA LWR via sockets can be found at:
``$MORSE_ROOT/examples/morse/scenarii/armature_samples/armature_services_tests.py``.


Adjustable parameters
---------------------

Use the **Properties >> Physics** panel in Blender to adjust the **Mass** of the robot.

The friction coefficient of the robot can be adjusted in the **Properties >> Material** panel.
	

TODO
----

- ...
