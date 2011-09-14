Armature actuator
=================

This actuator offers services to control a generic blender armature.
It is already included in some of the robot components that can use it,
as mentioned below.


Files 
-----

- Python: ``$MORSE_ROOT/src/morse/actuators/armature_actuator.py``
- Blender: ``$MORSE_ROOT/data/morse/actuators/armature_actuator.blend``

- Robots implementing the actuator:

	- PR2: ``$MORSE_ROOT/data/morse/robots/pr2/pr2_25_morse.blend``
	- KUKA LWR: ``$MORSE_ROOT/data/morse/robots/kuka_lwr.blend``

Use of the armature actuator
----------------------------

A sample python script of how to access the armature actuator via sockets can be found at:
``$MORSE_ROOT/examples/morse/scenarii/armature_samples/armature_services_tests.py``.


:mod:`armature_actuator` Module
-------------------------------

.. automodule:: morse.actuators.armature_actuator
    :members:
    :undoc-members:
    :show-inheritance:


TODO
----
- Add an option to activate a stream of data via component_mw
- ...
