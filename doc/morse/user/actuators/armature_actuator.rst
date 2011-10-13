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

Available services
------------------

All services provided by this actuator are synchronous

- **get_channels**: Returns a list with the name of the bones in the armature

- **get_rotations**: Returns a dictionary, where the key is the name of the channel and the value is a tuple containing the euler rotation of the bone

- **get_rotation**: This method receives as parameter the name of one bone, and returns the tuple of its euler rotation

- **get_dofs**: Returns a dictionary, where the key is the name of the channel and the value is a list containing 3 integers, either 1 or 0 to indicate if the bone can rotate around each of the 3 axis

- **get_IK_minmax**: Returns a dictionary, where the key is the name of the channel and the value is a list of three lists. Each of the three lists contains two elements, indicating the minimum and maximum angles that the bone can rotate to

- **get_IK_limits**: Same result as the **get_dofs** service

- **set_rotation**: This method receives as parameters the name of a bone and a list with three floating point numbers, indicating the euler rotation to be given to the bone

- **get_channel_lengths**: Returns a dictionary, where the key is the name of the channel and the value is a floating point number indicating the length of the bone

- **get_robot_parent_name**: Returns the name of the robot that is parent to the KUKA LWR


Use of the armature actuator
----------------------------

A sample python script of how to access the armature actuator via sockets
can be found at:
``$MORSE_ROOT/examples/morse/scenarii/armature_samples/armature_services_tests.py``.



:mod:`armature_actuator` Module
-------------------------------

.. automodule:: morse.actuators.armature_actuator
    :members:
    :undoc-members:
    :show-inheritance:
