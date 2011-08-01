PR2 Robot
=========

The MORSE model of the Willow Garage's PR2 robot.

For now the model's armatures can be read and set by the different pr2 services.


Model Info
----------

The model is imported from a Collada file that is generated from the `PR2 URDF file  <http://www.ros.org/wiki/pr2_description>`_.
The .dae file can be found at: ``$MORSE_ROOT/data/morse/components/robots/pr2/pr2.dae``
The imported .blend file can be found at: ``$MORSE_ROOT/data/morse/components/robots/pr2/pr2_25.blend``

The URDF to Collada converter changed all the object names, so these were remapped to the orignal URDF names. A renamed version of the PR2 model can be found at: ``$MORSE_ROOT/data/morse/components/robots/pr2/pr2_25_rename.blend``, this file includes the script that is used to rename all the objects.

A model with an armature for both arms and head can be found at: ``$MORSE_ROOT/data/morse/components/robots/pr2/pr2_25_armature.blend``.

A model with MORSE integration for the armature can be found at: ``$MORSE_ROOT/data/morse/components/robots/pr2/pr2_25_morse.blend``.


PR2 related Files
-----------------

- PR2 Collada file: ``$MORSE_ROOT/data/morse/components/robots/pr2/pr2.dae``
- PR2 original .blend Collada import: ``$MORSE_ROOT/data/morse/components/robots/pr2/pr2_25.blend``
- PR2 model with renamed objects: ``$MORSE_ROOT/data/morse/components/robots/pr2/pr2_25_rename.blend``
- PR2 model with armature: ``$MORSE_ROOT/data/morse/components/robots/pr2/pr2_25_armature.blend``
- PR2 model with MORSE integration: ``$MORSE_ROOT/data/morse/components/robots/pr2/pr2_25_morse.blend``

- PR2 robot script with MORSE services: ``$MORSE_ROOT/src/morse/robots/pr2.py```

- PR2 sample scenario: ``$MORSE_ROOT/examples/morse/scenarii/pr2_samples/pr2_services_test.blend``
- PR2 services test: ``$MORSE_ROOT/examples/morse/scenarii/pr2_samples/pr2_services_tests.py``


Use of the PR2
--------------

A sample python script of how to access the PR2 services via sockets can be found at ``$MORSE_ROOT/examples/morse/scenarii/pr2_samples/pr2_services_tests.py``.


Adjustable parameters
---------------------

Use the **Properties >> Physics** panel in Blender to adjust the **Mass** of the robot.

The friction coefficient of the robot can be adjusted in the **Properties >> Material** panel.


Services
--------

- **get_armatures**(): 
	(*Synchronous service*)
    MORSE Service that returns a list of all the armatures on the PR2 robot.

- **get_channels**(armature_name):
	(*Synchronous service*)
	MORSE Service that returns a list of the channels (bones) in the given armature 'armature_name'
	
- **get_dofs**(armature_name):
	(*Synchronous service*)
	MORSE Service that returns a dictionary with keys the channels of the given armature 'armature_name' and as values the corresponding dof axis as 'x', 'y' or 'z'.
	
- **get_dof**(armature_name, channel_name):
	(*Synchronous service*)
	MORSE Service that returns the dof axis of the given channel 'channel_name' on the given armature 'armature_name'.
	
- **get_rotations**(armature_name):
	(*Synchronous service*)
	MORSE Service that returns a dict with keys the channel names of the given armature 'armature_name', and values the rotation xyz values.
	
- **get_rotation**(armature_name, channel_name):
	(*Synchronous service*)
	MORSE Service that returns the rotation angles corresponding to the given channel 'channel_name' on the given armature 'armature_name'.
	
- **set_dof_rotations**(armature_name, angles):
	(*Synchronous service*)
	MORSE Service to set the rotion angles of the corresponding dof axes of the channels in 'armature_name'.
	angles must be a dict with as keys all the channel names of the given armature. And as values the angle of the channel dof axis that you want to set.
	
- **set_dof_rotation**(armature_name, channel_name, angle):
	(*Synchronous service*)
	MORSE Service to set the rotation angle of the corresponding dof axis of the 'channel_name' in 'armature_name'.
	

TODO
----

- Create sensors and actuators to control the PR2 armature. `A SensorActuator class would be handy for this  <https://sympa.laas.fr/sympa/arc/morse-users/2011-07/msg00099.html>`_.
- Add an actuator to move the translating PR2 torso.
- Expand the armature to include the hands.
- Add an actuator to control the movement of the PR2 base.
- Add Sensors for the PR2.