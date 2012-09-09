Stabilized Flight for quadrotor
===============================

This actuator controls a stabilized quadrotor using a linear model. 
Basically, it reads a command (phi, theta, psi, h), and computes, using a
second order filter the speed of the robot.
The quadrotor does not need ``Rigid Body`` physics.

Files
-----

  - Blender: ``$MORSE_ROOT/data/actuators/stabilized_quadrotor.blend``
  - Python: 
	
		- ``$MORSE_ROOT/src/morse/actuators/stabilized_quadrotor.py``
		- ``$MORSE_ROOT/src/morse/helpers/filt2.py``

Local data 
----------

  - **theta_c**: (float) Commands the pitch of the quadrotor. It is directly
	related to the acceleration of the quadrotor on the **x** axis.
  - **phi_c**: (float) Commands the roll of the quadrotor. It is directly
	related to the acceleration of the quadrotor on the **y** axis.
  - **psi_c**: (float) Commands the theta of the quadrotor. 
  - **h_c**: (float) The expected **z** of the quadrotor.

.. note:: Coordinates are given with respect to the origin of Blender's coordinate axis.
.. note:: The actuator does not consider friction force. Setting theta_c or
		  phi_c to 0 leads to a constant speed on axis x or y.

