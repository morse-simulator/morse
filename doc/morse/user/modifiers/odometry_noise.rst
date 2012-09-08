Odometry noise
==============

This modifier allows to simulate two common issues when calculating odometry :

	- an error in the scale factor used to compute the distance from the value
	  returned by the odometer (parameter **factor**)
	- the gyroscope natural drift (parameter **gyro_drift** (rad by tick))

Files
-----

- Python: ``$MORSE_ROOT/src/morse/modifiers/odometry_noise.py``

Modified data
-------------

The modifier only accumulate errors for a 2D odometry sensor. It modifies so
the following variables :

	- **dS** by the scale factor
	- **dx** considering the scale factor and gyroscope drift
	- **dy** considering the scale factor and gyroscope drift
	- **dyaw** considering the gyroscope drift
	- **x** considering the scale factor and gyroscope drift
	- **y** considering the scale factor and gyroscope drift
	- **yaw** considering the gyroscope drift
	- **vx** considering the new **dx**
	- **vy** considering the new **dy**
	- **wz** considering the new **dyaw**

Available methods
-----------------

- ``noisify``: Simulate drift of gyroscope and possible error in the scale
  factor
