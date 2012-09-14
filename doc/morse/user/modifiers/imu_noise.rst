IMU noise
=========

This modifier allows to simulate Gaussian noise for accelerometer and
gyroscope sensors of an IMU.
No bias is modeled so far.

Files
-----

- Python: ``$MORSE_ROOT/src/morse/modifiers/imu_noise.py``

Modified data
-------------

It modifies so the following variables :

	- **angular_velocity**: adds Gaussian noise with std dev ``gyro_std``
	  (default: 0.05)
	- **linear_acceleration**: adds Gaussian noise with std dev ``accel_std``
	  (default: 0.5)

Available methods
-----------------

- ``noisify``: Simulate noise for accelerometer and gyroscope.

By passing a dictionary with ``gyro_std`` and/or ``accel_std`` the applied
noise can be configured.
