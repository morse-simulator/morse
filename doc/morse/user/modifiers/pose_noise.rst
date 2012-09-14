Pose noise
==========

This modifier allows to simulate Gaussian noise for pose measurements.
If the variable ``orientation`` exists, it is taken to be a unit quaternion
and noise added to it. Otherwise rotational noise will be added to the roll,
pitch and yaw variables.

Files
-----

- Python: ``$MORSE_ROOT/src/morse/modifiers/pose_noise.py``

Modified data
-------------

It modifies so the following variables :

	- **x**: add Gaussian noise with std dev ``pos_std`` (default: 0.05)
	- **y**: add Gaussian noise with std dev ``pos_std`` (default: 0.05)
	- **z**: add Gaussian noise with std dev ``pos_std`` (default: 0.05)
	- **orientation**: assumed to be a unit quaternion and multiplied with
	  a noise quaternion with std dev ``ros_std`` (default: 5 degrees)
	- **roll**: add Gaussian noise with std dev ``rot_std`` (default: 5 deg)
	- **pitch**: add Gaussian noise with std dev ``rot_std`` (default: 5 deg)
	- **yaw**: add Gaussian noise with std dev ``rot_std`` (default: 5 deg)

Available methods
-----------------

- ``noisify``: Simulate noise for translation and rotation of a pose measurement.

By passing a dictionary with ``pos_std`` and/or ``rot_std`` the applied
noise can be configured.
