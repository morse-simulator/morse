MOOS
====

Installation
------------

MOOS and pymoos must both be installed in order to use the MOOS middleware. 

Please follow the instructions in the :doc:`installation procedure  <../installation>`.

-----

- Python: ``$MORSE_ROOT/src/morse/modifiers/moos_datastream.py``

Generation of MOOS app and variables
------------------------------------

The MOOS middleware creates a MOOS application called "MORSE_SIM" and posts a
message to the MOOS database for each variable output by a sensor. 
The names of the MOOS database variables are generated in the following way:

``<name_of_parent_blender_object>_<name_of_blender_object>_<variable_name>``

Available methods
-----------------

- ``read_message``: Gets information from a MOOS variable and stores it in the
  ``local_data`` dictionary of the associated component. 
- ``post_message``: Formats the contents of ``local_data`` into a string,
  and sends it to the MOOS database with a variable name given by the above
  convention.
  
Available extensions
--------------------

These files contain additional methods that can be used with the MOOS middleware.
To use them, it is necessary to list the file as the third element of the middleware
list, in the ``component_config.py`` script, as described in the :doc:`hooks <../hooks>`
documentation.

- GPS sensor: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/moos/gps.py``.
  Available methods:

	- ``post_gps``: Reads sensor-information from the simulated GPS sensor and
	  publishes it as three MOOS variables: ``zEast``,``zNorth``, and
	  ``zHeight``

- SICK laserscanner: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/moos/sick.py``.
  Available methods:

	- ``post_2DLaserScan``: Reads sensor-information from the simulated
	  SICK-laserscanner and publishes them as a ``zLidarDist`` vector. e.g.
	  [1x1080]{3.2,4.4,9.8,...,2.9}.  The scan window size, angular
	  resolution, and maximum range are also stored in the ``sScanAngle``,
	  ``sScanResolution``, and ``sScanRange`` variables respectively.  

- Pose sensor: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/moos/pose.py``.
  It has one available method:

	- ``post_pose``: Reads sensor-information from the pose sensor and
	  publishes it as:  ``simEast``, ``simNorth``, ``simHeight``, ``simYaw``,
	  ``simRoll``, and ``simPitch``.
 
- IMU sensor: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/moos/imu.py``. 
  It has one available method:

	- ``post_imu``: Reads angular velocity and linear acceleration from the
	  IMU sensor and publishes them as a ``zGyroX``, ``zGyroY``, ``zGyroZ``,
	  ``zAccelX``, ``zAccelY``, and ``zAccelZ``.

- Gyroscope sensor: Stored in the file  ``$MORSE_ROOT/src/morse/middleware/moos/gyroscope.py``.
  It has one available method:

	- ``post_gyroscope``: Reads orientation information from the gyroscope
	  sensor and publishes them as ``zYaw``, ``zRoll``,  and ``zPitch``.
