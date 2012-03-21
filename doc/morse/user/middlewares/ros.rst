ROS
===

Installation
------------

Due to lacking Python 3 compatibility of ROS message generation, you need to
patch ROS. Furthermore you need to install PyYAML.

Please follow the instructions in the :doc:`installation procedure  <../installation/mw/ros>`.

Files
-----

- Python: ``$MORSE_ROOT/src/morse/modifiers/ros_mw.py``

Generation of ROS-node and topics
----------------------------------

The ROS middleware creates one ROS-node called "morse" and on ROS topic for every sensor and every actuator. 
The names of the ROS-topics are generated in the following way:

``<name_of_parent_blender_object>/<name_of_blender_object>``

Available methods
-----------------

- ``read_message``: Gets information from a ROS-topic and stores it in the
  ``local_data`` dictionary of the associated component. 
- ``post_message``: Formats the contents of ``local_data`` into a string,
  and sends it to a ROS-topic
  
Available extensions
--------------------

These files contain additional methods that can be used with the ROS middleware.
To use them, it is necessary to list the file as the third element of the middleware
list, in the ``component_config.py`` script, as described in the :doc:`hooks <../hooks>`
documentation.

- accelerometer: Stored in the file ``$MORSE_ROOT/src/morse/middleware/ros/accelerometer.py``.
  Available methods:

	- ``post_twist``: Write a ``geometry_msgs/Twist`` messages to export
	  velocity information. No angular information, only linear ones.

- battery sensor:  Stored in the file ``$MORSE_ROOT/src/morse/middleware/ros/battery.py``.
  Available methods:

	- ``post_float32``: Exports the charge information as a single ``float32``

- camera sensor: Stored in the file  ``$MORSE_ROOT/src/morse/middleware/ros/camera.py``.
  It has one available method:

	- ``post_image``: Reads image information from the camera sensor and
	  publishes them as ``sensor_msgs/Image``. Moreover, it exports camera
	  information through a port of kind ``sensor_msgs/CameraInfo``.

- clock sensor : Stored in the file ``$MORSE_ROOT/src/morse/middleware/ros/clock.py``;
  It has one available method:

	- ``post_clock``: Exports the clock simulator as a ``rosgraph/Clock``
	  message.

- GPS sensor: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/gps.py``.
  Available methods:

	- ``post_poseStamped``: Reads sensor-information from the simulated GPS
	  sensor and publishes them as a ``geometry_msgs/PoseStamped`` message.
	- ``post_odometry``: Reads sensor-information from the simulated GPS
	  sensor and publishes them as a ``nav_msgs/Odometry`` message.

- IMU sensor: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/imu.py``.
  It has one available method:

	- ``post_odometry``: Exports the data of the IMU sensor as a ``nav_msgs/Odometry``
	- ``post_velocity_twist``: Exports the data of the IMU sensor as a ``geometry_msgs/Twist``

- Kuka-Arm controller: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/kuka_jointState.py``.
  Available methods:

	- ``read_jointState``: Reads a ``sensor_msgs/JointState`` message from the
	  specific ROS-topic and and applies the KuKa-arm-movement according to
	  them. NOTE: The JointState-messages must have a data array of length 7.

- Light controller: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/light.py``.
  Available callback:

	- read a single ROS boolean and control if we must or not emit light

- Platine controller: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/platine.py``.
  Available methods:

	- ``read_Vector3``: Reads a ``geometry_msgs/Vector3`` message from the
	  specific ROS-topic and sets the local data for "pan" and "tilt"
	  according to the rotation axis (pan: y-axis, tilt: z-axis)

- Pose sensor: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/pose.py``.
  It has two available methods:

	- ``post_odometry``: Reads sensor-information from the pose sensor
	  publishes them as a ``nav_msgs/Odometry`` message.
	- ``post_poseStamped``: Reads sensor-information from the pose sensor
	  publishes them as a ``geometry_msgs/PoseStamped`` message.


- v-omega actuator: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/read_vw_twist.py``.
  Available methods:

	- ``read_twist``: Reads a ``geometry_msgs/Twist`` message from the
	  specific ROS-topic and stores values for ``v`` and ``w`` in
	  ``local_data``. This is designed to be used with the v_omega actuator 
  
- xy-omega actuator: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/read_xyw_twist.py``.
  Available methods:

	- ``read_twist``: Reads a ``geometry_msgs/Twist`` message from the
	  specific ROS-topic and stores values for ``x``, ``y`` and ``w`` in
	  ``local_data``. This is designed to be used with the xy_omega actuator

- semantic camera sensor: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/semantic_camera.py``.
  Available methods:

	- ``post_string``: Posts the result of the camera sensor as a string, each
	  field separated by a comma. 

	- ``post_list_code``: Posts the result of the camera sensor as a string,
	  encoding a lisp expression, each field are separated by a space.

- SICK laserscanner: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/sick.py``.
  Available methods:

	- ``post_2DLaserScan``: Reads sensor-information from the simulated
	  SICK-laserscanner and publishes them as a ``sensor_msgs/PointCloud``
	  message.


- 2D waypoint actuator: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/waypoint2D.py``.
  Available methods:

    - ``read_waypoint``: Reads a ``geometry_msgs/Pose2D`` message from the specific ROS-topic and stores values for ``x``, ``y`` and ``z`` in ``local_data``. This is designed to be used with the waypoint actuator




