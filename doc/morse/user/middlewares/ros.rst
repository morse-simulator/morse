ROS
===

Installation
------------

Due to lacking Python 3 compatibility of ROS message generation, you need to
patch ROS. Furthermore you need to install PyYAML.

Please follow the instructions in the :doc:`installation procedure  <../installation/mw/ros>`.

Files
-----

- Blender: ``$MORSE_ROOT/data/morse/middleware/ros_mw.blend``
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

- v-omega actuator: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/read_vw_twist.py``.
  Available methods:

    - ``read_twist``: Reads a ``geometry_msgs/Twist`` message from the specific ROS-topic and stores values for ``v`` and ``w`` in ``local_data``. This is designed to be used with the v_omega actuator 
  
- xy-omega actuator: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/read_xyw_twist.py``.
  Available methods:

    - ``read_twist``: Reads a ``geometry_msgs/Twist`` message from the specific ROS-topic and stores values for ``x``, ``y`` and ``w`` in ``local_data``. This is designed to be used with the xy_omega actuator

- 2D waypoint actuator: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/waypoint2D.py``.
  Available methods:

    - ``read_waypoint``: Reads a ``geometry_msgs/Pose2D`` message from the specific ROS-topic and stores values for ``x``, ``y`` and ``z`` in ``local_data``. This is designed to be used with the waypoint actuator

- Platine controller: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/platine.py``.
  Available methods:

    - ``read_Vector3``: Reads a ``geometry_msgs/Vector3`` message from the specific ROS-topic and sets the local data for "pan" and "tilt" according to the rotation axis (pan: y-axis, tilt: z-axis)

- GPS sensor: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/gps.py``.
  Available methods:

    - ``post_poseStamped``: Reads sensor-information from the simulated GPS sensor and publishes them as a ``geometry_msgs/PoseStamped`` message.
    - ``post_odometry``: Reads sensor-information from the simulated GPS sensor and publishes them as a ``nav_msgs/Odometry`` message.

- Kuka-Arm controller: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/kuka_jointState.py``.
  Available methods:

    - ``read_jointState``: Reads a ``sensor_msgs/JointState`` message from the specific ROS-topic and and applies the KuKa-arm-movement according to them. NOTE: The JointState-messages must have a data array of length 7.

- SICK laserscanner: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/sick.py``.
  Available methods:

    - ``post_2DLaserScan``: Reads sensor-information from the simulated SICK-laserscanner and publishes them as a ``sensor_msgs/PointCloud`` message.

- Pose sensor: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/pose.py``.
  It has two available methods:

    - ``post_odometry``: Reads sensor-information from the pose sensor publishes them as a ``nav_msgs/Odometry`` message.
    - ``post_poseStamped``: Reads sensor-information from the pose sensor publishes them as a ``geometry_msgs/PoseStamped`` message.

- Odometry sensor: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/odometry_sensor.py``.
  It has two available methods:

    - ``post_pose``: Reads sensor-information from the pose sensor publishes them as a ``geometry_msgs/PoseStamped`` message.
    - ``post_twist``: Reads sensor-information from the pose sensor publishes them as a ``geometry_msgs/Twist`` message.
      NOTE: The angular part of the twist messages is build as follows: (x,y,z) = (roll, pitch, yaw)

- IMU sensor: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/ros/imu.py``. 
  It has one available method:

    - ``post_velocity_twist``: Reads velocity-information from the IMU sensor and publishes them as a ``geometry_msgs/Twist`` message.

- camera sensor: Stored in the file  ``$MORSE_ROOT/src/morse/middleware/ros/camera.py``.
  It has one available method:

	- ``post_image``: Reads image information from the camera sensor and
	  publishes them as ``sensor_msgs/Image``
