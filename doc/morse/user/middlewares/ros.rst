ROS
===

Installation
------------

Please follow the instructions in the
:doc:`installation procedure  <../installation/mw/ros>`.


Generation of ROS-node and topics
----------------------------------

The ROS middleware creates one ROS-node called "morse" and on ROS topic for
every sensor and every actuator. The names of the ROS-topics are generated in
the following way: ``<name_of_parent_blender_object>/<name_of_blender_object>``.

For instance, if you have an odometry sensor called "odometry" on a robot
called "atrv", the ROS Odometry messages will be published on ``/atrv/odometry``.


Available methods
-----------------

Please refer to :py:mod:`morse.middleware.ros` for a list of supported publishers
and subscribers.

morse.middleware.ros.abstract_ros.AbstractROS
---------------------------------------------

**type** : `std_msgs/Header <http://ros.org/doc/api/std_msgs/html/msg/Header.html>`_

Base class for all ROS Publishers and Subscribers


morse.middleware.ros.abstract_ros.ROSPublisher
----------------------------------------------

**type** : `std_msgs/Header <http://ros.org/doc/api/std_msgs/html/msg/Header.html>`_

Base class for all ROS Publishers


morse.middleware.ros.abstract_ros.ROSPublisherTF
------------------------------------------------

**type** : `std_msgs/Header <http://ros.org/doc/api/std_msgs/html/msg/Header.html>`_

Base class for all ROS Publishers with TF support


morse.middleware.ros.abstract_ros.ROSReader
-------------------------------------------

**type** : `std_msgs/Header <http://ros.org/doc/api/std_msgs/html/msg/Header.html>`_

Base class for all ROS Subscribers


morse.middleware.ros.abstract_ros.StringPublisher
-------------------------------------------------

**type** : `std_msgs/String <http://ros.org/doc/api/std_msgs/html/msg/String.html>`_

Publish a string containing a printable representation of the local data.


morse.middleware.ros.abstract_ros.StringReader
----------------------------------------------

**type** : `std_msgs/String <http://ros.org/doc/api/std_msgs/html/msg/String.html>`_

Subscribe to a String topic and log its data decoded as UTF-8.


morse.middleware.ros.accelerometer.TwistPublisher
-------------------------------------------------

**type** : `geometry_msgs/Twist <http://ros.org/doc/api/geometry_msgs/html/msg/Twist.html>`_

Publish the velocity of the acceleromter sensor.
No angular information, only linear ones.



morse.middleware.ros.battery.Float32Publisher
---------------------------------------------

**type** : `std_msgs/Float32 <http://ros.org/doc/api/std_msgs/html/msg/Float32.html>`_

Publish the charge of the battery sensor.


morse.middleware.ros.clock.ClockPublisher
-----------------------------------------

**type** : `rosgraph_msgs/Clock <http://ros.org/doc/api/rosgraph_msgs/html/msg/Clock.html>`_

Publish the simulator clock.


morse.middleware.ros.depth_camera.DepthCameraPublisher
------------------------------------------------------

**type** : `sensor_msgs/PointCloud2 <http://ros.org/doc/api/sensor_msgs/html/msg/PointCloud2.html>`_

Publish the depth field from the Camera perspective as XYZ point-cloud.
And send the transformation between the camera and the robot through TF.



morse.middleware.ros.destination.PointReader
--------------------------------------------

**type** : `geometry_msgs/Point <http://ros.org/doc/api/geometry_msgs/html/msg/Point.html>`_

Subscribe to a Point topic and set x,y,z local data.


morse.middleware.ros.force_torque.WrenchReader
----------------------------------------------

**type** : `geometry_msgs/Wrench <http://ros.org/doc/api/geometry_msgs/html/msg/Wrench.html>`_

Subscribe to a Wrench topic and set force and torque (x,y,z) local data.


morse.middleware.ros.gps.NavSatFixPublisher
-------------------------------------------

**type** : `sensor_msgs/NavSatFix <http://ros.org/doc/api/sensor_msgs/html/msg/NavSatFix.html>`_

Publish the GPS position of the robot.


morse.middleware.ros.imu.ImuPublisher
-------------------------------------

**type** : `sensor_msgs/Imu <http://ros.org/doc/api/sensor_msgs/html/msg/Imu.html>`_

Publish the data of the IMU sensor (without covariance).


morse.middleware.ros.infrared.RangePublisher
--------------------------------------------

**type** : `sensor_msgs/Range <http://ros.org/doc/api/sensor_msgs/html/msg/Range.html>`_

Publish the range of the infrared sensor.


morse.middleware.ros.jido_posture.JointStatePublisher
-----------------------------------------------------

**type** : `sensor_msgs/JointState <http://ros.org/doc/api/sensor_msgs/html/msg/JointState.html>`_

Publish the data of the posture sensor (for Jido).


morse.middleware.ros.jointstate.JointStatePR2Publisher
------------------------------------------------------

**type** : `sensor_msgs/JointState <http://ros.org/doc/api/sensor_msgs/html/msg/JointState.html>`_

Publish the data of the posture sensor after filling missing PR2 joints.


morse.middleware.ros.jointstate.JointStatePublisher
---------------------------------------------------

**type** : `sensor_msgs/JointState <http://ros.org/doc/api/sensor_msgs/html/msg/JointState.html>`_

Publish the data of the posture sensor.


morse.middleware.ros.jointtrajectorycontrollers.JointTrajectoryControllerStatePublisher
---------------------------------------------------------------------------------------

**type** : `pr2_controllers_msgs/JointTrajectoryControllerState <http://ros.org/doc/api/pr2_controllers_msgs/html/msg/JointTrajectoryControllerState.html>`_

Publish the data of the pr2 joint sensor.


morse.middleware.ros.kinect.XYZRGBPublisher
-------------------------------------------

**type** : `sensor_msgs/PointCloud2 <http://ros.org/doc/api/sensor_msgs/html/msg/PointCloud2.html>`_

Publish the merged image and depth field from the Kinect perspective as XYZRGB point-cloud.
And send the transformation between the camera and the robot through TF.



morse.middleware.ros.kuka_jointstate.JointStateReader
-----------------------------------------------------

**type** : `sensor_msgs/JointState <http://ros.org/doc/api/sensor_msgs/html/msg/JointState.html>`_

Subscribe to a JointState topic and set kuka_{1-7} to the position[0-6].


morse.middleware.ros.laserscanner.LaserScanPublisher
----------------------------------------------------

**type** : `sensor_msgs/LaserScan <http://ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html>`_

Publish the ``range_list`` of the laser scanner.


morse.middleware.ros.laserscanner.PointCloud2Publisher
------------------------------------------------------

**type** : `sensor_msgs/PointCloud2 <http://ros.org/doc/api/sensor_msgs/html/msg/PointCloud2.html>`_

Publish the ``point_list`` of the laser scanner.


morse.middleware.ros.light.BoolReader
-------------------------------------

**type** : `std_msgs/Bool <http://ros.org/doc/api/std_msgs/html/msg/Bool.html>`_

Subscribe to a boolean topic to control if we must or not emit light.


morse.middleware.ros.motion_vw.TwistReader
------------------------------------------

**type** : `geometry_msgs/Twist <http://ros.org/doc/api/geometry_msgs/html/msg/Twist.html>`_

Subscribe to a motion command and set ``v`` and ``w`` local data.


morse.middleware.ros.motion_xyw.TwistReader
-------------------------------------------

**type** : `geometry_msgs/Twist <http://ros.org/doc/api/geometry_msgs/html/msg/Twist.html>`_

Subscribe to a motion command and set ``x``, ``y`` and ``w`` local data.


morse.middleware.ros.odometry.OdometryPublisher
-----------------------------------------------

**type** : `nav_msgs/Odometry <http://ros.org/doc/api/nav_msgs/html/msg/Odometry.html>`_

Publish the odometry of the robot. And send the transformation between
``frame_id`` and ``child_frame_id`` args, default '/odom' and
'/base_footprint' through TF.



morse.middleware.ros.orientation.QuaternionReader
-------------------------------------------------

**type** : `geometry_msgs/Quaternion <http://ros.org/doc/api/geometry_msgs/html/msg/Quaternion.html>`_

Subscribe to a Quaternion topic and set roll,pitch,yaw local data.


morse.middleware.ros.platine.Vector3Reader
------------------------------------------

**type** : `geometry_msgs/Vector3 <http://ros.org/doc/api/geometry_msgs/html/msg/Vector3.html>`_

Subscribe to a Vector3 topic and set pan,tilt local data, according to
the rotation axis (pan: y-axis, tilt: z-axis).



morse.middleware.ros.pose.PoseStampedPublisher
----------------------------------------------

**type** : `geometry_msgs/PoseStamped <http://ros.org/doc/api/geometry_msgs/html/msg/PoseStamped.html>`_

Publish the position and orientation of the robot.


morse.middleware.ros.pose.TFPublisher
-------------------------------------

**type** : `std_msgs/Header <http://ros.org/doc/api/std_msgs/html/msg/Header.html>`_

Publish the transformation between
``frame_id`` and ``child_frame_id`` args, default '/map' and
'/base_link' through TF.



morse.middleware.ros.ptu_posture.JointStatePublisher
----------------------------------------------------

**type** : `sensor_msgs/JointState <http://ros.org/doc/api/sensor_msgs/html/msg/JointState.html>`_

Publish the data of the posture sensor as a ROS JointState message


morse.middleware.ros.read_pose.PoseReader
-----------------------------------------

**type** : `geometry_msgs/Pose <http://ros.org/doc/api/geometry_msgs/html/msg/Pose.html>`_

Subscribe to a Pose topic and set ``x``, ``y``, ``z`` and ``roll``,
``pitch``, ``yaw`` local data.



morse.middleware.ros.semantic_camera.SemanticCameraPublisher
------------------------------------------------------------

**type** : `std_msgs/String <http://ros.org/doc/api/std_msgs/html/msg/String.html>`_

Publish the data of the semantic camera as a ROS String message, each
field separated by a comma, with newlines (for better visualization in console).



morse.middleware.ros.semantic_camera.SemanticCameraPublisherLisp
----------------------------------------------------------------

**type** : `std_msgs/String <http://ros.org/doc/api/std_msgs/html/msg/String.html>`_

Publish the data of the semantic camera as a ROS String message,
that contains a lisp-list (each field are separated by a space).

This function was designed for the use with CRAM and the Adapto group.



morse.middleware.ros.video_camera.VideoCameraPublisher
------------------------------------------------------

**type** : `sensor_msgs/Image <http://ros.org/doc/api/sensor_msgs/html/msg/Image.html>`_

Publish the image from the Camera perspective.
And send the intrinsic matrix information in a separate topic of type
`sensor_msgs/CameraInfo  <http://ros.org/wiki/rviz/DisplayTypes/Camera>`_.



morse.middleware.ros.waypoint2D.Pose2DReader
--------------------------------------------

**type** : `geometry_msgs/Pose2D <http://ros.org/doc/api/geometry_msgs/html/msg/Pose2D.html>`_

Subscribe to a Pose2D topic and set ``x``, ``y``, ``z`` local data.
This is designed to be used with the waypoint actuator.


