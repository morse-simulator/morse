import logging; logger = logging.getLogger("morse." + __name__)
import mathutils
import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('nav_msgs'); roslib.load_manifest('geometry_msgs')
import rospy
from geometry_msgs.msg import Vector3, Quaternion, Pose, Twist, TransformStamped
from nav_msgs.msg import Odometry
from morse.middleware.ros.tfMessage import tfMessage
from morse.middleware.ros import ROSPublisher

class OdometryPublisher(ROSPublisher):

    def initalize(self):
        ROSPublisher.initalize(self, Odometry)
        self.register_tf()

        # store the frame ids
        self.frame_id = self.kwargs.get("frame_id", "/odom")
        self.child_frame_id = self.kwargs.get("child_frame_id", "/base_footprint")

        logger.info("Initialized the ROS odometry sensor with frame_id '%s' " +\
                    "and child_frame_id '%s'", self.frame_id, self.child_frame_id)

    def default(self, ci=None):
        """ Publish the data of the Odometry sensor as a ROS Odometry message
        """
        odometry = Odometry()
        odometry.header = self.get_ros_header()
        odometry.child_frame_id = self.child_frame_id

        # fill pose and twist
        odometry.pose.pose = self.get_pose()
        odometry.twist.twist = self.get_twist()

        self.publish(odometry)

        # send current odometry transform
        self.sendTransform(self.get_translation(),
                           self.get_orientation(),
                           odometry.header.stamp,
                           odometry.child_frame_id,
                           odometry.header.frame_id)


    def sendTransform(self, translation, rotation, time, child, parent):
        """
        :param translation: the translation of the transformtion as geometry_msgs/Vector3
        :param rotation: the rotation of the transformation as a geometry_msgs/Quaternion
        :param time: the time of the transformation, as a rospy.Time()
        :param child: child frame in tf, string
        :param parent: parent frame in tf, string

        Broadcast the transformation from tf frame child to parent on ROS topic ``"/tf"``.
        """

        t = TransformStamped()
        t.header.frame_id = parent
        t.header.stamp = time
        t.child_frame_id = child
        t.transform.translation = translation
        t.transform.rotation = rotation

        tfm = tfMessage([t])

        self.publish_tf(tfm)

    def get_orientation(self):
        """ Get the orientation from the local_data and return a quaternion """
        euler = mathutils.Euler((self.component_instance.local_data['roll'],
                                 self.component_instance.local_data['pitch'],
                                 self.component_instance.local_data['yaw']))
        quaternion = euler.to_quaternion()
        return quaternion

    def get_translation(self):
        """ Get the position from the local_data and return a ROS Vector3 """
        translation = Vector3()
        translation.x = self.component_instance.local_data['x']
        translation.y = self.component_instance.local_data['y']
        translation.z = self.component_instance.local_data['z']
        return translation

    def get_pose(self):
        """ Get the pose from the local_data and return a ROS Pose """
        pose = Pose()
        pose.position = self.get_translation()
        pose.orientation = self.get_orientation()
        return pose

    def get_twist(self):
        """ Get the twist from the local_data and return a ROS Twist """
        twist = Twist()
        twist.linear.x = self.component_instance.local_data['vx']
        twist.linear.y = self.component_instance.local_data['vy']
        twist.linear.z = self.component_instance.local_data['vz']
        twist.angular.x = self.component_instance.local_data['wx']
        twist.angular.y = self.component_instance.local_data['wy']
        twist.angular.z = self.component_instance.local_data['wz']
        return twist
