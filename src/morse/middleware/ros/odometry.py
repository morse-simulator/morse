import logging; logger = logging.getLogger("morse." + __name__)
import mathutils
import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('nav_msgs'); roslib.load_manifest('geometry_msgs')
import rospy
from geometry_msgs.msg import Vector3, Quaternion, Pose, Twist
from nav_msgs.msg import Odometry
from morse.middleware.ros import ROSPublisherTF

class OdometryPublisher(ROSPublisherTF):

    def initalize(self):
        ROSPublisherTF.initalize(self, Odometry)
        # store the frame ids
        self.frame_id = self.kwargs.get("frame_id", "/odom")
        self.child_frame_id = self.kwargs.get("child_frame_id", "/base_footprint")

        logger.info("Initialized the ROS odometry sensor with frame_id '%s' " +\
                    "and child_frame_id '%s'", self.frame_id, self.child_frame_id)

    def default(self, ci='unused'):
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
        self.sendTransform(self.get_position(),
                           self.get_orientation(),
                           odometry.header.stamp,
                           odometry.child_frame_id,
                           odometry.header.frame_id)


    def get_orientation(self):
        """ Get the orientation from the local_data and return a quaternion """
        euler = mathutils.Euler((self.component_instance.local_data['roll'],
                                 self.component_instance.local_data['pitch'],
                                 self.component_instance.local_data['yaw']))
        quaternion = euler.to_quaternion()
        return quaternion

    def get_position(self):
        """ Get the position from the local_data and return a ROS Vector3 """
        position = Vector3()
        position.x = self.component_instance.local_data['x']
        position.y = self.component_instance.local_data['y']
        position.z = self.component_instance.local_data['z']
        return position

    def get_pose(self):
        """ Get the pose from the local_data and return a ROS Pose """
        pose = Pose()
        pose.position = self.get_position()
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
