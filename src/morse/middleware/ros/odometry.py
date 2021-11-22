import logging; logger = logging.getLogger("morse." + __name__)
from geometry_msgs.msg import Vector3, Quaternion, Pose, Twist
from nav_msgs.msg import Odometry
from morse.middleware.ros import ROSPublisherTF, mathutils

class OdometryPublisher(ROSPublisherTF):
    """ Publish the odometry of the robot. And send the transformation between
    ``frame_id`` and ``child_frame_id`` args, default '/odom' and
    '/base_footprint' through TF.
    """
    ros_class = Odometry
    default_frame_id = '/odom'

    def initialize(self):
        ROSPublisherTF.initialize(self)
        # store the frame ids
        self.child_frame_id = self.kwargs.get("child_frame_id", "/base_footprint")

        logger.info("Initialized the ROS odometry sensor with frame_id '%s' " +\
                    "and child_frame_id '%s'", self.frame_id, self.child_frame_id)

    def default(self, ci='unused'):
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
        euler = mathutils.Euler((self.data['roll'],
                                 self.data['pitch'],
                                 self.data['yaw']))
        quaternion = euler.to_quaternion()
        return quaternion

    def get_position(self):
        """ Get the position from the local_data and return a ROS Vector3 """
        position = Vector3()
        position.x = self.data['x']
        position.y = self.data['y']
        position.z = self.data['z']
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
        twist.linear.x = self.data['vx']
        twist.linear.y = self.data['vy']
        twist.linear.z = self.data['vz']
        twist.angular.x = self.data['wx']
        twist.angular.y = self.data['wy']
        twist.angular.z = self.data['wz']
        return twist
