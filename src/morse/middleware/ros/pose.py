import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import PoseStamped, Vector3
from morse.middleware.ros import ROSPublisher, ROSPublisherTF, mathutils

class PoseStampedPublisher(ROSPublisher):
    """ Publish the position and orientation of the robot. """
    ros_class = PoseStamped
    default_frame_id = '/map'

    def default(self, ci='unused'):
        pose = PoseStamped()
        pose.header = self.get_ros_header()

        pose.pose.position.x = self.data['x']
        pose.pose.position.y = self.data['y']
        pose.pose.position.z = self.data['z']

        euler = mathutils.Euler((self.data['roll'],
                                 self.data['pitch'],
                                 self.data['yaw']))
        pose.pose.orientation = euler.to_quaternion()

        self.publish(pose)


class TFPublisher(ROSPublisherTF):
    """ Publish the transformation between
    ``frame_id`` and ``child_frame_id`` args, default '/map' and
    '/base_link' through TF.
    """
    default_frame_id = '/map'

    def initialize(self):
        ROSPublisherTF.initialize(self)
        # store the frame ids
        self.child_frame_id = self.kwargs.get("child_frame_id", "/base_link")

        logger.info("Initialized the ROS TF publisher with frame_id '%s' " + \
                    "and child_frame_id '%s'", self.frame_id, self.child_frame_id)

    def default(self, ci='unused'):
        header = self.get_ros_header()

        # send current odometry transform
        self.sendTransform(self.get_position(),
                           self.get_orientation(),
                           header.stamp,
                           self.child_frame_id,
                           header.frame_id)


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

