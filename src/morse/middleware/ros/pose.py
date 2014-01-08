import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Vector3, Quaternion
from morse.middleware.ros import ROSPublisher, ROSPublisherTF, mathutils

class PosePublisherHelper():
    """ Class providing only helper functions to retrieve the pose
    for ROS Pose, PoseStamped and TF Publishers.
    """

    def get_orientation(self):
        """ Get the orientation from the local_data
        and return a ROS geometry_msgs.Quaternion
        """
        ros_quat = Quaternion()
        try:
            mathutils_quat = self.data['orientation']
        except KeyError:
            euler = mathutils.Euler((self.data['roll'],
                                     self.data['pitch'],
                                     self.data['yaw']))
            mathutils_quat = euler.to_quaternion()

        ros_quat.x = mathutils_quat.x
        ros_quat.y = mathutils_quat.y
        ros_quat.z = mathutils_quat.z
        ros_quat.w = mathutils_quat.w

        return ros_quat

    def get_position(self):
        """ Get the position from the local_data
        and return a ROS geometry_msgs.Vector3
        """
        position = Vector3()
        try:
            position.x = self.data['position'][0]
            position.y = self.data['position'][1]
            position.z = self.data['position'][2]
        except KeyError:
            position.x = self.data['x']
            position.y = self.data['y']
            position.z = self.data['z']

        return position

    def get_pose(self):
        """ Get the pose from the local_data
        and return a ROS geometry_msgs.Pose
        """
        pose = Pose()
        pose.position = self.get_position()
        pose.orientation = self.get_orientation()

        return pose


class PosePublisher(ROSPublisher, PosePublisherHelper):
    """ Publish the position and orientation of the robot as
    ROS geomeetry_msgs.Pose message.
    """
    ros_class = Pose

    def default(self, ci='unused'):

        try:
            publish = self.data['valid']
        except KeyError:
            publish = True

        if publish:
            pose = self.get_pose()
            self.publish(pose)


class PoseStampedPublisher(ROSPublisher, PosePublisherHelper):
    """ Publish the position and orientation of the robot
    as ROS geometry_msgs.PoseStamped message.
    """
    ros_class = PoseStamped
    default_frame_id = '/map'

    def default(self, ci='unused'):

        try:
            publish = self.data['valid']
        except KeyError:
            publish = True

        if publish:
            pose = PoseStamped()
            pose.header = self.get_ros_header()
            pose.pose = self.get_pose()
            self.publish(pose)


class PoseWithCovarianceStampedPublisher(ROSPublisher, PosePublisherHelper):
    """ Publish the position and orientation of the robot including the covariance. """
    ros_class = PoseWithCovarianceStamped
    default_frame_id = '/map'

    def default(self, ci='unused'):

        try:
            publish = self.data['valid']
        except KeyError:
            publish = True

        if publish:
            pose = PoseWithCovarianceStamped()
            pose.header = self.get_ros_header()
            pose.pose.pose = self.get_pose()
            try:
                pose.pose.covariance = self.data['covariance_matrix']
            except KeyError:
                pass
            self.publish(pose)


class TFPublisher(ROSPublisherTF, PosePublisherHelper):
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
        
        try:
            publish = self.data['valid']
        except KeyError:
            publish = True
            
        if publish:
            header = self.get_ros_header()
    
            # send current odometry transform
            self.sendTransform(self.get_position(),
                               self.get_orientation(),
                               header.stamp,
                               self.child_frame_id,
                               header.frame_id)
