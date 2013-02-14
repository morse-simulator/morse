import logging; logger = logging.getLogger("morse." + __name__)
import mathutils
import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import PoseStamped
from morse.middleware.ros import ROSPublisher

class PoseStampedPublisher(ROSPublisher):
    """ Publish the position and orientation of the robot. """
    _type_name = "geometry_msgs/PoseStamped"

    def initialize(self):
        ROSPublisher.initialize(self, PoseStamped)
        self.frame_id = self.kwargs.get("frame_id", "/map")

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
