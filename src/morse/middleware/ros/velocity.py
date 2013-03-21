import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('geometry_msgs');
from geometry_msgs.msg import TwistStamped
from morse.middleware.ros import ROSPublisher

class TwistStampedPublisher(ROSPublisher):
    """ Publish the twist of the robot. """
    ros_class = TwistStamped
    default_frame_id = '/map'

    def default(self, ci='unused'):
        twist = TwistStamped()
        twist.header = self.get_ros_header()
        twist.twist.linear.x = self.data['world_linear_velocity'][0]
        twist.twist.linear.y = self.data['world_linear_velocity'][1]
        twist.twist.linear.z = self.data['world_linear_velocity'][2]
        twist.twist.angular.x = self.data['angular_velocity'][0]
        twist.twist.angular.y = self.data['angular_velocity'][1]
        twist.twist.angular.z = self.data['angular_velocity'][2]

        self.publish(twist)
