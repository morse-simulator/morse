import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Twist
from morse.middleware.ros import ROSPublisher

class TwistPublisher(ROSPublisher):

    def initialize(self):
        ROSPublisher.initialize(self, Twist)

    def default(self, ci='unused'):
        """ Publish the velocity of the acceleromter sensor as a
        ``geometry_msgs/Twist`` message

        No angular information, only linear ones.
        """
        twist = Twist()

        # Fill twist-msg with the values from the sensor
        twist.linear.x = self.data['velocity'][0]
        twist.linear.y = self.data['velocity'][1]
        twist.linear.z = self.data['velocity'][2]

        self.publish(twist)
