import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Twist
from morse.middleware.ros import ROSPublisher

class TwistPublisher(ROSPublisher):

    def initialize(self):
        ROSPublisher.initialize(self, Twist)

    def default(self, ci='unused'):
        """ Publish the data of the acceleromter sensor as a ROS Twist message

        Only the velocity part is exported.
        """
        twist = Twist()

        # Fill twist-msg with the values from the sensor
        twist.linear.x = self.data['velocity'][0]
        twist.linear.y = self.data['velocity'][1]
        twist.linear.z = self.data['velocity'][2]

        self.publish(twist)
