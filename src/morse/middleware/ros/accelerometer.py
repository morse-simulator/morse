import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Twist
from morse.middleware.ros import ROSPublisher

class TwistPublisher(ROSPublisher):
    """ Publish the velocity of the acceleromter sensor.
    No angular information, only linear ones.
    """
    ros_class = Twist

    def default(self, ci='unused'):
        twist = Twist()

        # Fill twist-msg with the values from the sensor
        twist.linear.x = self.data['velocity'][0]
        twist.linear.y = self.data['velocity'][1]
        twist.linear.z = self.data['velocity'][2]

        self.publish(twist)
