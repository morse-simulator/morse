from geometry_msgs.msg import TwistStamped
from morse.middleware.ros import ROSPublisher

class TwistPublisher(ROSPublisher):
    """ Publish the velocity of the acceleromter sensor.
    No angular information, only linear ones.
    """
    ros_class = TwistStamped

    def default(self, ci='unused'):
        twist = TwistStamped()
        twist.header = self.get_ros_header()

        # Fill twist-msg with the values from the sensor
        twist.twist.linear.x = self.data['velocity'][0]
        twist.twist.linear.y = self.data['velocity'][1]
        twist.twist.linear.z = self.data['velocity'][2]

        self.publish(twist)
