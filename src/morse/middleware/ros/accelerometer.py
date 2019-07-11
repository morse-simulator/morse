from geometry_msgs.msg import TwistStamped, \
                              AccelWithCovarianceStamped
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


class AccelWithCovarianceStampedPublisher(ROSPublisher):

    ros_class = AccelWithCovarianceStamped

    def default(self, ci='unused'):
        accel = AccelWithCovarianceStamped()
        accel.header = self.get_ros_header()

        accel.accel.accel.linear.x = self.data['acceleration'][0]
        accel.accel.accel.linear.y = self.data['acceleration'][1]
        accel.accel.accel.linear.z = self.data['acceleration'][2]

        accel.accel.accel.angular.x = self.data['angular_acceleration'][0]
        accel.accel.accel.angular.y = self.data['angular_acceleration'][1]
        accel.accel.accel.angular.z = self.data['angular_acceleration'][2]

        self.publish(accel)
