from std_msgs.msg import Float32
from morse.middleware.ros import ROSPublisher

class Float32Publisher(ROSPublisher):
    """ Publish the charge of the battery sensor. """
    ros_class = Float32

    def default(self, ci='unused'):
        msg = Float32()
        msg.data = self.data['charge']

        self.publish(msg)
